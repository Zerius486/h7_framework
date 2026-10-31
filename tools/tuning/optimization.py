"""三维物理参数的序贯多目标建议，不连接硬件。"""

import json
import numpy as np

METRICS = ("rms_error", "overshoot", "settling_time", "idle_ripple", "rms_slew", "saturation")


def hypervolume(points, reference):
    """计算最小化问题中二维支配区域的面积。"""
    points = np.asarray(points).reshape(-1, 2)
    points = points[np.all(points < reference, axis=1)]
    area, top = 0., reference[1]
    for x, y in points[np.argsort(points[:, 0])]:
        if y < top:
            area += (reference[0] - x) * (top - y)
            top = y
    return area


def kernel(a, b, length):
    """在归一化参数范围中计算Matérn 5/2协方差。"""
    radius = np.sqrt(np.sum(((a[:, None] - b[None, :]) / length)**2, axis=2))
    return (1 + np.sqrt(5)*radius + 5*radius**2/3) * np.exp(-np.sqrt(5)*radius)


def validate_trial(trial, bounds):
    """检查搜索与最终确认记录，拒绝缺失或无效指标。"""
    if trial.get("stage", "search") not in ("search", "confirmation"):
        raise ValueError("stage必须为search或confirmation")
    theta = np.asarray(trial["theta"], dtype=float)
    if (theta.shape != (3,) or np.any(~np.isfinite(theta)) or
            np.any(theta < bounds[:, 0]) or np.any(theta > bounds[:, 1])):
        raise ValueError("trial.theta超出配置范围")
    metrics = trial["metrics"]
    values = np.asarray([metrics[name] for name in METRICS], dtype=float)
    if (np.any(~np.isfinite(values)) or np.any(values < 0) or
            not isinstance(metrics["settled"], bool)):
        raise ValueError("性能指标必须为非负有限数值，settled必须为布尔值")
    return theta, metrics


def propose(config, trials, candidate_points=None):
    """联合采样历史点与候选后验，估计含噪声的单点超体积改进。"""
    bounds = np.asarray(config["bounds"], dtype=float)
    goals = config.get("objectives", ["rms_error", "rms_slew"])
    reference = np.asarray(config["reference"], dtype=float)
    if (bounds.shape != (3, 2) or np.any(~np.isfinite(bounds)) or
            np.any(bounds[:, 1] < bounds[:, 0]) or reference.shape != (2,) or
            np.any(~np.isfinite(reference)) or np.any(reference <= 0) or
            len(goals) != 2 or len(set(goals)) != 2 or any(g not in METRICS for g in goals)):
        raise ValueError("需要三个有效参数范围、两个不同指标及正的超体积参考点")
    if bounds[0, 0] <= 0 or bounds[1, 0] <= 0 or bounds[2, 0] < 0:
        raise ValueError("带宽、阻尼必须为正，前馈比例必须非负")
    limits = config.get("limits", {})
    if any(k not in METRICS or not np.isfinite(v) or v < 0 for k, v in limits.items()):
        raise ValueError("limits只能包含六项非负有限指标")
    length = float(config.get("length_scale", .3))
    draws = int(config.get("posterior_samples", 64))
    pool_size = int(config.get("pool_size", 256))
    if not np.isfinite(length) or length <= 0 or not 16 <= draws <= 512 or not 32 <= pool_size <= 1024:
        raise ValueError("长度尺度必须为正；后验采样数16～512，候选池32～1024")
    rng = np.random.default_rng(int(config.get("seed", 0)))
    span = bounds[:, 1] - bounds[:, 0]
    varying = span > 0
    scale = np.where(varying, span, 1.)
    # 分层随机初始池，固定维度始终取零。
    pool = np.zeros((pool_size, 3))
    for d in np.flatnonzero(varying):
        pool[:, d] = (rng.permutation(pool_size) + rng.random(pool_size)) / pool_size
    pool[0] = .5 * varying
    if candidate_points is not None:
        points = np.asarray(candidate_points, dtype=float).reshape(-1, 3)
        if np.any(~np.isfinite(points)) or np.any(points < bounds[:, 0]) or np.any(points > bounds[:, 1]):
            raise ValueError("回放候选超出配置范围")
        pool = (points - bounds[:, 0]) / scale
    constraints = [m for m in METRICS if limits.get(m, 0) > 0]
    names = list(dict.fromkeys(goals + constraints))
    grouped = {}
    contexts = {json.dumps(t.get("context"), sort_keys=True) for t in trials}
    if len(contexts) > 1:
        raise ValueError("不同工况不能混入同一次优化，请先按context拆分记录")
    for trial in trials:
        theta, metrics = validate_trial(trial, bounds)
        # 独立确认轨迹仅作报告，不用于下一候选建议。
        if trial.get("stage", "search") != "search":
            continue
        y = [float(metrics[name]) for name in names] + [0. if metrics["settled"] else 1.]
        grouped.setdefault(tuple(theta), []).append(y)
    x = np.asarray([(np.asarray(t) - bounds[:, 0]) / scale for t in grouped]).reshape(-1, 3)
    if len(x):
        pool = pool[np.min(np.linalg.norm(pool[:, None] - x[None, :], axis=2), axis=1) > 1e-6]
    if len(pool) == 0:
        return {"available": False, "reason": "参数范围已无新的候选点"}
    if len(x) < max(4, 1 + 2 * np.sum(varying)):
        index = 0 if not len(x) else int(np.argmax(np.min(
            np.linalg.norm(pool[:, None] - x[None, :], axis=2), axis=1)))
        method, acquisition = "space_filling", None
    else:
        values = [np.asarray(grouped[t]) for t in grouped]
        y = np.asarray([v.mean(axis=0) for v in values])
        noise = np.asarray([v.var(axis=0, ddof=1)/len(v) if len(v) > 1 else
                            np.zeros(y.shape[1]) for v in values])
        # 单次观测的噪声采用明确配置；重复试验会补充样本方差。
        relative_noise = float(config.get("relative_noise", .03))
        if not np.isfinite(relative_noise) or relative_noise <= 0:
            raise ValueError("relative_noise必须为正")
        mean = y.mean(axis=0)
        std = np.maximum(y.std(axis=0), np.maximum(abs(mean)*.1, 1e-6))
        xn = np.concatenate((x, pool))
        k = kernel(x, x, length)
        cross = kernel(x, xn, length)
        prior = kernel(xn, xn, length)
        samples = []
        for j in range(y.shape[1]):
            variance = noise[:, j]/std[j]**2 + relative_noise**2
            chol = np.linalg.cholesky(k + np.diag(variance))
            v = np.linalg.solve(chol, cross)
            mu = cross.T @ np.linalg.solve(chol.T, np.linalg.solve(chol, (y[:, j]-mean[j])/std[j]))
            covariance = prior - v.T @ v
            covariance = .5*(covariance + covariance.T) + np.eye(len(xn))*1e-8
            posterior = mu + rng.standard_normal((draws, len(xn))) @ np.linalg.cholesky(covariance).T
            samples.append(posterior*std[j] + mean[j])
        samples = np.maximum(np.stack(samples, axis=2), 0.)
        objective_index = [names.index(g) for g in goals]
        feasible = samples[:, :, -1] < .5
        for name in constraints:
            feasible &= samples[:, :, names.index(name)] <= limits[name]
        objective = samples[:, :, objective_index] / reference
        improvement = np.zeros(len(pool))
        for s in range(draws):
            baseline = objective[s, :len(x)][feasible[s, :len(x)]]
            hv = hypervolume(baseline, np.ones(2))
            for p in np.flatnonzero(feasible[s, len(x):]):
                value = hypervolume(np.vstack((baseline, objective[s, len(x)+p])), np.ones(2))
                improvement[p] += max(0., value-hv) / draws
        index = int(np.argmax(improvement))
        if np.max(improvement) <= 1e-12:
            index = int(np.argmax(np.min(np.linalg.norm(pool[:, None] - x[None, :], axis=2), axis=1)))
        method, acquisition = "sampled_noisy_hypervolume_improvement", float(improvement[index])
    return {"available": True, "method": method,
            "theta": (bounds[:, 0] + pool[index]*span).tolist(),
            "objectives": goals, "acquisition": acquisition,
            "measured_parameters": len(x), "verified": False}


def replay(config, trials, budget):
    """仅在完整实测候选表内回放，不能预测未测候选的实验结果。"""
    measured = {}
    confirmation = {}
    for trial in trials:
        target = confirmation if trial.get("stage") == "confirmation" else measured
        if trial.get("stage", "search") not in ("search", "confirmation"):
            raise ValueError("stage必须为search或confirmation")
        target.setdefault(tuple(trial["theta"]), []).append(trial)
    if not 1 <= budget <= len(measured):
        raise ValueError("回放预算必须在1与实测参数组数之间")
    goals = config.get("objectives", ["rms_error", "rms_slew"])
    limits = config.get("limits", {})
    reference = np.asarray(config["reference"], dtype=float)
    keys = list(measured)
    # 先调用一次完成配置、工况及所有指标的检查。
    propose(config, trials, keys)
    rng = np.random.default_rng(int(config.get("seed", 0)))
    random_order = list(rng.permutation(len(keys)))
    output = {}
    for method in ("recorded_order", "random", "bayesian"):
        observed, selected, history = [], [], []
        for step in range(budget):
            if method == "bayesian":
                suggestion = propose(config, observed, keys)
                if not suggestion["available"]:
                    break
                theta = np.asarray(suggestion["theta"])
                key = min((k for k in keys if k not in selected), key=lambda k: np.linalg.norm(theta-k))
            else:
                key = keys[step if method == "recorded_order" else random_order[step]]
            selected.append(key)
            observed.extend(measured[key])
            points = []
            confirmed = 0
            for k in selected:
                runs = confirmation.get(k, [])
                if not runs:
                    continue
                confirmed += 1
                # 独立确认的所有重复均满足约束，才计入确认前沿。
                valid = all(r["metrics"]["settled"] and all(
                    r["metrics"][name] <= value for name, value in limits.items() if value > 0)
                            for r in runs)
                if valid:
                    points.append([max(r["metrics"][g] for r in runs) / reference[j]
                                   for j, g in enumerate(goals)])
            history.append({"step": step + 1, "confirmed_parameters": confirmed,
                            "confirmation_hypervolume": hypervolume(points, np.ones(2))})
        output[method] = {"selected": [list(k) for k in selected], "history": history}
    return {"scope": "finite measured candidate table; unseen candidates are not evaluated",
            "has_independent_confirmation": bool(confirmation), "strategies": output}
