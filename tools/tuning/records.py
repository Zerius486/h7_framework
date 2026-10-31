"""读取辨识记录，按时间划分训练与验证数据。"""

from pathlib import Path
import numpy as np


def load_record(path):
    """支持原始24字节记录及带表头的CSV，不修改原始文件。"""
    path = Path(path)
    if path.suffix.lower() == ".bin":
        dtype = [("time_ms", "<u4"), ("rx_count", "<u4"),
                 ("rx_age_ms", "<u4"), ("speed_rad_s", "<f4"),
                 ("actual_current_a", "<f4"), ("executed_current_a", "<f4")]
        if path.stat().st_size % 24:
            raise ValueError("BIN长度必须是24字节的整数倍")
        raw = np.fromfile(path, dtype=dtype)
    else:
        raw = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8-sig")
    if raw.ndim != 1 or len(raw) < 100:
        raise ValueError("至少需要100条连续记录")
    required = ("time_ms", "speed_rad_s", "actual_current_a")
    if not all(name in raw.dtype.names for name in required):
        raise ValueError("缺少字段：" + ", ".join(required))
    data = {name: np.asarray(raw[name], dtype=float) for name in raw.dtype.names}
    if any(not np.all(np.isfinite(value)) for value in data.values()):
        raise ValueError("记录中包含非有限数值")
    steps = np.diff(data["time_ms"]) / 1000
    dt = float(np.median(steps))
    if dt <= 0 or np.max(np.abs(steps - dt)) > max(1e-6, dt * .02):
        raise ValueError("需要固定周期、时间递增的记录；请先按完整连续数据段拆分")
    return data, dt


def windows(data, dt, window, delay, minimum_speed):
    """生成不跨过零、停转的非重叠梯形积分窗口及外生工具变量。"""
    count = max(1, int(round(window / dt)))
    omega = data["speed_rad_s"]
    current = data["actual_current_a"]
    reference = data.get("reference")
    x, y, instruments = [], [], []
    for first in range(delay, len(omega) - count, count):
        last = first + count
        w = omega[first:last + 1]
        if np.min(np.abs(w)) < minimum_speed or np.any(w * w[0] <= 0):
            continue
        i = current[first - delay:last + 1 - delay]
        duration = count * dt
        x.append([w[-1] - w[0], dt * (np.sum(w) - .5 * (w[0] + w[-1])),
                  np.sign(w[0]) * duration, duration])
        y.append(dt * (np.sum(i) - .5 * (i[0] + i[-1])))
        if reference is not None:
            # 仅使用外部参考的历史，不能拿闭环电流指令代替外生参考。
            z = [1.0]
            for lag in (0, count, 2 * count, 4 * count):
                index = max(0, first - lag)
                z.extend([reference[index], np.tanh(reference[index])])
            instruments.append(z)
    return np.asarray(x).reshape(-1, 4), np.asarray(y), np.asarray(instruments)


def fit_model(x, y, instruments=None):
    """枚举B、F非负约束；M不取绝对值或正值钳位。"""
    if len(y) < 20:
        raise ValueError("有效滑动窗口不足20个")
    scale = np.linalg.norm(x, axis=0)
    if np.any(scale == 0):
        raise ValueError("回归矩阵缺少激励")
    a = x / scale
    if instruments is not None:
        u, s, _ = np.linalg.svd(instruments, full_matrices=False)
        q = u[:, s > s[0] * 1e-8]
        a, y = q.T @ a, q.T @ y
        if np.linalg.matrix_rank(a, tol=1e-7) < 4:
            raise ValueError("外生参考未提供足够的独立工具变量")
    elif np.linalg.matrix_rank(a, tol=1e-7) < 4:
        raise ValueError("回归矩阵秩不足")
    best = None
    for free in ([0, 1, 2, 3], [0, 1, 3], [0, 2, 3], [0, 3]):
        coefficient = np.zeros(4)
        coefficient[free] = np.linalg.lstsq(a[:, free], y, rcond=1e-8)[0]
        theta = coefficient / scale
        if theta[1] < 0 or theta[2] < 0:
            continue
        loss = np.sum((a @ coefficient - y) ** 2)
        if best is None or loss < best[0]:
            best = loss, theta
    if best is None:
        raise ValueError("约束拟合失败")
    return best[1]


def velocity_prediction(data, dt, theta, delay, minimum_speed):
    """在独立数据上每250ms重置一次初值，段内不注入实测速度。"""
    m, b, friction, bias = theta
    if m <= 0:
        return {"accepted": False, "reason": "nonpositive inertia"}
    omega, current = data["speed_rad_s"], data["actual_current_a"]
    active, elapsed, error, energy, samples, segments = False, 0., 0., 0., 0, 0
    predicted = initial = previous = 0.
    for k in range(delay, len(omega)):
        w, i = omega[k], current[k - delay]
        if abs(w) < minimum_speed:
            active = False
            continue
        if not active or w * initial <= 0 or elapsed >= .25:
            predicted = initial = w
            previous, elapsed, active = i, 0., True
            segments += 1
            continue
        input_current = .5 * (i + previous) - bias
        direction = np.sign(predicted) if predicted != 0 else np.copysign(1., input_current)
        force = input_current - friction * direction
        rate = b / m
        gain = -np.expm1(-rate * dt) / rate if rate > 1e-9 else dt
        next_w = np.exp(-rate * dt) * predicted + gain * force / m
        if (predicted == 0 or predicted * next_w < 0) and abs(input_current) <= friction:
            next_w = 0.
        predicted, previous = next_w, i
        elapsed += dt
        error += (predicted - w) ** 2
        energy += (w - initial) ** 2
        samples += 1
    fitness = 1 - error / energy if energy > 1e-12 else None
    return {"rmse": float(np.sqrt(error / samples)) if samples else None,
            "fitness": float(fitness) if fitness is not None else None,
            "samples": samples, "segments": segments,
            "accepted": bool(samples * dt >= .5 and segments >= 4 and
                             fitness is not None and fitness >= .75)}


def identify(data, dt, window, delay, minimum_speed, split=.65):
    """固定时间切分，所有参数只用前段拟合，后段用于报告预测误差。"""
    cut = int(len(data["time_ms"]) * split)
    train = {key: value[:cut] for key, value in data.items()}
    held = {key: value[cut:] for key, value in data.items()}
    x, y, z = windows(train, dt, window, delay, minimum_speed)
    vx, vy, _ = windows(held, dt, window, delay, minimum_speed)
    result = {"window_s": window, "delay_samples": delay,
              "training_windows": len(y), "validation_windows": len(vy)}
    for method in ("least_squares", "instrumental_variables"):
        if method == "instrumental_variables" and "reference" not in data:
            result[method] = {"available": False, "reason": "missing independent reference"}
            continue
        try:
            theta = fit_model(x, y, z if method == "instrumental_variables" else None)
            residual = vy - vx @ theta
            energy = np.sum((vy - np.mean(vy)) ** 2) if len(vy) else 0.
            result[method] = {
                "available": True, "parameters": dict(zip(("M", "B", "F", "L"), map(float, theta))),
                "physical": bool(theta[0] > 0),
                "current_fitness": float(1 - np.sum(residual ** 2) / energy) if energy > 0 else None,
                "velocity": velocity_prediction(held, dt, theta, delay, minimum_speed)}
        except ValueError as exc:
            result[method] = {"available": False, "reason": str(exc)}
    return result


def frequency_response(data, dt, period_ticks, harmonics, settling_periods):
    """按完整周期，用外部参考作工具变量估计闭环内被控对象频响。"""
    if "reference" not in data:
        raise ValueError("缺少独立外部参考reference，不能用执行电流充当工具变量")
    n = period_ticks
    if n < 4 or len(set(harmonics)) != len(harmonics) or any(h <= 0 or 2*h >= n for h in harmonics):
        raise ValueError("周期长度或谐波配置无效")
    spectra = []
    for start in range(settling_periods * n, len(data["reference"]) - n + 1, n):
        stop = start + n
        if "sample" in data and not np.array_equal(data["sample"][start:stop], np.arange(n)):
            raise ValueError("周期与sample字段不对齐")
        if "limited" in data and np.any(data["limited"][start:stop]):
            continue
        spectra.append([np.fft.rfft(data[key][start:stop])[harmonics] for key in
                        ("reference", "actual_current_a", "speed_rad_s")])
    if len(spectra) < 2:
        raise ValueError("至少需要两个未发生限幅的完整测量周期")
    spec = np.asarray(spectra)
    r, i, w = spec[:, 0], spec[:, 1], spec[:, 2]
    ir, wr = np.mean(i * r.conj(), axis=0), np.mean(w * r.conj(), axis=0)
    ii, ww = np.mean(abs(i)**2, axis=0), np.mean(abs(w)**2, axis=0)
    wi = np.mean(w * i.conj(), axis=0)
    if np.any(abs(ir) < 1e-10) or np.any(np.mean(abs(r)**2, axis=0) < 1e-10):
        raise ValueError("部分谐波缺少参考或电流激励")
    g = wr / ir
    per_period = np.divide(w, i, out=np.full_like(w, np.nan), where=abs(i) > 1e-10)
    return {"period_ticks": n, "valid_periods": len(spec), "bins": [
        {"frequency_hz": h / (n * dt), "real": float(g[k].real), "imag": float(g[k].imag),
         "gain": float(abs(g[k])), "phase_deg": float(np.angle(g[k], deg=True)),
         "coherence": float(np.clip(abs(wi[k])**2 / max(ii[k]*ww[k], 1e-30), 0, 1)),
         "period_dispersion": (float(np.nanstd(per_period[:, k]))
                               if np.count_nonzero(np.isfinite(per_period[:, k])) >= 2 else None)}
        for k, h in enumerate(harmonics)]}
