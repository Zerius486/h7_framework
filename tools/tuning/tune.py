"""离线辨识、闭环频响和多目标候选建议入口。"""

import argparse
import json
from pathlib import Path
import sys

from records import load_record, identify, frequency_response
from optimization import propose, replay


def read_json(path):
    """读取配置JSON或逐行试验JSON，不执行其中内容。"""
    text = Path(path).read_text(encoding="utf-8-sig")
    if Path(path).suffix == ".jsonl":
        return [json.loads(line) for line in text.splitlines() if line.strip()]
    return json.loads(text)


def main():
    """运行离线命令；只向指定输出文件写入新结果。"""
    parser = argparse.ArgumentParser(description="Offline system identification and tuning tools.")
    sub = parser.add_subparsers(dest="command", required=True)
    identification = sub.add_parser("identify", help="窗口/延迟及LS/IV对照")
    identification.add_argument("record", type=Path)
    identification.add_argument("--windows", type=float, nargs="+", default=[.01, .02, .04])
    identification.add_argument("--delays", type=int, nargs="+", default=[0])
    identification.add_argument("--min-speed", type=float, required=True)
    identification.add_argument("--split", type=float, default=.65)
    spectral = sub.add_parser("frequency", help="完整多正弦周期的闭环频响")
    spectral.add_argument("record", type=Path)
    spectral.add_argument("--period-ticks", type=int, required=True)
    spectral.add_argument("--harmonics", type=int, nargs="+", required=True)
    spectral.add_argument("--settling-periods", type=int, default=1)
    optimization = sub.add_parser("propose", help="从历史试验建议下一组参数")
    optimization.add_argument("config", type=Path)
    optimization.add_argument("trials", type=Path)
    comparison = sub.add_parser("replay", help="在完整实测候选表中比较搜索次序")
    comparison.add_argument("config", type=Path)
    comparison.add_argument("trials", type=Path)
    comparison.add_argument("--budget", type=int, required=True)
    for p in (identification, spectral, optimization, comparison):
        p.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        if args.command in ("propose", "replay"):
            if args.output and args.output.resolve() in (args.config.resolve(), args.trials.resolve()):
                raise ValueError("输出不能覆盖配置或原始试验")
            config, trials = read_json(args.config), read_json(args.trials)
            result = (propose(config, trials) if args.command == "propose" else
                      replay(config, trials, args.budget))
        else:
            if args.output and args.output.resolve() == args.record.resolve():
                raise ValueError("输出不能覆盖原始记录")
            data, dt = load_record(args.record)
            if args.command == "identify":
                if (not .2 <= args.split <= .8 or not 0 < args.min_speed < float("inf") or
                        any(not dt <= w <= .1 for w in args.windows) or
                        any(not 0 <= d <= 32 for d in args.delays)):
                    raise ValueError("切分比例0.2～0.8，最低速度为正，窗口dt～0.1s，延迟0～32")
                result = {"source": str(args.record.resolve()), "sampling_time": dt,
                          "split": args.split, "comparisons": [
                              identify(data, dt, w, d, args.min_speed, args.split)
                              for w in args.windows for d in args.delays]}
            else:
                if args.settling_periods < 1:
                    raise ValueError("至少保留一个过渡周期")
                result = frequency_response(data, dt, args.period_ticks, args.harmonics, args.settling_periods)
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        if args.output:
            args.output.write_text(text + "\n", encoding="utf-8")
        else:
            print(text)
        return 0
    except (ValueError, KeyError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
