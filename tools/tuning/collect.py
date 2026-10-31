"""收集H7调试日志，保存原始帧、英文日志和可直接分析的CSV。"""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None

MAGIC, VERSION, SAMPLE, TEXT = 0xA5, 1, 1, 2
HEADER = struct.Struct("<BBBBH")
SAMPLE_STRUCT = struct.Struct("<I H B B B B 2x 8f")


def crc16(data):
    """计算与板端相同的Modbus CRC16。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def write_sample(writer, payload):
    """解码并保存一个波形记录。"""
    if len(payload) != SAMPLE_STRUCT.size:
        return False
    time_ms, sequence, stage, flags, limited, reserved, *values = SAMPLE_STRUCT.unpack(payload)
    writer.writerow((time_ms, sequence, stage, flags, limited, reserved, *values))
    return True


def feed_bytes(buffer, chunk, writer, text, previous_sequence=None, statistics=None):
    """解析一段串口数据，保留尚未完整的尾帧。"""
    frames, samples, bad_crc, sequence_gaps = 0, 0, 0, 0
    buffer.extend(chunk)
    while len(buffer) >= HEADER.size:
        if buffer[0] != MAGIC:
            text.write(f"RAW_SYNC_DROP byte=0x{buffer.pop(0):02X}\n")
            continue
        _, version, frame_type, length, sequence = HEADER.unpack(buffer[:HEADER.size])
        total = HEADER.size + length + 2
        if version != VERSION or length > 96:
            text.write(f"FRAME_REJECT sequence={sequence} reason=header\n")
            del buffer[0]
            continue
        if len(buffer) < total:
            break
        frame = bytes(buffer[:total])
        del buffer[:total]
        if crc16(frame[:-2]) != struct.unpack_from("<H", frame, total - 2)[0]:
            bad_crc += 1
            text.write(f"FRAME_REJECT sequence={sequence} reason=crc\n")
            continue
        payload = frame[HEADER.size:-2]
        message = payload.decode("ascii", errors="replace") if frame_type == TEXT else None
        if message is not None and re.match(r"^\[\d+\] INFO RUN_START MODE=", message):
            previous_sequence = None
        if previous_sequence is not None:
            gap = (sequence - previous_sequence - 1) & 0xFFFF
            if gap < 0x8000:
                sequence_gaps += gap
                if gap:
                    text.write(f"FRAME_GAP previous={previous_sequence} sequence={sequence} missing={gap}\n")
        previous_sequence = sequence
        frames += 1
        if frame_type == SAMPLE:
            valid = write_sample(writer, payload)
            samples += int(valid)
        elif frame_type == TEXT:
            text.write(message)
            if statistics is not None:
                summary = re.search(r"\bRUN_STOP DROPPED=(\d+) TEXT_DROPPED=(\d+)", message)
                if summary:
                    statistics["firmware_dropped_frames"] += int(summary.group(1))
                    statistics["firmware_dropped_text_frames"] += int(summary.group(2))
                    statistics["run_summaries"] += 1
            text.flush()
    return frames, samples, bad_crc, sequence_gaps, previous_sequence


def collect(port, baudrate, output, duration, metadata):
    """读取串口并按一次运行创建不可覆盖的结果目录。"""
    if serial is None:
        raise RuntimeError("需要安装pyserial：python -m pip install pyserial")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    meta = dict(metadata)
    meta.update({"format_version": 1, "started_utc": datetime.now(timezone.utc).isoformat(),
                 "port": port, "baudrate": baudrate})
    (output / "metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with (output / "raw.bin").open("wb") as raw, (output / "events.log").open("w", encoding="utf-8") as text, \
            (output / "samples.csv").open("w", newline="", encoding="utf-8") as csv_file, \
            serial.Serial(port, baudrate=baudrate, timeout=.1) as device:
        writer = csv.writer(csv_file)
        writer.writerow(("time_ms", "sequence", "stage", "flags", "limited", "reserved",
                         "reference", "position_rad", "speed_rad_s", "actual_current_a",
                         "requested_current_a", "executed_current_a", "model_inertia", "model_fitness"))
        buffer = bytearray()
        deadline = time.monotonic() + duration if duration > 0 else None
        frames, samples, bad_crc, sequence_gaps = 0, 0, 0, 0
        previous_sequence = None
        statistics = {"firmware_dropped_frames": 0,
                      "firmware_dropped_text_frames": 0, "run_summaries": 0}
        interrupted = False
        try:
            while deadline is None or time.monotonic() < deadline:
                chunk = device.read(512)
                if not chunk:
                    continue
                raw.write(chunk)
                frame_count, sample_count, bad_count, gap_count, previous_sequence = \
                    feed_bytes(buffer, chunk, writer, text, previous_sequence, statistics)
                frames += frame_count
                samples += sample_count
                bad_crc += bad_count
                sequence_gaps += gap_count
        except KeyboardInterrupt:
            interrupted = True
        raw.flush()
        csv_file.flush()
        text.flush()
        meta.update({"frames": frames, "samples": samples, "bad_crc": bad_crc,
                     "sequence_gaps": sequence_gaps, "interrupted": interrupted,
                     "ended_utc": datetime.now(timezone.utc).isoformat()})
        meta.update(statistics)
        if not statistics["run_summaries"]:
            meta["firmware_dropped_frames"] = None
            meta["firmware_dropped_text_frames"] = None
        (output / "metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return meta


def main():
    """启动一次可选的串口采集。"""
    parser = argparse.ArgumentParser(description="Collect H7 tuning logs to local files.")
    parser.add_argument("port")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0, help="秒；0表示持续到Ctrl-C")
    parser.add_argument("--metadata", type=Path)
    args = parser.parse_args()
    try:
        if args.baudrate <= 0 or args.duration < 0:
            raise ValueError("baudrate必须为正，duration不能为负")
        metadata = json.loads(args.metadata.read_text(encoding="utf-8")) if args.metadata else {}
        print(json.dumps(collect(args.port, args.baudrate, args.output, args.duration, metadata), ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
