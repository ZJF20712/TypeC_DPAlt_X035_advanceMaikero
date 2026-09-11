#!/usr/bin/env python3
"""COM26 串口抓取: 带时间戳落盘 + 控制台回显
用法: python tools/serial_capture.py [COM口] [超时秒]
默认 COM26 115200, 输出 tools/uart_capture_<ts>.log
Ctrl+C 或超时后退出。
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM26"
TIMEOUT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 600

ts = time.strftime("%Y%m%d_%H%M%S")
out_path = rf"E:\wch_pd\usb-pd-dp-amd\tools\uart_capture_{ts}.log"

ser = serial.Serial(PORT, 115200, timeout=0.2)
print(f"capturing {PORT} @115200 -> {out_path} (timeout {TIMEOUT_S}s)", flush=True)
start = time.time()
with open(out_path, "w", encoding="utf-8", errors="replace") as f:
    while time.time() - start < TIMEOUT_S:
        data = ser.read(512)
        if data:
            stamp = f"[{time.time() - start:8.3f}] "
            text = data.decode("utf-8", errors="replace")
            f.write(stamp + text)
            f.flush()
            print(stamp + text, end="", flush=True)
ser.close()
print(f"\ncapture ended -> {out_path}", flush=True)
