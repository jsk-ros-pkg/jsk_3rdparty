#!/usr/bin/env python3

import os
import sys
import serial
import threading
import time
import datetime
from colorama import Fore, Style, init

init(autoreset=True)  # 色設定のリセット自動

def read_serial(port, baud, color, label=None):
    if label is None:
        label = os.path.basename(port)
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        while running:
            line = ser.readline()
            if line:
                text = line.decode(errors='ignore').strip()
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")
                # 端末出力（色付き）
                print(f"{color}[{timestamp}] [{label}] {text}{Style.RESET_ALL}")
    except serial.SerialException as e:
        print(f"{label} ERROR: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python dual_serial_logger.py <PORT1> <PORT2> <BAUD>")
        sys.exit(1)

    port1 = sys.argv[1]
    port2 = sys.argv[2]
    baud = int(sys.argv[3])

    running = True

    t1 = threading.Thread(target=read_serial, args=(port1, baud, Fore.GREEN))
    t2 = threading.Thread(target=read_serial, args=(port2, baud, Fore.BLUE))

    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        running = False
        print("\nStopping...")
        t1.join()
        t2.join()
