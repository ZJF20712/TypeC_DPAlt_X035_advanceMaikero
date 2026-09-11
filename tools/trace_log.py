#!/usr/bin/env python3
"""usb-pd-dp-amd 板载日志抓取工具
用法:
  python tools/trace_log.py            # 只解码已有的 logdump_*.bin
  python tools/trace_log.py --capture  # 编译产物在 /tmp/build 时: 烧录+复位+轮询抓取+解码
依赖: MounRiver Studio 2 自带的 OpenOCD + WCH-Link
说明: 固件在 pd_log_ram(2KB RAM 环形缓冲)中记录全部日志,
      本工具通过 halt/dump/resume 周期读取并解码。
"""
import glob, os, re, struct, subprocess, sys

TMP = r"C:/Users/12161/AppData/Local/Temp/build"
ELF = TMP + "/usb-pd-dp-amd.elf"
CAP = 2040
NM = (r"C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components"
      r"/WCH/Toolchain/RISC-V Embedded GCC12/bin/riscv-wch-elf-nm.exe")
OPENOCD = (r"C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components"
           r"/WCH/OpenOCD/OpenOCD/bin/openocd.exe")
CFG = (r"C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components"
       r"/WCH/OpenOCD/OpenOCD/bin/wch-riscv.cfg")


def ram_addr():
    out = subprocess.check_output([NM, ELF]).decode()
    m = re.search(r"^([0-9a-f]+) B pd_log_ram\s*$", out, re.M)
    return "0x" + m.group(1)


def capture(rounds=25, interval_ms=1800):
    addr = ram_addr()
    tcl = TMP + "/poll_log.tcl"
    with open(tcl, "w") as f:
        f.write(f'for {{set i 0}} {{$i < {rounds}}} {{incr i}} {{\n'
                f'    halt\n    sleep 40\n'
                f'    dump_image "{TMP}/logdump_$i.bin" {addr} 2048\n'
                f'    resume\n    sleep {interval_ms}\n}}\nshutdown\n')
    for f in glob.glob(TMP + "/logdump_*.bin"):
        os.remove(f)
    subprocess.run([OPENOCD, "-f", CFG, "-c", "init", "-c", "reset halt",
                    "-c", f"program {ELF} verify", "-c", "reset halt",
                    "-c", "resume", "-f", tcl])


def decode():
    files = sorted(glob.glob(TMP + "/logdump_*.bin"),
                   key=lambda f: int(re.search(r"_(\d+)\.bin", f).group(1)))
    last_wr, pending, lines = None, b"", []
    for f in files:
        d = open(f, "rb").read()
        rd, wr = struct.unpack("<II", d[:8])
        data = d[8:]
        if last_wr is None:
            last_wr = rd
        if wr <= last_wr:
            continue
        for idx in range(max(last_wr, wr - CAP), wr):
            pending += bytes([data[idx % CAP] & 0xFF])
            if pending.endswith(b"\n"):
                lines.append(pending.decode("utf-8", "replace").rstrip("\r\n"))
                pending = b""
        last_wr = wr
    if pending:
        lines.append(pending.decode("utf-8", "replace"))
    print(f"total lines: {len(lines)}")
    print("\n".join(lines))


if __name__ == "__main__":
    if "--capture" in sys.argv:
        capture()
    decode()
