#!/usr/bin/env python3
"""
命令行编译验证（不依赖 MounRiver IDE / 不触碰 obj/）。

使用 MRS2 自带 RISC-V GCC12 工具链，按工程编译选项编译全部源文件
并链接生成 ELF，用于在 IDE 之外快速验证代码改动。

用法:  python tools/build_check.py
"""
import subprocess
import sys
from pathlib import Path

PROJ = Path(__file__).resolve().parent.parent
GCC = Path(r"C:\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components"
           r"\WCH\Toolchain\RISC-V Embedded GCC12\bin\riscv-wch-elf-gcc.exe")
OUT = PROJ / "obj" / "build_check"

# 与 IDE 生成 makefile 一致的编译选项
CFLAGS = [
    "-march=rv32imac_xw", "-mabi=ilp32", "-msmall-data-limit=8", "-msave-restore",
    "-fmax-errors=20", "-O2", "-fmessage-length=0", "-fsigned-char",
    "-ffunction-sections", "-fdata-sections", "-fno-common", "-Wunused",
    "-Wuninitialized", "-Wformat", "-g", "-DDEBUG=0", "-DSDI_PRINT=0", "-std=gnu11",
]
INCLUDES = [
    "Debug", "Core", "User", "Peripheral/inc",
    "CherryUSB/core", "CherryUSB/common", "CherryUSB/class/cdc",
    "User/usb-cdc", "User/millis", "User/led-strip", "User/usb-pd", "User/apu",
]
# 与 IDE 构建一致的源文件集合（含本次新增 User/apu 三个文件）
SRC_FILES = [
    "CherryUSB/class/cdc/usbd_cdc_acm.c",
    "CherryUSB/core/usbd_core.c",
    "Core/core_riscv.c",
    "Debug/debug.c",
    "Peripheral/src/ch32x035_adc.c",
    "Peripheral/src/ch32x035_awu.c",
    "Peripheral/src/ch32x035_dbgmcu.c",
    "Peripheral/src/ch32x035_dma.c",
    "Peripheral/src/ch32x035_exti.c",
    "Peripheral/src/ch32x035_flash.c",
    "Peripheral/src/ch32x035_gpio.c",
    "Peripheral/src/ch32x035_i2c.c",
    "Peripheral/src/ch32x035_iwdg.c",
    "Peripheral/src/ch32x035_misc.c",
    "Peripheral/src/ch32x035_opa.c",
    "Peripheral/src/ch32x035_pwr.c",
    "Peripheral/src/ch32x035_rcc.c",
    "Peripheral/src/ch32x035_spi.c",
    "Peripheral/src/ch32x035_tim.c",
    "Peripheral/src/ch32x035_usart.c",
    "Peripheral/src/ch32x035_wwdg.c",
    "User/ch32x035_it.c",
    "User/cherryusb-port/usb_ch58x_dc_usbfs.c",
    "User/led-strip/led_strip.c",
    "User/main.c",
    "User/millis/millis.c",
    "User/system_ch32x035.c",
    "User/usb-cdc/usb_cdc_print.c",
    "User/usb-pd/usb_pd_cc.c",
    "User/usb-pd/usb_pd_log.c",
    "User/usb-pd/usb_pd_message.c",
    "User/usb-pd/usb_pd_monitor.c",
    "User/usb-pd/usb_pd_phy.c",
    "User/usb-pd/usb_pd_policy.c",
    "User/usb_vbus_measure.c",
    "User/apu/apu_io.c",
    "User/apu/apu_i2c.c",
    "User/apu/apu_amu.c",
    "User/apu/uart4_dbg.c",
]
STARTUP = "Startup/startup_ch32x035.S"
LINKER = "Ld/Link.ld"


def main():
    if not GCC.exists():
        print(f"toolchain not found: {GCC}")
        return 1
    OUT.mkdir(parents=True, exist_ok=True)

    def rel(p):
        return str(Path(p).relative_to(PROJ)).replace("\\", "/")

    sources = [PROJ / f for f in SRC_FILES]
    sources += [PROJ / STARTUP]

    objs = []
    failed = 0
    for src in sources:
        obj = OUT / (src.stem + ".o")
        cmd = [str(GCC)] + CFLAGS
        cmd += [f'-I"{(PROJ / inc)}"' for inc in INCLUDES]
        cmd += ["-c", "-o", str(obj), str(src)]
        # 路径含空格，用列表直接调用
        cmd = [str(GCC)] + CFLAGS + [f"-I{PROJ / inc}" for inc in INCLUDES] + \
              ["-c", "-o", str(obj), str(src)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            failed += 1
            print(f"FAIL {rel(src)}")
            print(r.stdout)
            print(r.stderr)
        else:
            objs.append(obj)

    if failed:
        print(f"\n{failed} file(s) failed to compile")
        return 1

    elf = OUT / "usb-pd-dp-amd.elf"
    lcmd = [str(GCC)] + CFLAGS + [
        f"-T{PROJ / LINKER}", "-nostartfiles", "-Xlinker", "--gc-sections",
        f"-Wl,-Map,{OUT / 'usb-pd-dp-amd.map'}",
        "--specs=nano.specs", "--specs=nosys.specs",
        "-Wl,--print-memory-usage", "-o", str(elf),
    ] + [str(o) for o in objs]
    r = subprocess.run(lcmd, capture_output=True, text=True)
    print(r.stdout)
    if r.returncode != 0:
        print("LINK FAIL")
        print(r.stderr)
        return 1

    size = GCC.with_name("riscv-wch-elf-size.exe")
    subprocess.run([str(size), str(elf)])
    print(f"\nOK: {len(objs)} sources compiled, link passed -> {elf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
