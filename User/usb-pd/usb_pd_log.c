#include "usb_pd_log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ch32x035.h"
#include "millis.h"
#include "uart4_dbg.h"
#include "usb_cdc_print.h"

/* 单条日志缓冲区大小 */
#define PD_LOG_BUF_SIZE 256

/* SDI 输出总开关：默认关闭——无人监视时每段日志阻塞 2ms，多行累计
 * 数十 ms 会拖慢 PD 响应时序（超出 tSenderResponse 窗口）。
 * 调试请用 RAM 日志环（tools/trace_log.py）；如需实时 SDI 监视置 1 重编译 */
#define PD_LOG_SDI_ENABLE 0

/* SDI 单段等待 DATA0 清零的超时（调试器不在场时快速降级） */
#define PD_SDI_SEG_TIMEOUT_MS 2
/* 连续失败次数达到后禁用 SDI 输出（重新上电或 pd_log_init 后重试） */
#define PD_SDI_FAIL_LIMIT 5

static bool sdi_disabled = false;
static uint8_t sdi_fail_cnt = 0;

/* 调试模块 DMDATA0/DMDATA1（与 WCH SDI Printf 协议一致） */
#define PD_SDI_DATA0 ((volatile uint32_t *)0xE0000380)
#define PD_SDI_DATA1 ((volatile uint32_t *)0xE0000384)

/*
 * RAM 日志环形缓冲（调试器通过 halt + dump_image 抓取）
 * 布局：[0]rd [1]wr [2..]数据；rd/wr 为累积计数（不取模），数据位于 buf[i % N]
 */
#define PD_LOG_RAM_SIZE 2048

volatile uint32_t pd_log_ram[2 + (PD_LOG_RAM_SIZE / 4)];
#define PD_LOG_RAM_DATA ((char *)&pd_log_ram[2])
#define PD_LOG_RAM_CAP  (PD_LOG_RAM_SIZE - 8)

/**
 * @brief  等待 DMDATA0 被调试器清零（空闲）
 * @return true:空闲可写 false:超时（调试器未读取）
 */
static bool sdi_wait_free(void) {
    uint32_t start = millis();

    while (*PD_SDI_DATA0 != 0u) {
        if (millis() - start > PD_SDI_SEG_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  按 WCH SDI Printf 协议输出（每段 7 字节：DATA0 低字节为长度）
 * @note   每行独立尝试：调试器未读取时仅放弃本次输出，不禁用通道，
 *         调试器随时接入即可恢复接收
 */
static void sdi_write(const char *buf, uint16_t len) {
    uint16_t i = 0;

#if PD_LOG_SDI_ENABLE
    if (sdi_disabled) {
        return;
    }
#else
    return; /* SDI 输出关闭：零开销 */
#endif
    while (i < len) {
        uint8_t n = (len - i) > 7 ? 7 : (uint8_t)(len - i);
        uint8_t seg[7] = {0};

        memcpy(seg, &buf[i], n);

        if (!sdi_wait_free()) {
            /* Debugger does not read for a long time: disable SDI output to avoid accumulated per-line log
             * blocking (each line up to tens of ms) slowing down the PD response timing */
            if (++sdi_fail_cnt >= PD_SDI_FAIL_LIMIT) {
                sdi_disabled = true;
            }
            return;
        }
        sdi_fail_cnt = 0;

        *PD_SDI_DATA1 = (uint32_t)seg[3] | ((uint32_t)seg[4] << 8) | ((uint32_t)seg[5] << 16) | ((uint32_t)seg[6] << 24);
        *PD_SDI_DATA0 = (uint32_t)n | ((uint32_t)seg[0] << 8) | ((uint32_t)seg[1] << 16) | ((uint32_t)seg[2] << 24);

        i += n;
    }
}

/**
 * @brief  写入 RAM 日志环形缓冲（供调试器 halt + dump 抓取）
 */
static void ram_write(const char *buf, uint16_t len) {
    uint32_t wr = pd_log_ram[1];

    for (uint16_t i = 0; i < len; i++) {
        PD_LOG_RAM_DATA[(wr + i) % PD_LOG_RAM_CAP] = buf[i];
    }
    pd_log_ram[1] = wr + len;
    /* 防止读指针被覆盖（正常不会发生：日志速率远低于读取速率） */
    if (pd_log_ram[1] - pd_log_ram[0] > PD_LOG_RAM_CAP) {
        pd_log_ram[0] = pd_log_ram[1] - PD_LOG_RAM_CAP;
    }
}

/**
 * @brief  CDC 输出（仅已枚举且 DTR 打开，否则跳过防止 ep_tx_busy 死等）
 * @note   I2C 占用 PC16/17 后不再初始化 CDC，此路自然静默，保留兼容
 */
static void cdc_write(const char *str) {
    if (cdc_acm_is_configured() && cdc_acm_get_dtr()) {
        cdc_acm_prints((char *)str);
    }
}

/**
 * @brief  UART4 输出（PA5，TXE 中断非阻塞；未初始化时内部直接跳过）
 */
static void uart4_write(const char *buf, uint16_t len) {
    uart4_dbg_write(buf, len);
}

void pd_log_init(void) {
    *PD_SDI_DATA0 = 0;
    pd_log_ram[0] = 0;
    pd_log_ram[1] = 0;
}

void pd_logf(const char *format, ...) {
    static char buf[PD_LOG_BUF_SIZE];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buf, sizeof(buf) - 1, format, args);
    va_end(args);

    if (len <= 0) {
        return;
    }
    if (len > (int)sizeof(buf) - 1) {
        len = (int)sizeof(buf) - 1;
    }

    ram_write(buf, (uint16_t)len);
    sdi_write(buf, (uint16_t)len);
    cdc_write(buf);
    uart4_write(buf, (uint16_t)len);
}
