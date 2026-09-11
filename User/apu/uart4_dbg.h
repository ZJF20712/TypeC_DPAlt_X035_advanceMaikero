#pragma once

#include <stdint.h>

#include "apu_config.h"

/* ============================================================
 * UART4 调试口（PA5 = TX，115200 8N1，TXE 中断非阻塞发送）
 * 与灯带互斥复用 PA5（UART4_TX / SPI1_SCK，QNF20 无第三组引脚），
 * 由 APU_DBG_UART4_ENABLE 选择：
 *   1 = 串口调试输出（COM26 日志，灯带停用）
 *   0 = LED 状态灯模式——本模块全部退化为空函数，调用点零开销
 * ============================================================ */

#if APU_DBG_UART4_ENABLE

/**
 * @brief  初始化 UART4 调试口（PA5 复用推挽，TXE 中断驱动）
 * @note   启用后灯带 SPI 不可用（main.c 按宏跳过灯带初始化）
 */
void uart4_dbg_init(void);

/**
 * @brief  非阻塞写入发送环形缓冲，缓冲满时丢弃并计数
 */
void uart4_dbg_write(const char *data, uint16_t len);

/**
 * @brief  丢弃计数值（诊断日志洪泛）
 */
uint32_t uart4_dbg_dropped(void);

#if APU_DBG_UART4_REMAP_SCAN
/**
 * @brief  REMAP 校准任务：每 2s 轮换一个安全 USART4_RM 编码发横幅，
 *         在 COM26 上观察到可读横幅即锁定对应编码（随后更新
 *         APU_DBG_UART4_REMAP 并关闭本开关）。仅轮换不触碰
 *         PC16/17(I2C) 与 SWD 引脚的编码
 */
void uart4_dbg_remap_scan_task(void);
#endif

#else /* !APU_DBG_UART4_ENABLE: 调试函数退化为空函数，调用点零开销 */

static inline void uart4_dbg_init(void) {
}

static inline void uart4_dbg_write(const char *data, uint16_t len) {
    (void)data;
    (void)len;
}

static inline uint32_t uart4_dbg_dropped(void) {
    return 0u;
}

#if APU_DBG_UART4_REMAP_SCAN
static inline void uart4_dbg_remap_scan_task(void) {
}
#endif

#endif /* APU_DBG_UART4_ENABLE */
