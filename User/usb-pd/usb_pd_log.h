#pragma once

#include <stdint.h>

/**
 * @brief  初始化日志模块（SDI 通道清零，需在 millis_init 之后调用）
 */
void pd_log_init(void);

/**
 * @brief  统一日志输出（多路）
 *         1. RAM 日志环（调试器 halt + dump 抓取，tools/trace_log.py）
 *         2. SDI / SWD 虚拟串口（默认关闭，见 usb_pd_log.c 开关）
 *         3. USB CDC（PC16/17 让给 I2C 后不再初始化，此路自然静默）
 *         4. UART4 @ PA5（TXE 中断非阻塞，主调试口，COM26）
 * @note   仅可在主循环上下文调用（中断内调用可能阻塞）
 * @note   format 属性让编译器检查格式串与实参匹配（可变参数无类型检查，
 *         错位只能靠它拦截——本次实测踩坑：缺 millis() 导致日志整体错位）
 */
void pd_logf(const char *format, ...) __attribute__((format(printf, 1, 2)));
