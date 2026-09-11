#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  初始化 APU 侧 GPIO：HPD(PB1 输出低) / APU_RST#(PB3 输入) / INT(PB11 输入+EXTI)
 * @note   上电即输出 HPD=0（板载 100K 下拉兜底），满足 AMD "VDD_33 未上电
 *         时 HPD 不得为高" 的要求（APU_RST# 门控另见 apu_io_hpd_request）
 */
void apu_io_init(void);

/**
 * @brief  HPD 请求置位/清零（DP Alt Mode 事件，由上层策略机调用）
 * @param  on true: 请求拉高 HPD；false: 立即拉低
 * @note   拉高需同时满足: 上层已确认 DP 模式生效 + APU_RST# 已释放(S0)。
 *         条件不满足时保持低并记录，待条件满足后由 apu_io_process 补齐
 */
void apu_io_hpd_request(bool on);

/**
 * @brief  IRQ_HPD 短脉冲（DisplayPort 规范 50~500us 低脉冲）
 * @note   仅在 HPD 已为高时有效；内部阻塞约 500us，勿在中断上下文调用
 */
void apu_io_hpd_irq_pulse(void);

/**
 * @brief  读取 APU_RST# 当前电平
 * @return true: APU_RST# 有效(低, 复位/睡眠中)  false: 已释放(S0)
 */
bool apu_io_apu_rst_asserted(void);

/**
 * @brief  周期处理（主循环调用）：按 APU_RST# 状态维护 HPD 门控输出
 * @note   APU_RST# 有效(复位/睡眠)期间强制 HPD=低
 */
void apu_io_process(void);

/**
 * @brief  读取并清除 INT 中断事件（EXTI 捕获的上升沿计数）
 * @param  edge_ts 输出最近一次上升沿的时刻 (ms)，可为 NULL
 * @return 自上次调用以来的 INT 上升沿次数
 * @note   INT 高有效；APU_RST# 有效期间捕获的边沿不可信（APU 掉电浮空），
 *         由调用方结合 apu_io_apu_rst_asserted() 过滤
 */
uint8_t apu_io_int_consume(uint32_t *edge_ts);

/**
 * @brief  INT 引脚当前电平（高有效）
 * @note   多个 PD-Target 共享中断线时，电平为高也可能是其他目标的中断
 */
bool apu_io_int_asserted(void);
