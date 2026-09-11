#include "apu_io.h"

#include "ch32x035.h"
#include "ch32x035_exti.h"
#include "ch32x035_gpio.h"
#include "ch32x035_misc.h"
#include "debug.h"
#include "millis.h"

#include "apu_config.h"
#include "usb_pd_log.h"

/* IRQ_HPD 低脉冲宽度 (us)，DisplayPort 规范允许 50~500us */
#define HPD_IRQ_PULSE_US 500

/* HPD 请求（DP 模式事件）与实际输出电平 */
static bool hpd_request = false;
static bool hpd_output  = false;

/* INT 事件统计（EXTI ISR 写 / 主循环读清） */
static volatile uint8_t int_edge_cnt = 0;
static volatile uint32_t int_edge_ts = 0;

/**
 * @brief  直接驱动 HPD 输出电平
 */
static void hpd_write(bool high) {
    GPIO_WriteBit(APU_HPD_PORT, APU_HPD_PIN, high ? Bit_SET : Bit_RESET);
    hpd_output = high;
}

/**
 * @brief  按 "DP 请求 与 APU_RST#释放" 的与门刷新 HPD 实际输出
 */
static void hpd_update(void) {
    bool want = hpd_request && !apu_io_apu_rst_asserted();

    if (want != hpd_output) {
        hpd_write(want);
        pd_logf("%ums APU: HPD -> %s (req=%u rst#=%s)\r\n", millis(), want ? "HIGH" : "low",
                hpd_request ? 1 : 0, apu_io_apu_rst_asserted() ? "assert" : "ok");
    }
}

/**
 * @brief  EXTI Line11 (PB11 USBC_PD_INT) 中断服务
 * @note   只记录事件，日志/I2C 处理均放在主循环上下文
 */
void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_8_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line11);
        int_edge_ts = millis();
        int_edge_cnt++;
    }
}

void apu_io_init(void) {
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(APU_HPD_RCC | APU_RST_RCC | APU_INT_RCC | RCC_APB2Periph_AFIO, ENABLE);

    /* HPD: 推挽输出，复位默认低（100K 下拉兜底） */
    gpio.GPIO_Pin = APU_HPD_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(APU_HPD_PORT, &gpio);
    hpd_write(false);
    hpd_request = false;

    /* APU_RST#: 低有效，APU 侧推挽驱动，浮空输入 */
    gpio.GPIO_Pin = APU_RST_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(APU_RST_PORT, &gpio);

    /* USBC_PD_INT: 高有效，浮空输入 + 上升沿 EXTI */
    gpio.GPIO_Pin = APU_INT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(APU_INT_PORT, &gpio);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

    EXTI_InitTypeDef exti = {0};
    exti.EXTI_Line = EXTI_Line11;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic = {0};
    nvic.NVIC_IRQChannel = EXTI15_8_IRQn;
    /* 低优先级: PD(0) > EXTI(1) > UART4 日志(2)，避免日志类中断挤占 PD 时序 */
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    pd_logf("%ums APU: io init (HPD=PB1 low, RST#=PB3 in, INT=PB11 EXTI)\r\n", millis());
    pd_logf("%ums APU: initial RST#=%s INT=%d\r\n", millis(),
            apu_io_apu_rst_asserted() ? "assert" : "release",
            GPIO_ReadInputDataBit(APU_INT_PORT, APU_INT_PIN));
}

void apu_io_hpd_request(bool on) {
    if (hpd_request == on) {
        return;
    }
    hpd_request = on;
    hpd_update();
}

void apu_io_hpd_irq_pulse(void) {
    if (!hpd_output) {
        return;
    }
    hpd_write(false);
    Delay_Us(HPD_IRQ_PULSE_US);
    hpd_write(true);
}

bool apu_io_apu_rst_asserted(void) {
    return (GPIO_ReadInputDataBit(APU_RST_PORT, APU_RST_PIN) == Bit_RESET);
}

void apu_io_process(void) {
    /* 原始电平变化监测：G3/S5/复位期间 RST# 与 INT 的行为证据 */
    static bool rst_last = false;
    static bool int_last = false;
    static uint32_t last_log_ms = 0;
    static bool first = true;

    bool rst_now = apu_io_apu_rst_asserted();
    bool int_now = (GPIO_ReadInputDataBit(APU_INT_PORT, APU_INT_PIN) == Bit_SET);

    if (first || rst_now != rst_last || int_now != int_last) {
        uint32_t now = millis();

        if (first || (now - last_log_ms) >= 100) { /* 电平抖动限频 100ms */
            pd_logf("%ums APU: raw RST#=%s INT=%d\r\n", now,
                    rst_now ? "assert" : "release", int_now ? 1 : 0);
            last_log_ms = now;
        }
        rst_last = rst_now;
        int_last = int_now;
        first = false;
    }

    /* APU_RST# 释放/拉低变化时由与门逻辑自动收敛 HPD 电平 */
    hpd_update();
}

uint8_t apu_io_int_consume(uint32_t *edge_ts) {
    NVIC_DisableIRQ(EXTI15_8_IRQn);
    uint8_t cnt = int_edge_cnt;
    uint32_t ts = int_edge_ts;
    int_edge_cnt = 0;
    NVIC_EnableIRQ(EXTI15_8_IRQn);

    if (edge_ts) {
        *edge_ts = ts;
    }
    return cnt;
}

bool apu_io_int_asserted(void) {
    return GPIO_ReadInputDataBit(APU_INT_PORT, APU_INT_PIN) == Bit_SET;
}
