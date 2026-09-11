#include "uart4_dbg.h"

#include <stdio.h>
#include <string.h>

#include "ch32x035.h"
#include "ch32x035_gpio.h"
#include "ch32x035_misc.h"
#include "ch32x035_rcc.h"
#include "ch32x035_usart.h"
#include "debug.h"
#include "millis.h"

#include "apu_config.h"

#if APU_DBG_UART4_ENABLE

/* 发送环形缓冲（2 的幂）。115200 下约 11.5KB/s，512B 可吸收突发日志 */
#define UART4_TX_BUF_SIZE 512
#define UART4_TX_BUF_MASK (UART4_TX_BUF_SIZE - 1)

static volatile uint8_t tx_buf[UART4_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0; /* ISR 写入位置 */
static volatile uint16_t tx_tail = 0; /* 下一次发送位置 */
static volatile uint32_t tx_dropped = 0;
static uint8_t uart4_inited = 0;

void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART4_IRQHandler(void) {
    if (USART_GetITStatus(USART4, USART_IT_TXE) != RESET) {
        if (tx_tail != tx_head) {
            USART_SendData(USART4, tx_buf[tx_tail]);
            tx_tail = (tx_tail + 1) & UART4_TX_BUF_MASK;
        } else {
            /* 缓冲空: 关 TXE 中断休眠，下次写入时重新唤醒 */
            USART_ITConfig(USART4, USART_IT_TXE, DISABLE);
        }
    }
}

void uart4_dbg_init(void) {
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4, ENABLE);

    /* PA5 复用推挽 = UART4 TX（USART4_RM=001，默认 SCK 位让给串口） */
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    uint32_t pcfr = AFIO->PCFR1;
    pcfr &= ~AFIO_PCFR1_USART4_REMAP;
    pcfr |= ((uint32_t)(APU_DBG_UART4_REMAP & 0x7)) << 12;
    AFIO->PCFR1 = pcfr;

    usart.USART_BaudRate = APU_DBG_UART4_BAUDRATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART4, &usart);

    /* 低优先级: PD(0) > EXTI(1) > UART4(2)，日志不得挤占 PD 时序 */
    nvic.NVIC_IRQChannel = USART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(USART4, USART_IT_TXE, DISABLE);
    USART_Cmd(USART4, ENABLE);
    uart4_inited = 1;
}

void uart4_dbg_write(const char *data, uint16_t len) {
    if (!uart4_inited || data == NULL || len == 0) {
        return;
    }

    NVIC_DisableIRQ(USART4_IRQn);
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (tx_head + 1) & UART4_TX_BUF_MASK;
        if (next == tx_tail) {
            tx_dropped++;
            break; /* 缓冲满: 丢弃余下内容（新入者淘汰，保住已排队日志连续性） */
        }
        tx_buf[tx_head] = (uint8_t)data[i];
        tx_head = next;
    }
    NVIC_EnableIRQ(USART4_IRQn);

    /* 缓冲有数据则唤醒发送（TXE 中断关着时） */
    if (USART_GetITStatus(USART4, USART_IT_TXE) == RESET) {
        USART_ITConfig(USART4, USART_IT_TXE, ENABLE);
    }
}

uint32_t uart4_dbg_dropped(void) {
    return tx_dropped;
}

#if APU_DBG_UART4_REMAP_SCAN
void uart4_dbg_remap_scan_task(void) {
    /* 安全编码集合: 排除 TX 落在 PC16/PC17(I2C) 与经 PC18/PC19 的组合
     * 000->PB0  001->PA5  011->PB9  100->PB13  110->PB13 */
    static const uint8_t rm_list[] = {0x0, 0x1, 0x3, 0x4, 0x6};
    static uint8_t idx = 0;
    static uint32_t last_ms = 0;
    char banner[48];

    uint32_t now = millis();
    if (now - last_ms < 2000) {
        return;
    }
    last_ms = now;

    uint8_t rm = rm_list[idx];
    idx = (idx + 1) % (sizeof(rm_list) / sizeof(rm_list[0]));

    USART_Cmd(USART4, DISABLE);
    uint32_t pcfr = AFIO->PCFR1;
    pcfr &= ~AFIO_PCFR1_USART4_REMAP;
    pcfr |= ((uint32_t)rm) << 12;
    AFIO->PCFR1 = pcfr;
    USART_Cmd(USART4, ENABLE);

    int n = snprintf(banner, sizeof(banner),
                     "\r\nUART4 remap scan: USART4_RM=0x%X (001=PA5)\r\n", rm);
    uart4_dbg_write(banner, (uint16_t)n);
}
#endif

#endif /* APU_DBG_UART4_ENABLE */
