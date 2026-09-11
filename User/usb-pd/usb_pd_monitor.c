#include "usb_pd_monitor.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "millis.h"
#include "usb_pd_cc.h"
#include "usb_pd_log.h"
#include "usb_pd_message.h"
#include "usb_pd_phy.h"
#include "usb_pd_policy.h"

/* CC 连接状态 */
static cc_state_t cc_state = {0};

/**
 * @brief  初始化 USB PD（CC 检测 + PHY 收发 + SRC 策略机）
 */
void usb_pd_monitor_init(void) {
    // 使能时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    // CC 检测（SRC 角色 Rp 上拉）
    usb_pd_cc_init();

    // PD 物理层（中断接收 + GoodCRC 自动应答）
    usb_pd_phy_init();

    // SRC 策略机
    usb_pd_policy_init();
}

/**
 * @brief  USB PD 周期处理（主循环 10ms 调用）
 */
void usb_pd_monitor_process(void) {
    static uint32_t diag_ms = 0;
    static uint16_t diag_bit = 0, diag_byte = 0;

    // RX 位级活动诊断：统计帧前导/位活动标志（整帧解不出也会置位）
    uint8_t st = USBPD->STATUS;
    if (st & (IF_RX_BIT | IF_RX_BYTE)) {
        USBPD->STATUS |= (IF_RX_BIT | IF_RX_BYTE);
        if (st & IF_RX_BIT) diag_bit++;
        if (st & IF_RX_BYTE) diag_byte++;
    }

    // CC 连接状态检测（attach/detach 驱动策略机）
    usb_pd_cc_check_connection(&cc_state);

    // 策略机周期处理（定时发送：SourceCap / PS_RDY / VDM 流程）
    usb_pd_policy_process();

    // 处理并打印消息缓冲区中的新消息
    pd_msg_buffer_t *msg_buffer = get_message_buffer();
    while (msg_buffer->read_idx != msg_buffer->write_idx) {
        pd_msg_t *msg = &msg_buffer->msgs[msg_buffer->read_idx];

        // 先交给策略机（保证响应时延），再打印
        usb_pd_policy_handle_msg(msg);
        print_message(msg);

        msg_buffer->read_idx = (msg_buffer->read_idx + 1) % PD_MSG_BUFFER_SIZE;
    }

    // 每秒输出一次 RX 位级活动统计
    if (millis() - diag_ms >= 1000) {
        diag_ms = millis();
        pd_logf("%ums RX diag: bit=%u byte=%u\r\n", millis(), diag_bit, diag_byte);
        diag_bit = 0;
        diag_byte = 0;
    }
}
