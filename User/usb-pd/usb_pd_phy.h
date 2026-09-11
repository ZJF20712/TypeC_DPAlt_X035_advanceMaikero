#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "ch32x035_usbpd.h"

/* 发送结果 */
#define PD_PHY_TX_OK   0
#define PD_PHY_TX_FAIL 1

/**
 * @brief  初始化 USB PD 物理层（含中断、接收模式）
 */
void usb_pd_phy_init(void);

/**
 * @brief  发送 PD 消息（自动构建 Header / MessageID / 重试 / 等待 GoodCRC）
 * @note   仅可在主循环上下文调用，阻塞约 1~3ms
 * @param  msg_type 消息类型（DEF_TYPE_*）
 * @param  vdos 数据对象数组（可 NULL）
 * @param  num_vdo 数据对象数量（0~7）
 * @return PD_PHY_TX_OK / PD_PHY_TX_FAIL
 */
uint8_t usb_pd_phy_send_msg(uint8_t msg_type, const uint32_t *vdos, uint8_t num_vdo);

/**
 * @brief  发送硬复位（Hard Reset）
 */
void usb_pd_phy_send_hard_reset(void);

/**
 * @brief  向 SOP'（线缆插头）发送 Discover Identity 请求
 * @note   模拟主机完整序列（对齐笔记本行为）；无线缆芯片时不等待应答
 */
void usb_pd_phy_send_sop1_disc_ident(void);

/**
 * @brief  设置后续发送使用的 PD 规范版本（跟随对端）
 * @param  rev DEF_PD_REVISION_20 / DEF_PD_REVISION_30
 */
void usb_pd_phy_set_spec_rev(uint8_t rev);

/**
 * @brief  获取当前 PD 规范版本
 */
uint8_t usb_pd_phy_get_spec_rev(void);

/**
 * @brief  重置 MessageID 计数器（Soft Reset / 连接建立时调用）
 */
void usb_pd_phy_reset_msgid(void);
