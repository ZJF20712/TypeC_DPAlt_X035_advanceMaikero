#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "usb_pd_message.h"

/**
 * @brief  初始化 PD SRC 策略机
 */
void usb_pd_policy_init(void);

/**
 * @brief  策略机周期处理（主循环调用）
 */
void usb_pd_policy_process(void);

/**
 * @brief  处理一条收到的 PD 消息（SOP）
 * @param  msg 消息指针（来自消息缓冲区）
 */
void usb_pd_policy_handle_msg(pd_msg_t *msg);

/**
 * @brief  CC 连接建立事件（CC 检测层回调）
 * @param  cc 1:CC1 2:CC2
 */
void usb_pd_policy_event_attach(uint8_t cc);

/**
 * @brief  CC 连接断开事件（CC 检测层回调）
 */
void usb_pd_policy_event_detach(void);

/**
 * @brief  是否已建立 PD 合同
 */
bool usb_pd_policy_contracted(void);

/**
 * @brief  DP Alt Mode 是否已协商完成
 */
bool usb_pd_policy_dp_active(void);
