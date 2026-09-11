#pragma once

#include <stdbool.h>
#include <stdint.h>

/* AMD USB PD I2C Target 通信通道模式（文档 Chapter 4: 4.1 Polling / 4.2 Interrupt，
 * 由平台 BIOS 预配置，无只读的模式寄存器，需按事务合法性反推） */
typedef enum {
    APU_AMU_MODE_UNKNOWN = 0,
    APU_AMU_MODE_POLLING,     /* 直读 3 字节状态，命令码读非法 */
    APU_AMU_MODE_INTERRUPT,   /* INT 引脚事件驱动，0x80/0xA0/0xA2 命令码读 */
} apu_amu_mode_t;

/* Crossbar 连接模式（写控制字节 bit3:0，文档 Table 3/6/7） */
typedef enum {
    APU_CB_SAFE = 0,        /* 00b: Safe State（LP 必须置 1） */
    APU_CB_USB3 = 1,        /* 01b: USB 3.x Connected */
    APU_CB_DP_4LANE = 2,    /* 10b: DP Alternate Mode - 4 lanes */
    APU_CB_USB3_DP_L01 = 3, /* 11b: USB 3.x + DP Lanes 0&1 */
} apu_cb_mode_t;

/* Crossbar 方向（写控制字节 bit4） */
#define APU_ORIENT_NORMAL 0
#define APU_ORIENT_FLIPPED 1

/**
 * @brief  初始化 AMD 协议层
 */
void apu_amu_init(void);

/**
 * @brief  主循环任务：目标在线后自动做一次只读模式判定，并驱动 Crossbar
 *         状态机（Polling/Interrupt 由 APU_AMU_POLLING_MODE 选择）
 */
void apu_amu_task(void);

/**
 * @brief  通信通道模式判定结果
 */
apu_amu_mode_t apu_amu_mode(void);

/**
 * @brief  请求切换 Crossbar 模式（内部自动 Safe State 中转 + 完成确认）
 * @param  cc_active 当前活动 CC 线（1=CC1, 2=CC2），仅用于日志
 * @param  mode 目标模式
 * @param  orientation APU_ORIENT_NORMAL / APU_ORIENT_FLIPPED
 * @note   非阻塞：请求入队后由 apu_amu_task 驱动；重复请求以最新为准。
 *         切换到非 DP 模式完成后会自动拉低 HPD 请求（APM 4.2.3.6.1 规则）
 */
void apu_amu_request_mode(uint8_t cc_active, apu_cb_mode_t mode, uint8_t orientation);

/**
 * @brief  Crossbar Ready 镜像（状态读/命令码读维护）
 */
bool apu_amu_cb_ready(void);

/**
 * @brief  最近一次成功完成的 Crossbar 模式与方向（镜像）
 */
apu_cb_mode_t apu_amu_cb_mode(void);
uint8_t apu_amu_cb_orientation(void);

/**
 * @brief  是否有切换请求进行中（未完成）
 */
bool apu_amu_cb_busy(void);

/**
 * @brief  Crossbar 模式镜像是否有效（复位/挂起后为 false，由同步阶段重建）
 */
bool apu_amu_mirror_valid(void);

/**
 * @brief  Crossbar 是否处于 DP 通路（DP 4-lane 或 USB3+DP）且最近切换已完成
 * @note   HPD 拉高的前提条件之一（文档 4.1.3.6.1）
 */
bool apu_amu_dp_path_active(void);
