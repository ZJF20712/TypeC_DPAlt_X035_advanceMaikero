#pragma once

#include <stdbool.h>
#include <stdint.h>

/* I2C 事务结果 */
typedef enum {
    APU_I2C_OK = 0,      /* ACK 正常完成 */
    APU_I2C_NACK,        /* 地址或数据被 NACK（目标未就绪/地址错/未上电） */
    APU_I2C_TIMEOUT,     /* 等待标志超时（总线卡死或目标拉死） */
} apu_i2c_result_t;

/**
 * @brief  初始化 I2C1 主机（SCL=PC16 / SDA=PC17，开漏由 I2C 外设管理）
 * @return true: 外设就绪
 */
bool apu_i2c_init(void);

/**
 * @brief  探测 7 位地址是否 ACK（等价 0 字节写）
 */
apu_i2c_result_t apu_i2c_probe(uint8_t addr7);

/**
 * @brief  写事务: START + addr+W + 数据 + STOP
 */
apu_i2c_result_t apu_i2c_write(uint8_t addr7, const uint8_t *data, uint8_t len);

/**
 * @brief  读事务: START + addr+R + 读 len 字节 + STOP（末字节 NACK）
 */
apu_i2c_result_t apu_i2c_read(uint8_t addr7, uint8_t *data, uint8_t len);

/**
 * @brief  写后重复启动读: START+写数据 -> Sr+读数据 -> STOP
 * @note   AMD Interrupt 模式的命令码读（0x80/0xA0/0xA2）依赖此格式
 */
apu_i2c_result_t apu_i2c_write_read(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                                    uint8_t *rdata, uint8_t rlen);

/**
 * @brief  REMAP 校准 + 目标地址探测（设备接入后由扫描任务周期调用）
 * @param  found 输出: 校准成功时锁定的 I2C1_RM 编码
 * @return true: APU_I2C_TARGET_ADDR_7BIT 得到 ACK（REMAP 已锁定）
 * @note   逐一尝试配置中列出的安全编码并探测目标地址；失败时调用方可
 *         继续用 apu_i2c_bus_scan 做全地址扫描辅助定位
 */
bool apu_i2c_remap_calibrate(uint8_t *found);

/**
 * @brief  全地址扫描（0x08~0x77），结果经日志输出
 * @return ACK 到的设备数量
 */
uint8_t apu_i2c_bus_scan(void);

/**
 * @brief  周期扫描任务（主循环调用）：APU 目标出现前每 5s 校准+探测一次
 * @return true: 目标地址已确认在线
 */
bool apu_i2c_scan_task(void);

/**
 * @brief  目标是否已确认在线（扫描任务成功后置位）
 */
bool apu_i2c_target_ready(void);

/**
 * @brief  总线恢复：I2C 外设软复位 + SCL 发 9 个时钟释放被拉死的 SDA
 * @note   连续超时后调用；CH32 侧复位不会改变 APU Target 状态
 */
void apu_i2c_bus_recover(void);

/**
 * @brief  挂起总线（APU_RST# 有效期间调用）：外设停用 + 引脚释放，
 *         隔离 APU 掉电瞬间的总线毛刺（防外设误锁 BUSY）
 */
void apu_i2c_suspend(void);

/**
 * @brief  恢复总线（APU_RST# 释放后调用），完整重新初始化
 */
void apu_i2c_resume(void);
