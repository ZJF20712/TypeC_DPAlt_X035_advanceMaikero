#include "apu_i2c.h"

#include "ch32x035.h"
#include "ch32x035_gpio.h"
#include "ch32x035_i2c.h"
#include "ch32x035_rcc.h"
#include "debug.h"
#include "millis.h"

#include "apu_config.h"
#include "apu_io.h"
#include "usb_pd_log.h"

/* 连续超时达到后触发总线恢复 */
#define I2C_TIMEOUT_RECOVER_LIMIT 3

/* 扫描任务周期 (ms) */
#define I2C_SCAN_PERIOD_MS 5000
/* 全地址扫描的降频周期（每次约 30-100ms 阻塞，不能跟着校准探测每轮都做） */
#define I2C_FULL_SCAN_EVERY 12

static bool i2c_inited = false;
static uint8_t locked_remap = 0xFF; /* 校准成功的 I2C1_RM 编码（0xFF 未锁定） */
static bool target_online = false;  /* 目标地址已确认 ACK */
static uint8_t timeout_streak = 0;  /* 连续失败计数（触发总线恢复） */
static uint32_t last_scan_ms = 0;

#if APU_I2C_USE_SW

/* ============================================================
 * 软件 I2C：GPIO 开漏模拟（PC16=SCL / PC17=SDA）
 * PC16/17 位于 CFGXR，无硬件开漏输出模式，用
 * "输出低(CFG nibble=0x1, ODR=0) / 释放为浮空输入(nibble=0x4)" 模拟
 * ============================================================ */

/* ============================================================
 * 软件 I2C：GPIO 开漏模拟（PC16=SCL / PC17=SDA）
 * PC16/17 位于 CFGXR，无硬件开漏输出模式，用
 * "输出低(CFG nibble=0x1, ODR=0) / 释放为浮空输入(nibble=0x4)" 模拟
 * 支持运行时交换 SCL/SDA 引脚角色（排查接线交叉）
 * ============================================================ */

static bool sw_swapped = false; /* false: SCL=PC16/SDA=PC17（板级默认） */

/* CFGXR nibble: MODE=00+CNF=01(0x4)=浮空输入释放；MODE=01+CNF=00(0x1)=推挽输出低 */
static void sw_pin_set(uint8_t shift, bool drive) {
    uint32_t mask = 0xFu << shift;
    GPIOC->CFGXR = (GPIOC->CFGXR & ~mask) | ((drive ? 0x1u : 0x4u) << shift);
}

static void sw_scl(bool drive) {
    sw_pin_set(sw_swapped ? 4 : 0, drive);
}

static void sw_sda(bool drive) {
    sw_pin_set(sw_swapped ? 0 : 4, drive);
}

static bool sw_sda_level(void) {
    return GPIOC->INDR & (sw_swapped ? GPIO_Pin_16 : GPIO_Pin_17);
}

#define SW_DELAY() Delay_Us(APU_I2C_SW_HALF_US)

static bool apu_i2c_sw_init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* ODR 预置 0：驱动模式时输出低；释放模式靠外部上拉回高 */
    GPIOC->BCR = GPIO_Pin_16 | GPIO_Pin_17;
    sw_scl(false);
    sw_sda(false);

    i2c_inited = true;
    target_online = false;
    timeout_streak = 0;
    pd_logf("%ums APU: i2c SW init SCL=%s SDA=%s idle SCL=%d SDA=%d\r\n", millis(),
            sw_swapped ? "PC17" : "PC16", sw_swapped ? "PC16" : "PC17",
            (GPIOC->INDR & (sw_swapped ? GPIO_Pin_17 : GPIO_Pin_16)) ? 1 : 0,
            sw_sda_level() ? 1 : 0);
    return true;
}

/**
 * @brief  起始条件: SCL 高时 SDA 下降
 * @return false: SDA 被外部拉死，无法发起
 */
static bool sw_start(void) {
    sw_sda(false);
    sw_scl(false);
    SW_DELAY();
    if (!sw_sda_level()) {
        return false; /* 总线被占（SDA 拉死） */
    }
    sw_sda(true);
    SW_DELAY();
    sw_scl(true);
    SW_DELAY();
    return true;
}

static void sw_stop(void) {
    sw_sda(true);
    SW_DELAY();
    sw_scl(false);
    SW_DELAY();
    sw_sda(false); /* SCL 高期间 SDA 上升 = STOP */
    SW_DELAY();
}

/**
 * @brief  发送一个字节
 * @return true: 目标 ACK；false: NACK
 */
static bool sw_write_byte(uint8_t b) {
    for (uint8_t i = 8; i; i--, b <<= 1) {
        if (b & 0x80) {
            sw_sda(false);
        } else {
            sw_sda(true);
        }
        SW_DELAY();
        sw_scl(false);
        SW_DELAY();
        sw_scl(true);
    }
    /* 第 9 拍收 ACK */
    sw_sda(false);
    SW_DELAY();
    sw_scl(false);
    SW_DELAY();
    bool ack = !sw_sda_level();
    sw_scl(true);
    return ack;
}

/**
 * @brief  接收一个字节（ack=true 回 ACK，最后一字节传 false 回 NACK）
 */
static uint8_t sw_read_byte(bool ack) {
    uint8_t b = 0;

    sw_sda(false);
    for (uint8_t i = 8; i; i--) {
        SW_DELAY();
        sw_scl(false);
        SW_DELAY();
        b = (uint8_t)((b << 1) | (sw_sda_level() ? 1 : 0));
        sw_scl(true);
    }
    if (ack) {
        sw_sda(true);
    } else {
        sw_sda(false);
    }
    SW_DELAY();
    sw_scl(false);
    SW_DELAY();
    sw_scl(true);
    sw_sda(false);
    return b;
}

/**
 * @brief  总线恢复: SDA 被拉死时发 9 个 SCL 时钟 + STOP
 */
static void sw_recover(void) {
    sw_scl(false);
    for (uint8_t i = 0; i < 9 && !sw_sda_level(); i++) {
        sw_scl(true);
        SW_DELAY();
        sw_scl(false);
        SW_DELAY();
    }
    sw_stop();
}

apu_i2c_result_t apu_i2c_sw_probe(uint8_t addr7) {
    if (!i2c_inited) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_start()) {
        return APU_I2C_TIMEOUT; /* SDA 拉死 */
    }
    bool ack = sw_write_byte((uint8_t)(addr7 << 1));
    sw_stop();
    return ack ? APU_I2C_OK : APU_I2C_NACK;
}

apu_i2c_result_t apu_i2c_sw_write(uint8_t addr7, const uint8_t *data, uint8_t len) {
    if (!i2c_inited) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_start()) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_write_byte((uint8_t)(addr7 << 1))) {
        sw_stop();
        return APU_I2C_NACK;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (!sw_write_byte(data[i])) {
            sw_stop();
            return APU_I2C_NACK;
        }
    }
    sw_stop();
    return APU_I2C_OK;
}

apu_i2c_result_t apu_i2c_sw_read(uint8_t addr7, uint8_t *data, uint8_t len) {
    if (!i2c_inited || len == 0) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_start()) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_write_byte((uint8_t)((addr7 << 1) | 1))) {
        sw_stop();
        return APU_I2C_NACK;
    }
    for (uint8_t i = 0; i < len; i++) {
        data[i] = sw_read_byte(i != len - 1);
    }
    sw_stop();
    return APU_I2C_OK;
}

apu_i2c_result_t apu_i2c_sw_write_read(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                                       uint8_t *rdata, uint8_t rlen) {
    if (!i2c_inited || rlen == 0) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_start()) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_write_byte((uint8_t)(addr7 << 1))) {
        sw_stop();
        return APU_I2C_NACK;
    }
    for (uint8_t i = 0; i < wlen; i++) {
        if (!sw_write_byte(wdata[i])) {
            sw_stop();
            return APU_I2C_NACK;
        }
    }
    /* 重复起始 */
    sw_sda(false);
    SW_DELAY();
    if (!sw_start()) {
        return APU_I2C_TIMEOUT;
    }
    if (!sw_write_byte((uint8_t)((addr7 << 1) | 1))) {
        sw_stop();
        return APU_I2C_NACK;
    }
    for (uint8_t i = 0; i < rlen; i++) {
        rdata[i] = sw_read_byte(i != rlen - 1);
    }
    sw_stop();
    return APU_I2C_OK;
}

#else /* !APU_I2C_USE_SW: 硬件 I2C1 实现（保留备用） */

/**
 * @brief  等待库事件（BUSY/MSL/SB/ADDR/TXE... 组合）就绪
 */
static apu_i2c_result_t wait_event(uint32_t event) {
    uint32_t start = millis();

    while (I2C_CheckEvent(I2C1, event) != READY) {
        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            return APU_I2C_NACK;
        }
        if (millis() - start > APU_I2C_TIMEOUT_MS) {
#if APU_I2C_DEBUG
            pd_logf("%ums APU: evt %08lX t/o S1=%04X S2=%04X C1=%04X INDR=%08lX\r\n",
                    millis(), event, I2C1->STAR1, I2C1->STAR2, I2C1->CTLR1, GPIOC->INDR);
#endif
            return APU_I2C_TIMEOUT;
        }
    }
    return APU_I2C_OK;
}

/**
 * @brief  等待指定标志清零（用于 BUSY 释放）
 */
static apu_i2c_result_t wait_flag_clear(uint32_t flag) {
    uint32_t start = millis();

    while (I2C_GetFlagStatus(I2C1, flag) != RESET) {
        if (millis() - start > APU_I2C_TIMEOUT_MS) {
#if APU_I2C_DEBUG
            pd_logf("%ums APU: flag %08lX clr t/o S1=%04X S2=%04X INDR=%08lX\r\n",
                    millis(), flag, I2C1->STAR1, I2C1->STAR2, GPIOC->INDR);
#endif
            return APU_I2C_TIMEOUT;
        }
    }
    return APU_I2C_OK;
}

/**
 * @brief  收尾: 生成 STOP 并确保总线回到空闲
 */
static void finish_stop(void) {
    I2C_GenerateSTOP(I2C1, ENABLE);
    wait_flag_clear(I2C_FLAG_BUSY);
    /* 收尾恢复全局 ACK 使能（读流程可能关闭） */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/**
 * @brief  写阶段公共体: START + 地址(W) [+ 数据]，重复启动由 write_read 复用
 */
static apu_i2c_result_t do_write_phase(uint8_t addr7, const uint8_t *data, uint8_t len) {
    apu_i2c_result_t r;

    I2C_GenerateSTART(I2C1, ENABLE);
    r = wait_event(I2C_EVENT_MASTER_MODE_SELECT);
    if (r != APU_I2C_OK) {
        return r;
    }

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Transmitter);
    r = wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    if (r != APU_I2C_OK) {
        return r;
    }

    for (uint8_t i = 0; i < len; i++) {
        r = wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTING);
        if (r != APU_I2C_OK) {
            return r;
        }
        I2C_SendData(I2C1, data[i]);
    }
    if (len > 0) {
        r = wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        if (r != APU_I2C_OK) {
            return r;
        }
    }
    return APU_I2C_OK;
}

static apu_i2c_result_t hw_write_once(uint8_t addr7, const uint8_t *data, uint8_t len) {
    apu_i2c_result_t r;

    if (!i2c_inited) {
        return APU_I2C_TIMEOUT;
    }
    r = wait_flag_clear(I2C_FLAG_BUSY);
    if (r != APU_I2C_OK) {
        return r;
    }

    r = do_write_phase(addr7, data, len);
    finish_stop();
    return r;
}

/**
 * @brief  硬件事务: 超时(总线锁死)时先恢复再重试一次
 * @note   APU 掉电瞬间的总线毛刺会使外设误锁 BUSY(线双高仍 S2.bit1=1)，
 *         SWRST+9 时钟+重建外设可恢复（前提总线电气正常）
 */
apu_i2c_result_t apu_i2c_hw_write(uint8_t addr7, const uint8_t *data, uint8_t len) {
    apu_i2c_result_t r = hw_write_once(addr7, data, len);

    if (r == APU_I2C_TIMEOUT) {
        pd_logf("%ums APU: i2c write t/o -> recover & retry\r\n", millis());
        apu_i2c_bus_recover();
        r = hw_write_once(addr7, data, len);
    }
    return r;
}

static apu_i2c_result_t hw_read_once(uint8_t addr7, uint8_t *data, uint8_t len) {
    apu_i2c_result_t r;

    if (!i2c_inited || len == 0) {
        return APU_I2C_TIMEOUT;
    }
    r = wait_flag_clear(I2C_FLAG_BUSY);
    if (r != APU_I2C_OK) {
        return r;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    r = wait_event(I2C_EVENT_MASTER_MODE_SELECT);
    if (r != APU_I2C_OK) {
        finish_stop();
        return r;
    }

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Receiver);
    r = wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    if (r != APU_I2C_OK) {
        finish_stop();
        return r;
    }

    for (uint8_t i = 0; i < len; i++) {
        /* 末字节前关 ACK，向目标表明传输结束 */
        if (i == len - 1) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }
        uint32_t start = millis();
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET) {
            if (millis() - start > APU_I2C_TIMEOUT_MS) {
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                finish_stop();
                return APU_I2C_TIMEOUT;
            }
        }
        data[i] = I2C_ReceiveData(I2C1);
    }

    finish_stop();
    return APU_I2C_OK;
}

apu_i2c_result_t apu_i2c_hw_read(uint8_t addr7, uint8_t *data, uint8_t len) {
    apu_i2c_result_t r = hw_read_once(addr7, data, len);

    if (r == APU_I2C_TIMEOUT) {
        pd_logf("%ums APU: i2c read t/o -> recover & retry\r\n", millis());
        apu_i2c_bus_recover();
        r = hw_read_once(addr7, data, len);
    }
    return r;
}

static apu_i2c_result_t hw_write_read_once(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                                           uint8_t *rdata, uint8_t rlen) {
    apu_i2c_result_t r;

    if (!i2c_inited || rlen == 0) {
        return APU_I2C_TIMEOUT;
    }
    r = wait_flag_clear(I2C_FLAG_BUSY);
    if (r != APU_I2C_OK) {
        return r;
    }

    r = do_write_phase(addr7, wdata, wlen);
    if (r != APU_I2C_OK) {
        finish_stop();
        return r;
    }

    /* 重复启动进入读方向 */
    I2C_GenerateSTART(I2C1, ENABLE);
    r = wait_event(I2C_EVENT_MASTER_MODE_SELECT);
    if (r != APU_I2C_OK) {
        finish_stop();
        return r;
    }
    I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Receiver);
    r = wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    if (r != APU_I2C_OK) {
        finish_stop();
        return r;
    }

    for (uint8_t i = 0; i < rlen; i++) {
        if (i == rlen - 1) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }
        uint32_t start = millis();
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET) {
            if (millis() - start > APU_I2C_TIMEOUT_MS) {
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                finish_stop();
                return APU_I2C_TIMEOUT;
            }
        }
        rdata[i] = I2C_ReceiveData(I2C1);
    }

    finish_stop();
    return APU_I2C_OK;
}

apu_i2c_result_t apu_i2c_hw_write_read(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                                       uint8_t *rdata, uint8_t rlen) {
    apu_i2c_result_t r = hw_write_read_once(addr7, wdata, wlen, rdata, rlen);

    if (r == APU_I2C_TIMEOUT) {
        pd_logf("%ums APU: i2c wr+rd t/o -> recover & retry\r\n", millis());
        apu_i2c_bus_recover();
        r = hw_write_read_once(addr7, wdata, wlen, rdata, rlen);
    }
    return r;
}

/**
 * @brief  切换 I2C1_REMAP 编码（先关 PE）
 */
static void i2c_apply_remap(uint8_t remap) {
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;
    uint32_t pcfr = AFIO->PCFR1;
    pcfr &= ~AFIO_PCFR1_I2C1_REMAP;
    pcfr |= ((uint32_t)(remap & 0x7)) << 2;
    AFIO->PCFR1 = pcfr;
    I2C1->CTLR1 |= I2C_CTLR1_PE;
}

bool apu_i2c_hw_init(void) {
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef i2c = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* SCL=PC16 / SDA=PC17，复用推挽（开漏特性由 I2C 外设输出级管理，
     * 与 WCH 官方 EVT I2C 例程一致），外部上拉由板端提供 */
    gpio.GPIO_Pin = GPIO_Pin_16 | GPIO_Pin_17;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio);

    uint32_t pcfr = AFIO->PCFR1;
    pcfr &= ~AFIO_PCFR1_I2C1_REMAP;
    pcfr |= ((uint32_t)(APU_I2C_REMAP_PC16SCL & 0x7)) << 2;
    AFIO->PCFR1 = pcfr;

    I2C_DeInit(I2C1);
    i2c.I2C_ClockSpeed = APU_I2C_SPEED_HZ;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_16_9;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);

    i2c_inited = true;
    locked_remap = APU_I2C_REMAP_PC16SCL;
    /* target_online 保留：总线恢复/重建外设不改变目标可达性 */
    timeout_streak = 0;

    pd_logf("%ums APU: i2c1 init PC16=SCL PC17=SDA %ukHz (RM=0x%X)\r\n", millis(),
            APU_I2C_SPEED_HZ / 1000, APU_I2C_REMAP_PC16SCL);
    return true;
}

#endif /* APU_I2C_USE_SW */

/* ============================================================
 * 公共 API（按 APU_I2C_USE_SW 路由到软件/硬件实现）
 * ============================================================ */

bool apu_i2c_init(void) {
#if APU_I2C_USE_SW
    locked_remap = 0;
    return apu_i2c_sw_init();
#else
    locked_remap = APU_I2C_REMAP_PC16SCL;
    return apu_i2c_hw_init();
#endif
}

apu_i2c_result_t apu_i2c_probe(uint8_t addr7) {
#if APU_I2C_USE_SW
    return apu_i2c_sw_probe(addr7);
#else
    return apu_i2c_hw_write(addr7, NULL, 0);
#endif
}

apu_i2c_result_t apu_i2c_write(uint8_t addr7, const uint8_t *data, uint8_t len) {
#if APU_I2C_USE_SW
    return apu_i2c_sw_write(addr7, data, len);
#else
    return apu_i2c_hw_write(addr7, data, len);
#endif
}

apu_i2c_result_t apu_i2c_read(uint8_t addr7, uint8_t *data, uint8_t len) {
#if APU_I2C_USE_SW
    return apu_i2c_sw_read(addr7, data, len);
#else
    return apu_i2c_hw_read(addr7, data, len);
#endif
}

apu_i2c_result_t apu_i2c_write_read(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                                    uint8_t *rdata, uint8_t rlen) {
#if APU_I2C_USE_SW
    return apu_i2c_sw_write_read(addr7, wdata, wlen, rdata, rlen);
#else
    return apu_i2c_hw_write_read(addr7, wdata, wlen, rdata, rlen);
#endif
}

bool apu_i2c_remap_calibrate(uint8_t *found) {
#if APU_I2C_USE_SW
    /* 软件实现无 REMAP 概念，直接探测目标 */
    apu_i2c_result_t r = apu_i2c_probe(APU_I2C_TARGET_ADDR_7BIT);
    pd_logf("%ums APU: calibrate SCL=%s probe 0x%02X -> %s\r\n", millis(),
            sw_swapped ? "PC17" : "PC16", APU_I2C_TARGET_ADDR_7BIT,
            r == APU_I2C_OK ? "ACK" : (r == APU_I2C_NACK ? "nack" : "stuck"));
    if (r == APU_I2C_OK) {
        if (found) {
            *found = 0;
        }
        return true;
    }
    return false;
#else
    /* 校准顺序见 apu_config.h：优先本板接线编码，
     * 其余仅用于错接诊断（避开 SWD 使用的 PC18/PC19） */
    static const uint8_t remap_list[] = APU_I2C_REMAP_CAL_LIST;

    for (uint8_t i = 0; i < APU_I2C_REMAP_CAL_CNT; i++) {
        uint8_t rm = remap_list[i];

        if (rm == locked_remap) {
            /* 已锁定的编码无需重复切换，直接探测 */
        } else {
            i2c_apply_remap(rm);
        }

        apu_i2c_result_t r = apu_i2c_probe(APU_I2C_TARGET_ADDR_7BIT);
        pd_logf("%ums APU: calibrate RM=0x%X probe 0x%02X -> %s\r\n", millis(), rm,
                APU_I2C_TARGET_ADDR_7BIT,
                r == APU_I2C_OK ? "ACK" : (r == APU_I2C_NACK ? "nack" : "timeout"));

        if (r == APU_I2C_OK) {
            locked_remap = rm;
            if (found) {
                *found = rm;
            }
            return true;
        }
        if (rm == locked_remap) {
            /* 锁定编码反而失联: 恢复默认编码避免停在错编码上 */
            locked_remap = APU_I2C_REMAP_PC16SCL;
        }
    }

    /* 全部失败: 恢复默认编码 */
    i2c_apply_remap(APU_I2C_REMAP_PC16SCL);
    locked_remap = APU_I2C_REMAP_PC16SCL;
    return false;
#endif
}

uint8_t apu_i2c_bus_scan(void) {
    uint8_t found = 0;

    pd_logf("%ums APU: bus scan 0x08-0x77:\r\n", millis());
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (apu_i2c_probe(addr) == APU_I2C_OK) {
            pd_logf("  APU:   ACK 0x%02X\r\n", addr);
            found++;
        }
    }
    if (found == 0) {
        pd_logf("  APU:   none\r\n");
    }
    return found;
}

void apu_i2c_bus_recover(void) {
#if APU_I2C_USE_SW
    pd_logf("%ums APU: i2c bus recover (sw)\r\n", millis());
    sw_recover();
#else
    pd_logf("%ums APU: i2c bus recover\r\n", millis());

    /* 外设软复位 */
    I2C_Cmd(I2C1, DISABLE);
    I2C1->CTLR1 |= I2C_CTLR1_SWRST;
    I2C1->CTLR1 &= ~I2C_CTLR1_SWRST;

    /* SCL(PC16) 改 GPIO 输出，时钟 9 拍释放可能被目标拉死的 SDA(PC17) */
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = GPIO_Pin_16;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);
    GPIO_SetBits(GPIOC, GPIO_Pin_16);
    Delay_Us(5);

    for (uint8_t i = 0; i < 9; i++) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_17) == Bit_SET) {
            break; /* SDA 已释放 */
        }
        GPIO_ResetBits(GPIOC, GPIO_Pin_16);
        Delay_Us(5);
        GPIO_SetBits(GPIOC, GPIO_Pin_16);
        Delay_Us(5);
    }

    /* 模拟 STOP: SCL 高时令 SDA 产生低->高，释放目标（若 9 拍后仍被拉死） */
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_17) == Bit_RESET) {
        /* 换 SDA 为输出并拉低 */
        gpio.GPIO_Pin = GPIO_Pin_17;
        GPIO_Init(GPIOC, &gpio);
        GPIO_ResetBits(GPIOC, GPIO_Pin_17);
        Delay_Us(5);
        GPIO_SetBits(GPIOC, GPIO_Pin_16); /* SCL 保持高 */
        Delay_Us(5);
        GPIO_SetBits(GPIOC, GPIO_Pin_17); /* SCL 高期间 SDA 上升沿 = STOP */
        Delay_Us(5);
    }

    /* 恢复外设（重新走一遍完整初始化） */
    apu_i2c_init();
#endif
}

/**
 * @brief  挂起总线（APU_RST# 有效期间调用）：PE 关闭 + 引脚释放为浮空输入，
 *         隔离 APU 掉电瞬间经电平转换器耦合进来的总线毛刺，
 *         防止外设把毛刺误锁为 BUSY
 */
void apu_i2c_suspend(void) {
#if APU_I2C_USE_SW
    /* 软件 I2C 无外设状态可锁，引脚保持释放即可 */
    sw_scl(false);
    sw_sda(false);
#else
    I2C_Cmd(I2C1, DISABLE);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = GPIO_Pin_16 | GPIO_Pin_17;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);
#endif
    i2c_inited = false;
}

/**
 * @brief  恢复总线（APU_RST# 释放后调用），完整重新初始化
 */
void apu_i2c_resume(void) {
    apu_i2c_init();
}

bool apu_i2c_scan_task(void) {
    /* RST# 有效期间总线挂起（apu_amu_task 管理），不发任何事务 */
    if (apu_io_apu_rst_asserted()) {
        return target_online;
    }
    if (target_online) {
        return true;
    }
    if (!i2c_inited) {
        return false;
    }

    uint32_t now = millis();
    if ((uint32_t)(now - last_scan_ms) < I2C_SCAN_PERIOD_MS) {
        return false;
    }
    last_scan_ms = now;

    uint8_t rm = 0;
    if (apu_i2c_remap_calibrate(&rm)) {
        target_online = true;
        pd_logf("%ums APU: target 0x%02X online\r\n", millis(), APU_I2C_TARGET_ADDR_7BIT);
        apu_i2c_bus_scan(); /* 顺带报告总线上其他设备，辅助确认拓扑 */
        return true;
    }

    if (++timeout_streak >= I2C_TIMEOUT_RECOVER_LIMIT) {
        timeout_streak = 0;
        apu_i2c_bus_recover();
    }

    /* 校准失败轮次降频做全扫描：诊断目标是否换了地址/总线是否异常 */
    static uint8_t full_scan_div = 0;
    if (++full_scan_div >= I2C_FULL_SCAN_EVERY) {
        full_scan_div = 0;
        apu_i2c_bus_scan();
    }

#if APU_I2C_USE_SW
    /* 每 2 轮失败切换一次 SCL/SDA 引脚角色，排查接线交叉 */
    static uint8_t fail_rounds = 0;
    if (++fail_rounds % 2 == 0) {
        sw_swapped = !sw_swapped;
        pd_logf("%ums APU: i2c switch wiring -> SCL=%s\r\n", millis(),
                sw_swapped ? "PC17" : "PC16");
    }
#endif
    return false;
}

bool apu_i2c_target_ready(void) {
    return target_online;
}
