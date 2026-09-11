#include "usb_pd_cc.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "led_strip.h"
#include "millis.h"
#include "usb_pd_log.h"
#include "usb_pd_message.h"
#include "usb_pd_policy.h"
#include "usb_vbus_measure.h"

#define USE_CC_CTRL
#define USE_CC_RD_CTRL

/* 连接检测去抖次数（10ms 检测周期） */
#define CC_ATTACH_DEBOUNCE 3
#define CC_DETACH_DEBOUNCE 5

/* CC 状态周期监测打印间隔 */
#define CC_REPORT_PERIOD_MS 1000

/* CC 线状态 */
typedef enum {
    CC_LINE_LOW = 0, // <0.22V
    CC_LINE_RA,      // Ra 0.22~0.66V（音频附件/线缆）
    CC_LINE_RD,      // Rd 0.66~2.2V（sink 接入）
    CC_LINE_OPEN,    // >2.2V（Rp 拉高，开路）
} cc_line_state_t;

void usb_pd_cc_en(bool en) {
#ifdef USE_CC_CTRL
    static uint8_t inited = 0;

    if (!inited) {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        inited = 1;
    }

    GPIO_WriteBit(GPIOB, GPIO_Pin_0, en ? Bit_SET : Bit_RESET);
#endif
}

void usb_pd_cc_rd_en(bool en) {
#ifdef USE_CC_RD_CTRL
    static uint8_t inited = 0;

    if (!inited) {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        inited = 1;
    }

    GPIO_WriteBit(GPIOB, GPIO_Pin_12, en ? Bit_SET : Bit_RESET);
#endif
}

/**
 * @brief  设置 CC 线 Rp 上拉（SRC 角色呈现）
 * @param  cc_num 1:CC1, 2:CC2
 * @param  en true:Rp 上拉，false:无上拉
 * @note   Rp-3.0A 档，与 5V/3A PDO 一致
 */
static void cc_rp_set(uint8_t cc_num, bool en) {
    volatile uint16_t *port_cc = (cc_num == 1) ? &USBPD->PORT_CC1 : &USBPD->PORT_CC2;

    if (en) {
        *port_cc = (*port_cc & ~CC_PU_Mask) | CC_PU_330;
    } else {
        *port_cc = (*port_cc & ~CC_PU_Mask) | CC_NO_PU;
    }
}

/**
 * @brief  读取 CC 线状态
 * @param  cc_num 1:CC1, 2:CC2
 * @return cc_line_state_t
 */
static cc_line_state_t cc_line_state(uint8_t cc_num) {
    register uint32_t pin;
    volatile uint16_t *port_cc;

    if (cc_num == 1) {
        pin = PIN_CC1;
        port_cc = &USBPD->PORT_CC1;
    } else {
        pin = PIN_CC2;
        port_cc = &USBPD->PORT_CC2;
    }

    /* 未接负载时 Rp 将 CC 拉至 VDD：GPIO 高阈值 (2.2V) 判定开路 */
    if (GPIOC->INDR & pin) {
        return CC_LINE_OPEN;
    }

    /* >0.66V 且 <2.2V：Rd 接入（330uA × Rd ≈ 1.5~1.7V） */
    *port_cc = (*port_cc & ~(CC_CMP_Mask | PA_CC_AI)) | CC_CMP_66;
    Delay_Us(2);
    if (*port_cc & PA_CC_AI) {
        return CC_LINE_RD;
    }

    /* 0.22V~0.66V：Ra（音频附件 / 线缆 Ra） */
    *port_cc = (*port_cc & ~(CC_CMP_Mask | PA_CC_AI)) | CC_CMP_22;
    Delay_Us(2);
    if (*port_cc & PA_CC_AI) {
        return CC_LINE_RA;
    }

    return CC_LINE_LOW;
}

/**
 * @brief  比较器逐档扫描 CC 电压（未连接时用于监测打印）
 * @param  cc_num 1:CC1, 2:CC2
 * @return 电压档位值 mV：0 / 22 / 45 / 55 / 66 / 95 / 123 / 330(>2.2V 开路)
 */
static uint16_t cc_scan_voltage(uint8_t cc_num) {
    static const uint16_t cmp_list[] = {CC_CMP_22, CC_CMP_45, CC_CMP_55, CC_CMP_66, CC_CMP_95, CC_CMP_123};
    static const uint16_t vol_list[] = {22, 45, 55, 66, 95, 123};
    register uint32_t pin;
    volatile uint16_t *port_cc;
    uint16_t cc_vol = 0;
    uint8_t i;

    if (cc_num == 1) {
        pin = PIN_CC1;
        port_cc = &USBPD->PORT_CC1;
    } else {
        pin = PIN_CC2;
        port_cc = &USBPD->PORT_CC2;
    }

    /* Rp 上拉且无负载：被拉至 VDD，GPIO 高阈值 (2.2V) 判定开路 */
    if (GPIOC->INDR & pin) {
        return 330;
    }

    for (i = 0; i < sizeof(cmp_list) / sizeof(cmp_list[0]); i++) {
        *port_cc = (*port_cc & ~(CC_CMP_Mask | PA_CC_AI)) | cmp_list[i];
        Delay_Us(2);
        if (*port_cc & PA_CC_AI) {
            cc_vol = vol_list[i];
        } else {
            break;
        }
    }

    /* 恢复比较器为 0.66V 档 */
    *port_cc = (*port_cc & ~(CC_CMP_Mask | PA_CC_AI)) | CC_CMP_66;
    return cc_vol;
}

/**
 * @brief  初始化 CC 引脚（SRC 角色：双线 Rp 上拉）
 */
void usb_pd_cc_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;

    /* SRC：双线呈现 Rp-3.0A，等待 sink 接入 */
    cc_rp_set(1, true);
    cc_rp_set(2, true);
}

/**
 * @brief  检测 CC 连接状态（SRC 角色）
 * @param  cc_state: CC 状态
 */
void usb_pd_cc_check_connection(cc_state_t *cc_state) {
    static uint32_t report_ms = 0;

    /* 周期监测打印：CC 比较器电压 + VBUS */
    if (millis() - report_ms >= CC_REPORT_PERIOD_MS) {
        report_ms = millis();
        if (cc_state->cc_connection) {
            pd_logf("%ums CC: attached CC%u, VBUS:%05umV, VDD:%dmV\r\n",
                    millis(), cc_state->cc_connection, adc_get_vbus_mv(), adc_get_vdd_mv());
        } else {
            pd_logf("%ums CC: idle CC1:%03umV CC2:%03umV VBUS:%05umV\r\n",
                    millis(), cc_scan_voltage(1), cc_scan_voltage(2), adc_get_vbus_mv());
        }
    }

    if (cc_state->cc_connection) {
        /* 已连接：检测 sink 拔出（活动线被 Rp 重新拉高 >2.2V） */
        register uint32_t pin = (cc_state->cc_connection == 1) ? PIN_CC1 : PIN_CC2;

        if (GPIOC->INDR & pin) {
            if (++cc_state->cc_connection_count >= CC_DETACH_DEBOUNCE) {
                pd_logf("%ums Detach:CC%u, VBUS:%05umV\r\n",
                        millis(), cc_state->cc_connection, adc_get_vbus_mv());
                usb_pd_cc_detach(cc_state);
            }
        } else {
            cc_state->cc_connection_count = 0;
        }
        return;
    }

    /* 未连接：检测 Rd 接入（规范判定：一线 vRd 窗口、另一线非 vRd，
     * 允许另一线为开路 / Ra / 弱下拉等各种非 Rd 状态） */
    cc_line_state_t s1 = cc_line_state(1);
    cc_line_state_t s2 = cc_line_state(2);

    /* 恢复比较器为 0.66V 档 */
    USBPD->PORT_CC1 = (USBPD->PORT_CC1 & ~(CC_CMP_Mask | PA_CC_AI)) | CC_CMP_66;
    USBPD->PORT_CC2 = (USBPD->PORT_CC2 & ~(CC_CMP_Mask | PA_CC_AI)) | CC_CMP_66;

    bool cond = (s1 == CC_LINE_RD && s2 != CC_LINE_RD) ||
                (s2 == CC_LINE_RD && s1 != CC_LINE_RD);

    /* E-mark 线缆检测：单线 Ra（无 Rd）说明线缆带电子标记芯片。
     * E-mark 线缆的 Ra 与设备 Rd 恒在不同线上——Ra 出现后，
     * 设备 Rd 会出现在另一条线（本机无 VCONN，线缆芯片须自 VBUS 取电） */
    static uint8_t emark_line = 0;
    if (s1 == CC_LINE_RA && s2 != CC_LINE_RA && emark_line != 1) {
        emark_line = 1;
        pd_logf("%ums CC: EMark cable Ra on CC1, expect device Rd on CC2\r\n", millis());
    } else if (s2 == CC_LINE_RA && s1 != CC_LINE_RA && emark_line != 2) {
        emark_line = 2;
        pd_logf("%ums CC: EMark cable Ra on CC2, expect device Rd on CC1\r\n", millis());
    } else if (s1 != CC_LINE_RA && s2 != CC_LINE_RA) {
        emark_line = 0;
    }

    if (!cond) {
        cc_state->cc_connection_count = 0;
        return;
    }

    if (++cc_state->cc_connection_count < CC_ATTACH_DEBOUNCE) {
        return;
    }
    cc_state->cc_connection_count = 0;

    /* 确认连接：选择活动 CC，关闭另一线 Rp */
    if (s1 == CC_LINE_RD) {
        cc_state->cc_connection = 1;
    } else {
        cc_state->cc_connection = 2;
    }

    if (cc_state->cc_connection == 1) {
        USBPD->CONFIG &= ~CC_SEL;
        cc_rp_set(2, false);
        led_strip_set_pixel_with_refresh(0, 0x00, 0x00, 0x0A); // RGB BLUE
        pd_logf("%ums Attach:CC1, VBUS:%05umV\r\n", millis(), adc_get_vbus_mv());
    } else {
        USBPD->CONFIG |= CC_SEL;
        cc_rp_set(1, false);
        led_strip_set_pixel_with_refresh(0, 0x00, 0x0A, 0x00); // RGB GREEN
        pd_logf("%ums Attach:CC2, VBUS:%05umV\r\n", millis(), adc_get_vbus_mv());
    }

    usb_pd_policy_event_attach(cc_state->cc_connection);
}

void usb_pd_cc_detach(cc_state_t *cc_state) {
    cc_state->cc_connection = 0;
    cc_state->cc_connection_count = 0;
    reset_message_counter();
    /* 恢复双线 Rp，等待重新接入 */
    cc_rp_set(1, true);
    cc_rp_set(2, true);
    usb_pd_policy_event_detach();
    led_strip_set_pixel_with_refresh(0, 0x0A, 0x00, 0x00); // RGB RED
}
