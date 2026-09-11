#include "ch32x035.h"
#include "debug.h"
#include "led_strip.h"
#include "millis.h"
#include "usb_pd_log.h"
#include "usb_pd_monitor.h"
#include "usb_pd_cc.h"
#include "usb_vbus_measure.h"

#include "apu_config.h"
#include "apu_amu.h"
#include "apu_io.h"
#include "apu_i2c.h"
#include "uart4_dbg.h"

#if APU_DBG_UART4_ENABLE
/* UART4 调试占用 PA5(SPI1 默认 SCK，QNF20 上 SPI1 无其他引脚组)，
 * 调试期间跳过灯带 SPI 初始化；后续灯带状态色调用 send_byte 内
 * is_spi_initialized 守卫自动旁路（LED 功能让位于串口日志） */
#define LED_STRIP_ENABLE 0
#else
#define LED_STRIP_ENABLE 1
#endif

#if LED_STRIP_ENABLE
static void led_strip_rainbow_effect(void) {
    uint8_t brightness = 0;
    const uint8_t target_brightness = 0x0A;
    const uint16_t steps = 360;
    const float brightness_step = (float)target_brightness / steps;

    for (uint16_t hue = 0; hue < steps; hue++) {
        brightness = (uint8_t)(hue * brightness_step);
        if (brightness > target_brightness) {
            brightness = target_brightness;
        }
        led_strip_set_pixel_hsv(0, hue, 220, brightness);
        led_strip_refresh();
        Delay_Ms(3);
    }
}
#endif

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    Delay_Init();
    // SDI_Printf_Enable();  /* UART4 不可用时的后备调试口（PC18/PC19 SWD） */

    millis_init();
    adc_init();

#if LED_STRIP_ENABLE
    // LED Strip
    led_strip_init();
    led_strip_rainbow_effect();
    /* 上电动画结束停在品红帧，显式设回空闲红（无设备基线色） */
    led_strip_set_pixel_with_refresh(0, 0x0A, 0x00, 0x00);
#endif

    /* UART4 调试口（PA5 -> COM26，RV-LinkE）尽早可用，接管 CDC */
    uart4_dbg_init();

    /* APU 侧 IO：HPD(PB1)=低 / APU_RST#(PB3) / INT(PB11+EXTI) */
    apu_io_init();

    /* USB PD Monitor（CC 检测 + PD PHY + SRC 策略机） */
    usb_pd_monitor_init();

    /* 日志（RAM 环 + SDI(关) + CDC(未初始化,静默) + UART4） */
    pd_log_init();
    pd_logf("\r\nusb-pd-dp-amd - USB-C PD Source (5V3A) + DP Alt Mode + AMD APU I2C\r\n");

    /* I2C1 主机（SCL=PC16 / SDA=PC17），目标 0x5C 上线前由扫描任务周期探测 */
    apu_i2c_init();

    /* AMD 协议层（目标在线后自动做一次只读模式判定） */
    apu_amu_init();

    // cc_en
    usb_pd_cc_en(true);

    // SRC 角色：关闭 Rd 下拉（Rp 由 USBPD 外设内部电流源提供）
    usb_pd_cc_rd_en(false);

    while (1) {
        static uint32_t last_process_millis = 0;
        if (millis() - last_process_millis >= 10) {
            last_process_millis = millis();
            usb_pd_monitor_process();
        }

        /* APU 任务先于策略协调: RST# 挂起/恢复与复位后首次状态同步
         * （post-RST CB state 打印）不被 reconciler 的首笔写抢跑 */
        apu_amu_task();

        /* AMD I2C 目标探测：APU 进 S0 后扫描任务自动确认在线 */
        apu_i2c_scan_task();

        /* HPD 的 APU_RST# 与门维护 */
        apu_io_process();

#if APU_DBG_UART4_REMAP_SCAN
        /* 首次上电校准 PA5：在 COM26 观察哪个 USART4_RM 出字 */
        uart4_dbg_remap_scan_task();
#endif
    }
}
