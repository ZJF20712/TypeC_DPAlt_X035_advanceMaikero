#pragma once

/* ============================================================
 * AMD APU 互联板级配置
 * 依据:《AMD USB Power Delivery I2C Target Programming Guide
 *       for APU Gen2.3》(Pub #57239 Rev 0.90) 与本板原理图
 * ============================================================ */

/* ---------------- I2C 总线 ---------------- */
/* AMD USB PD I2C Target 的 7 位地址（Chapter 3, Table 2）:
 * 0x5C + Port Index 0 + Non-USB4 端口 */
#define APU_I2C_TARGET_ADDR_7BIT   0x5C

/* I2C 速率: AMD Target 支持 Standard(100k)/Fast(400k)，取保守 100k */
#define APU_I2C_SPEED_HZ           100000

/* I2C 单笔事务等待标志的超时 (ms) */
#define APU_I2C_TIMEOUT_MS         10

/* -------- CH32X035 AFIO_PCFR1 I2C1_RM[2:0] 编码 (RM GPIO 章) --------
 * 000: SCL=PA10 SDA=PA11      001: SCL=PA13 SDA=PA14
 * 010: SCL=PC16 SDA=PC17  <-- 本板接线（与 USB DM/DP 复用）
 * 011: SCL=PC19 SDA=PC18      1x0: SCL=PC17 SDA=PC16
 * 1x1: SCL=PC18 SDA=PC19
 * 注: 011/101/111 会把 AF 引脚压到 PC18/PC19(SWDIO/SWCLK)，禁用 */
#define APU_I2C_REMAP_PC16SCL      0x2 /* 010b: 与本板接线一致 */

/* REMAP 校准扫描顺序（设备接入后由扫描任务逐一验证）:
 * 仅尝试落在 PC16/PC17 上的编码 + 默认编码，避开 SWD 引脚 */
#define APU_I2C_REMAP_CAL_LIST     { APU_I2C_REMAP_PC16SCL, 0x4, 0x0, 0x1 }
#define APU_I2C_REMAP_CAL_CNT      4

/* ---------------- CC 活动线 -> Orientation 映射 ----------------
 * AMD 请求: 0=Normal(TX1=A2/A3), 1=Flipped(TX1=B2/B3)
 * 兼容设计: 板上 CC1/CC2 与连接器 A/B 面的对应关系由硬件布线决定，
 * 若方向反了只需翻转 APU_ORIENTATION_CC1_IS_NORMAL。
 * 实测记录: CC1->Normal 与 CC1->Flipped 两种方向写入后 U 盘均未枚举，
 * 方向非枚举失败主因，恢复常规映射 CC1->Normal */
#define APU_ORIENTATION_CC1_IS_NORMAL 1

#define APU_ORIENT_FROM_CC(cc_active) \
    ((uint8_t)(((cc_active) == 1) ? (APU_ORIENTATION_CC1_IS_NORMAL ? 0 : 1) \
                                  : (APU_ORIENTATION_CC1_IS_NORMAL ? 1 : 0)))

/* ---------------- APU 侧 GPIO ---------------- */
/* HPD: PB1 输出 -> APU DP HPD。
 * 板上 100K 下拉保证复位/上电期 HPD=低（安全电平）。
 * AMD 4.2.3.6.1 方案 2: HPD = DP_HPD事件 与 APU_RST# 的固件与门，
 * 因此推挽输出即可，不改开漏。 */
#define APU_HPD_PORT               GPIOB
#define APU_HPD_PIN                GPIO_Pin_1
#define APU_HPD_RCC                RCC_APB2Periph_GPIOB

/* APU_RST#: PB3 输入, 低有效(复位/S3~S5 有效)。APU 侧推挽驱动，浮空输入 */
#define APU_RST_PORT               GPIOB
#define APU_RST_PIN                GPIO_Pin_3
#define APU_RST_RCC                RCC_APB2Periph_GPIOB

/* USBC_PD_INT: PB11 输入, 高有效。APU 侧推挽驱动，浮空输入。
 * 注意 X035 的 PB 口不支持内部下拉，无法也不需要配置。 */
#define APU_INT_PORT               GPIOB
#define APU_INT_PIN                GPIO_Pin_11
#define APU_INT_RCC                RCC_APB2Periph_GPIOB

/* ---------------- 调试串口 (UART4 @ PA5) ----------------
 * PA5 = USART4_RM=001 的 TX（RM: 001 TX/PA5）。
 * 冲突: PA5 默认复用为 SPI1_SCK(QNF20 上 SPI1 无其他引脚组)。
 * 1 = 串口调试输出: main.c 跳过灯带初始化，调试函数生效（COM26 日志）
 * 0 = LED 状态灯模式: 灯带照常显示状态色（attach/合同/DP/失败），
 *     uart4_dbg 系列函数在 uart4_dbg.h 内退化为空函数，调用点零开销。
 *     （RAM 日志环不受影响，调试器 halt 仍可经 tools/trace_log.py 抓取） */
#define APU_DBG_UART4_ENABLE       1 /* 串口调试 + LED 共存 */
#define APU_DBG_UART4_BAUDRATE     115200
#define APU_DBG_UART4_REMAP        0x1 /* USART4_RM=001b -> TX=PA5 */

/* 置 1: 启动后轮换安全的 USART4_RM 编码发横幅，用于 COM26 上
 * 实测确认 PA5 出字（仅轮换不触碰 PC16/17 的编码），确认后置回 0 */
#define APU_DBG_UART4_REMAP_SCAN   0

/* RAM 日志环的读取工具见 tools/trace_log.py（调试器 halt 抓取） */

/* ---------------- AMD Crossbar 命令通道模式 ----------------
 * 与平台 BIOS 预配置的目标行为一致（文档: 两种模式由平台软件预配置）。
 * Step 2 只读实测当前平台为 Polling 模式（命令码被忽略、INT 不锁存事件），
 * 默认 1=Polling；若换平台为 Interrupt 模式则置 0 切换状态机实现。 */
#define APU_AMU_POLLING_MODE 1

/* Crossbar 命令参数（文档 Chapter 4，两种模式写格式相同）:
 * SW 命令超时 250ms（4.1.3.2），失败按 Table 19 重试 3 次间隔 5ms */
#define APU_CB_CMD_TIMEOUT_MS 250
#define APU_CB_RETRY_MAX      3
#define APU_CB_RETRY_BACKOFF_MS 5

/* USB3 <-> USB3+DP 互切时 Crossbar 在 Safe State 的停留上限（4.1.3.5），
 * 超限会导致 USB3 链路丢失退到 USB2，仅用于越限告警 */
#define APU_CB_SAFE_DWELL_LIMIT_MS 84

/* Polling 稳态状态轮询周期（维护 Crossbar Ready 镜像 / 目标失联检测） */
#define APU_CB_STATUS_POLL_MS 50

/* ---------------- DP Alt 双向支持 ----------------
 * APU 的 DP AUX 极性固定（内部 crossbar 只翻转 ML 通道），反插朝向下
 * SBU1/SBU2 对调导致 AUX 反极性——EDID/DPCD 读不到，DP 协商"成功"也无画面。
 * 实测记录: CC1/Normal 出画面，CC2/Flipped 无输出。
 * 0: 反插时不进入 DP Alt（仅提供 USB3/USB2，日志说明原因）；
 * 1: 板上已加 AUX 极性切换器件（控制脚接固件 GPIO）后置 1 */
#define APU_DP_FLIPPED_SUPPORTED 0

/* ---------------- 临时调试 ----------------
 * I2C 等待超时时转储 I2C/GPIO 寄存器（联调用，正常工作后置 0） */
#define APU_I2C_DEBUG 1

/* ---------------- I2C 实现选择 ----------------
 * 置 0: 硬件 I2C1（SCL=PC16 / SDA=PC17，复用推挽+外部上拉）
 * 之前扫描 timeout 的根因是 G3 期 3V3 上拉未供电、总线浮空导致外设
 * 误锁 BUSY；上拉修复后硬件外设恢复正常，故切回硬件 I2C。
 * 置 1 仍可退回软件 I2C（GPIO 开漏模拟）备用 */
#define APU_I2C_USE_SW 0

/* 软件 I2C 半位周期 (us)，约 100kHz */
#define APU_I2C_SW_HALF_US 5
