#include "usb_pd_policy.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "led_strip.h"
#include "millis.h"
#include "usb_pd_log.h"
#include "usb_pd_phy.h"

#include "apu_amu.h"
#include "apu_config.h"
#include "apu_io.h"

/* ================= PD 定时参数 (ms) ================= */
#define SRC_CAP_SEND_DELAY_MS 160   /* attach 后延时首发 SourceCap */
#define SRC_CAP_RETRY_MAX_MS  2000  /* SourceCap 重试间隔封顶（非 PD 设备退避） */
#define SRC_CAP_STALL_MS      300   /* SourceCap 被 GoodCRC 后等待 Request 的超时，超时发 HardReset */
#define SRC_TRANSITION_MS     25   /* Accept -> PS_RDY 间隔 (tSrcTransition) */
#define VDM_ACK_TIMEOUT_MS    500  /* VDM 请求等待响应超时 */
#define VDM_BUSY_RETRY_MS     100  /* BUSY 响应后重试延时 */
#define VDM_RETRY_MAX         3    /* VDM 请求尝试次数 */

/* ================= SVDM ================= */
#define USB_SID_PD           0xFF00 /* Discover Identity 专用 SVID */
#define USB_SID_DISPLAYPORT  0xFF01 /* DisplayPort Alt Mode SVID */

#define SVDM_CMD_DISCOVER_IDENT 0x01
#define SVDM_CMD_DISCOVER_SVID  0x02
#define SVDM_CMD_DISCOVER_MODES 0x03
#define SVDM_CMD_ENTER_MODE     0x04
#define SVDM_CMD_EXIT_MODE      0x05
#define SVDM_CMD_ATTENTION      0x06
#define SVDM_CMD_DP_STATUS      0x10
#define SVDM_CMD_DP_CONFIGURE   0x11

#define SVDM_CMDT_REQ  0
#define SVDM_CMDT_ACK  1
#define SVDM_CMDT_NAK  2
#define SVDM_CMDT_BUSY 3

#define SVDM_CMDT_MASK (3u << 6)
#define SVDM_HDR(svid, ver, opos, cmdt, cmd) (((uint32_t)(svid) << 16) | (1u << 15) | ((uint32_t)(ver) << 13) | ((uint32_t)(opos) << 8) | ((uint32_t)(cmdt) << 6) | (cmd))

/* ================= DP Alt Mode VDO ================= */
/* DP Capability VDO (DPAM 1.0) */
#define DP_CAP_CAPABILITY_MASK  0x3u
#define DP_CAP_UFP_D           0x1u
#define DP_CAP_DFP_D           0x2u
#define DP_CAP_SIGNALING_DP    (1u << 2)
#define DP_CAP_RECEPTACLE      (1u << 6)
#define DP_CAP_USB2            (1u << 7)
#define DP_CAP_DFP_D_PINS(v)   (((v) >> 8) & 0xFFu)
#define DP_CAP_UFP_D_PINS(v)   (((v) >> 16) & 0xFFu)

/* Pin assignments */
#define DP_PIN_A (1u << 0)
#define DP_PIN_B (1u << 1)
#define DP_PIN_C (1u << 2)
#define DP_PIN_D (1u << 3)
#define DP_PIN_E (1u << 4)
#define DP_PIN_F (1u << 5)

/* DP Status Update VDO */
#define DP_STATUS_CONN_MASK     0x3u
#define DP_STATUS_POWER_LOW      (1u << 2)
#define DP_STATUS_ENABLED        (1u << 3)
#define DP_STATUS_PREFER_MF      (1u << 4)
#define DP_STATUS_SWITCH_USB     (1u << 5)
#define DP_STATUS_EXIT_DP_MODE   (1u << 6)
#define DP_STATUS_HPD_STATE      (1u << 7)
#define DP_STATUS_IRQ_HPD        (1u << 8)

/* DP Configure VDO */
#define DP_CONF_UFP_U_AS_DFP_D   (1u << 0)
#define DP_CONF_UFP_U_AS_UFP_D   (1u << 1)
#define DP_CONF_SIGNALING_DP     (1u << 2)
#define DP_CONF_PIN_SHIFT        8

/* ================= 本设备能力 ================= */
/* 固定 PDO: 5V / 3A（flags 对齐常见 DP 转接线期待: 双角色电源+非受限供电+USB通信+双角色数据+非分块扩展消息） */
#define PD_SRC_PDO_5V3A (((uint32_t)0x2F << 24) | ((uint32_t)100 << 10) | 300)

/* 诊断开关：复刻笔记本的 SourceCap（PD3.0 + 5V3A/9V3A 双 PDO，逐字节一致），
 * 用于判别转接线不请求的原因是否在报文内容。验证后须改回 0（我们实际无 9V 能力） */
#define DIAG_NOTEBOOK_CAPS 0

/* Identity: ID Header + Cert Stat + Product + DFP VDO */
#define PD_ID_HEADER_VDO ((1u << 31) | (1u << 26) | (2u << 23) | (2u << 21) | 0x1A86)
#define PD_CERT_STAT_VDO (0)
#define PD_PRODUCT_VDO   ((0x0350u << 16) | 0x0100u)
#define PD_DFP_VDO       ((1u << 29) | (1u << 24) | (2u << 22) | 1u)

/* Discover SVIDs 应答: DisplayPort + 终止符 */
#define PD_SVID_VDO ((uint32_t)USB_SID_DISPLAYPORT << 16)

/* DP Capability VDO: DFP_D only, DP signaling, receptacle, pin C/D/E, no USB2 */
#define PD_DP_CAP_VDO (DP_CAP_DFP_D | DP_CAP_SIGNALING_DP | DP_CAP_RECEPTACLE | ((DP_PIN_C | DP_PIN_D | DP_PIN_E) << 8))

/* 我们支持的 DFP_D pin assignments: A/B/C/D/E
 * F 为 TBT3 兼容专用，不广告。
 * 映射: A/B/E -> Crossbar Mode2 (DP 4-lane)；C/D -> Mode3 (USB3+2-lane DP)。
 * 注: B 为 2-lane DP-only，由 4-lane crossbar 承载（ML0/ML1 正常工作）。
 * AMD Mode3 正插通道 = VESA C（USB3: TX1=A2/A3 + RX1=B10/B11），
 * 反插时自动等效 VESA D 结构，固件无需区分 C/D。 */
#define PD_DP_DFP_PINS (DP_PIN_A | DP_PIN_B | DP_PIN_C | DP_PIN_D | DP_PIN_E)

/* ================= 状态定义 ================= */
typedef enum {
    PD_SRC_STATE_IDLE = 0,
    PD_SRC_STATE_ATTACHED,     /* 已连接，等待发送 SourceCap */
    PD_SRC_STATE_SRC_CAP_SENT, /* 已发 SourceCap，等待 Request */
    PD_SRC_STATE_ACCEPT_SENT,  /* 已发 Accept，等待发送 PS_RDY */
    PD_SRC_STATE_RUNNING,      /* 合同建立，运行中（VDM/DP 流程） */
} pd_src_state_t;

typedef enum {
    DP_STEP_NONE = 0,
    DP_STEP_DISC_IDENT,
    DP_STEP_DISC_SVID,
    DP_STEP_DISC_MODES,
    DP_STEP_ENTER,
    DP_STEP_STATUS,
    DP_STEP_CONFIGURE,
    DP_STEP_DONE,
    DP_STEP_FAILED,
} dp_step_t;

static pd_src_state_t src_state = PD_SRC_STATE_IDLE;
static dp_step_t dp_step = DP_STEP_NONE;

static uint32_t state_timer = 0;     /* 当前状态起始时刻 */
static uint8_t last_rx_msgid = 0xFF; /* 最近处理的 RX MessageID（0xFF 表示无效） */
static bool dp_started = false;      /* DP 流程是否已启动过（避免重新 Request 后重复启动） */

/* DP 流程状态 */
static uint8_t dp_cmd_pending = 0;  /* 当前等待响应的 SVDM 命令 */
static uint8_t dp_retries = 0;      /* 当前步骤的尝试次数 */
static uint32_t dp_deadline = 0;    /* 当前等待响应的截止时刻 */
static uint8_t dp_busy_wait = 0;    /* BUSY 等待中 */
static uint32_t dp_busy_timer = 0;
static uint32_t dp_partner_mode_vdo = 0;
static uint8_t dp_pin_assign = 0;   /* 选中的 pin assignment (DP_PIN_x) */
static uint32_t src_cap_delay = SRC_CAP_SEND_DELAY_MS; /* SourceCap 当前重试间隔 */
static uint8_t dp_hpd_state = 0xFF; /* 上次记录的 HPD（0xFF 表示未知） */
static bool dp_attention_seen = false; /* 对端已发 Attention（HPD 事件通知）*/
/* HPD 门控规则: Configure ACK 后也不拉高，必须等到对端 Attention 携带
 * HPD/IRQ_HPD 才跟随——防止对端尚未就绪时误触发主机 DP 模式 */

/* ================= AMD Crossbar 协调 ================= */
static uint8_t amu_cc_active = 0;    /* 活动 CC 线（1=CC1, 2=CC2, 0=未连接） */

/* 请求收尾监视与失败冷却 */
static bool amu_req_watch = false;   /* 已发出请求且未观察到收尾 */
static uint32_t amu_fail_ms = 0;     /* 最近一次请求失败时刻（冷却起点） */
static uint32_t amu_last_issue_ms = 0;

#define AMU_REQ_FAIL_COOLDOWN_MS 1000
#define AMU_REQ_DEBOUNCE_MS 50

/**
 * @brief  Crossbar 状态协调器（策略机周期调用）
 *         按策略机当前状态派生期望的 Crossbar 模式，与镜像不一致时发起切换：
 *   - 未连接           -> Safe
 *   - 已连接（未配置DP）-> USB 3.x（纯 USB 设备的 MUX 方向）
 *   - DP Configure ACK -> Pin E: DP 4-lane / Pin C|D: USB3 + DP(Lane0/1)
 * 覆盖: 睡眠唤醒(APU_RST# 周期后镜像重同步)、硬复位、对端 Exit Mode 等所有
 *       需要回落/重配的场景，无需在每个事件点手工补写。
 */
static void amu_reconcile(void) {
    apu_cb_mode_t want;
    uint8_t want_ori;

    /* 镜像无效（复位/挂起后）: 等 amu 的 SYNC 阶段建立镜像后再对账，
     * 避免盲写抢跑导致 post-RST 状态打印被跳过 */
    if (!apu_amu_mirror_valid()) {
        return;
    }

    if (src_state == PD_SRC_STATE_IDLE) {
        want = APU_CB_SAFE;
        want_ori = apu_amu_cb_orientation(); /* Safe 态方向无意义，保持当前 */
    } else if (dp_step == DP_STEP_DONE) {
        want = (dp_pin_assign == DP_PIN_E) ? APU_CB_DP_4LANE : APU_CB_USB3_DP_L01;
        want_ori = APU_ORIENT_FROM_CC(amu_cc_active);
    } else {
        want = APU_CB_USB3;
        want_ori = APU_ORIENT_FROM_CC(amu_cc_active);
    }

    /* 切换进行中: 挂起收尾监视，等待完成 */
    if (apu_amu_cb_busy()) {
        amu_req_watch = true;
        return;
    }
    /* 请求刚收尾: 校验结果，失败则进入冷却 */
    if (amu_req_watch) {
        amu_req_watch = false;
        if (apu_amu_cb_mode() != want || apu_amu_cb_orientation() != want_ori) {
            amu_fail_ms = millis();
            pd_logf("%ums SRC: CB reconcile fail (want MODE=%u got %u)\r\n", millis(),
                    want, apu_amu_cb_mode());
        }
    }

    /* 期望状态已达成（镜像有效且一致）: 无动作 */
    if (apu_amu_cb_mode() == want && apu_amu_cb_orientation() == want_ori &&
        apu_amu_cb_ready()) {
        return;
    }

    /* 失败冷却 / 发出去抖 */
    if ((int32_t)(millis() - amu_fail_ms) < AMU_REQ_FAIL_COOLDOWN_MS ||
        (int32_t)(millis() - amu_last_issue_ms) < AMU_REQ_DEBOUNCE_MS) {
        return;
    }

    apu_amu_request_mode(amu_cc_active, want, want_ori);
    amu_last_issue_ms = millis();
    amu_req_watch = true;
}

/**
 * @brief  HPD 门控派生（策略机周期调用）
 *         HPD = (DP Configure 已 ACK) && (对端 HPD 为高) && (Crossbar DP 通路就绪)
 *         APU_RST# 与门与 Safe 态清零由 apu_io/apu_amu 内部兜底。
 */
static void amu_hpd_derive(void) {
    /* HPD = Configure ACK && 对端已发 Attention && 对端 HPD 为高 && DP 通路就绪 */
    bool want_hpd = (dp_step == DP_STEP_DONE) &&
                    dp_attention_seen &&
                    (dp_hpd_state == 1) &&
                    apu_amu_dp_path_active();
    apu_io_hpd_request(want_hpd);
}

/* CrossBar 派生指示灯状态（边沿刷新，避免重发 WS2812 数据） */
static bool amu_led_dp_pink = false;
static bool amu_led_usb_purple = false;

/**
 * @brief  CrossBar 派生指示灯（策略机周期调用）
 *         优先级: DP 粉(握手 ACK + CrossBar DP 通路完成) > USB3 紫(USB3 配置完成)；
 *         两者都不满足时保持当前色（attach 蓝/绿、合同白、空闲红由事件维护）
 */
static void amu_led_derive(void) {
    if (!apu_amu_mirror_valid() || apu_amu_cb_busy()) {
        return; /* 切换进行中/镜像未同步: 保持当前色 */
    }

    bool pink = apu_amu_dp_path_active();
    bool purple = !pink && (apu_amu_cb_mode() == APU_CB_USB3);

    if (pink != amu_led_dp_pink) {
        amu_led_dp_pink = pink;
        if (pink) {
            led_strip_set_pixel_with_refresh(0, 0x0A, 0x05, 0x06); /* 粉: DP(含 DP+USB) 就绪 */
        }
    }
    if (purple != amu_led_usb_purple) {
        amu_led_usb_purple = purple;
        if (purple) {
            led_strip_set_pixel_with_refresh(0, 0x0A, 0x00, 0x0A); /* 紫: USB3 就绪 */
        }
    }
}

/* ================= 前置声明 ================= */
static uint32_t dp_conf_vdo(void);
static void dp_fail(const char *reason);
static void dp_status_update(uint32_t status);
static void dp_send_req_start(void);
static bool dp_alt_allowed(void);
static void hard_reset_recovery(void);

/* ================= 工具函数 ================= */

/* 从消息数据中读取 32 位数据对象（小端） */
static uint32_t msg_vdo_get(const uint8_t *data, uint8_t index) {
    const uint8_t *p = &data[2 + index * 4];
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static const char *dp_pin_name(uint8_t pin) {
    switch (pin) {
    case DP_PIN_A: return "A";
    case DP_PIN_B: return "B";
    case DP_PIN_C: return "C";
    case DP_PIN_D: return "D";
    case DP_PIN_E: return "E";
    case DP_PIN_F: return "F";
    default: return "?";
    }
}

/* DP Configure VDO: UFP_U 作为 UFP_D（对端为 DP 接收端）+ DP 信号 + pin assignment */
static uint32_t dp_conf_vdo(void) {
    return DP_CONF_UFP_U_AS_UFP_D | DP_CONF_SIGNALING_DP | ((uint32_t)dp_pin_assign << DP_CONF_PIN_SHIFT);
}

/* 发送简单的控制消息 */
static uint8_t send_ctrl(uint8_t msg_type) {
    return usb_pd_phy_send_msg(msg_type, NULL, 0);
}

/* 对不支持的消息按规范版本回 NotSupported / Reject */
static void send_not_supported(void) {
    if (usb_pd_phy_get_spec_rev() == DEF_PD_REVISION_30) {
        send_ctrl(DEF_TYPE_NOT_SUPPORT);
    } else {
        send_ctrl(DEF_TYPE_REJECT);
    }
}

/* 发送 SourceCapabilities（5V3A） */
static void send_source_caps(void) {
#if DIAG_NOTEBOOK_CAPS
    /* 诊断：复刻笔记本 SourceCap（PD3.0 + 5V3A/9V3A） */
    uint32_t pdo[2] = {(((uint32_t)0x2F << 24) | ((uint32_t)100 << 10) | 300),
                       (((uint32_t)180) << 10 | 300)};
    uint8_t num = 2;
    usb_pd_phy_set_spec_rev(DEF_PD_REVISION_30);
#else
    uint32_t pdo[1] = {PD_SRC_PDO_5V3A};
    uint8_t num = 1;
#endif
    bool negotiating = (src_state == PD_SRC_STATE_ATTACHED) || (src_state == PD_SRC_STATE_SRC_CAP_SENT);

    if (usb_pd_phy_send_msg(DEF_TYPE_SRC_CAP, pdo, num) == PD_PHY_TX_OK) {
        pd_logf("%ums SRC: Send SourceCap 5V3A\r\n", millis());
        src_cap_delay = SRC_CAP_SEND_DELAY_MS;
        if (negotiating) {
            src_state = PD_SRC_STATE_SRC_CAP_SENT;
            state_timer = millis();
        }
    } else {
        pd_logf("%ums SRC: SourceCap TX fail\r\n", millis());
        if (src_state == PD_SRC_STATE_SRC_CAP_SENT) {
            /* 周期重发失败，硬复位后重新协商 */
            usb_pd_phy_send_hard_reset();
            hard_reset_recovery();
        } else {
            /* 首发失败，指数退避后重试（非 PD 设备不回 GoodCRC，固定间隔
             * 重试会形成发送风暴拖垮主循环，封顶 2s） */
            state_timer = millis();
            src_cap_delay *= 2;
            if (src_cap_delay > SRC_CAP_RETRY_MAX_MS) {
                src_cap_delay = SRC_CAP_RETRY_MAX_MS;
            }
        }
    }
}

/* 硬复位（收发均可触发）后的状态清理，保持 CC 连接重新协商 */
static void hard_reset_recovery(void) {
    src_cap_delay = SRC_CAP_SEND_DELAY_MS;
    dp_attention_seen = false;
    usb_pd_phy_reset_msgid();
    last_rx_msgid = 0xFF;
    dp_step = DP_STEP_NONE;
    dp_cmd_pending = 0;
    dp_busy_wait = 0;
    dp_hpd_state = 0xFF;
    src_state = PD_SRC_STATE_ATTACHED;
    state_timer = millis();
}

/* ================= DP Alt Mode 流程 ================= */

/* 发送 SVDM 请求并登记等待响应 */
static uint8_t dp_send_req(uint16_t svid, uint8_t opos, uint8_t cmd, uint32_t payload, uint8_t payload_cnt) {
    uint32_t vdos[2];

    vdos[0] = SVDM_HDR(svid, 0, opos, SVDM_CMDT_REQ, cmd);
    if (payload_cnt > 0) {
        vdos[1] = payload;
    }

    if (usb_pd_phy_send_msg(DEF_TYPE_VENDOR_DEFINED, vdos, 1 + payload_cnt) != PD_PHY_TX_OK) {
        /* 发送失败（无 GoodCRC）：计数并置即时超时，由 process 重试 */
        dp_cmd_pending = cmd;
        dp_retries++;
        dp_deadline = millis();
        return PD_PHY_TX_FAIL;
    }

    dp_cmd_pending = cmd;
    dp_retries++;
    dp_deadline = millis() + VDM_ACK_TIMEOUT_MS;
    return PD_PHY_TX_OK;
}

/* 进入新的 DP 步骤：重置该步骤的重试计数 */
static void dp_step_to(dp_step_t step) {
    dp_step = step;
    dp_retries = 0;
}

/* 当前 VDM 步骤的请求重发 */
static void dp_resend_current(void) {
    switch (dp_step) {
    case DP_STEP_DISC_IDENT:
        dp_send_req(USB_SID_PD, 0, SVDM_CMD_DISCOVER_IDENT, 0, 0);
        break;
    case DP_STEP_DISC_SVID:
        dp_send_req(USB_SID_PD, 0, SVDM_CMD_DISCOVER_SVID, 0, 0);
        break;
    case DP_STEP_DISC_MODES:
        dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DISCOVER_MODES, 0, 0);
        break;
    case DP_STEP_ENTER:
        dp_send_req(USB_SID_DISPLAYPORT, 1, SVDM_CMD_ENTER_MODE, 0, 0);
        break;
    case DP_STEP_STATUS:
        dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DP_STATUS, 1, 1);
        break;
    case DP_STEP_CONFIGURE:
        dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DP_CONFIGURE, dp_conf_vdo(), 1);
        break;
    default:
        break;
    }
}

static void dp_fail(const char *reason) {
    dp_step = DP_STEP_FAILED;
    dp_cmd_pending = 0;
    pd_logf("%ums DP: Alt Mode FAILED - %s\r\n", millis(), reason);
    led_strip_set_pixel_with_refresh(0, 0x0A, 0x00, 0x00); // RGB RED
}

/* 解析并记录 DP Status VDO，并跟随 HPD/IRQ_HPD 事件 */
static void dp_status_update(uint32_t status) {
    uint8_t hpd = (status & DP_STATUS_HPD_STATE) ? 1 : 0;
    bool irq_hpd = (status & DP_STATUS_IRQ_HPD) ? true : false;

    pd_logf("%ums DP: Status 0x%08lX HPD:%u%s Enabled:%u LowPower:%u\r\n",
                    millis(), status, hpd,
                    irq_hpd ? "(IRQ)" : "",
                    (status & DP_STATUS_ENABLED) ? 1 : 0,
                    (status & DP_STATUS_POWER_LOW) ? 1 : 0);

    if (dp_hpd_state == 0xFF || dp_hpd_state != hpd) {
        dp_hpd_state = hpd;
        pd_logf("%ums DP: partner HPD -> %s\r\n", millis(), hpd ? "HIGH" : "LOW");
    }

    /* IRQ_HPD: HPD 短脉冲（DP 规范 50~500us），仅在 DP 配置完成后有意义 */
    if (irq_hpd && dp_step == DP_STEP_DONE) {
        apu_io_hpd_irq_pulse();
    }
}

/* ================= VDM 应答构造 ================= */

/* 回复 SVDM（复制请求头，仅替换 Command Type） */
static uint8_t send_vdm_rsp(uint32_t req_vdm_hdr, uint8_t cmdt, const uint32_t *payload, uint8_t payload_cnt) {
    uint32_t vdos[5];
    uint8_t i;

    vdos[0] = (req_vdm_hdr & ~SVDM_CMDT_MASK) | ((uint32_t)cmdt << 6);
    for (i = 0; i < payload_cnt; i++) {
        vdos[1 + i] = payload[i];
    }

    return usb_pd_phy_send_msg(DEF_TYPE_VENDOR_DEFINED, vdos, 1 + payload_cnt);
}

/* 处理对端发来的 SVDM 请求 */
static void handle_vdm_request(uint32_t vh, const uint32_t *vdo, uint8_t cnt) {
    uint16_t svid = (uint16_t)(vh >> 16);
    uint8_t opos = (vh >> 8) & 0x07;
    uint8_t cmd = vh & 0x1F;
    uint32_t rsp[4];
    uint8_t rsp_cnt = 0;
    uint8_t cmdt = SVDM_CMDT_NAK;

    switch (cmd) {
    case SVDM_CMD_DISCOVER_IDENT:
        /* ID Header + Cert Stat + Product + DFP VDO */
        rsp[0] = PD_ID_HEADER_VDO;
        rsp[1] = PD_CERT_STAT_VDO;
        rsp[2] = PD_PRODUCT_VDO;
        rsp[3] = PD_DFP_VDO;
        rsp_cnt = 4;
        cmdt = SVDM_CMDT_ACK;
        break;

    case SVDM_CMD_DISCOVER_SVID:
        rsp[0] = PD_SVID_VDO;
        rsp_cnt = 1;
        cmdt = SVDM_CMDT_ACK;
        break;

    case SVDM_CMD_DISCOVER_MODES:
        if (svid == USB_SID_DISPLAYPORT) {
            rsp[0] = PD_DP_CAP_VDO;
            rsp_cnt = 1;
            cmdt = SVDM_CMDT_ACK;
        }
        break;

    case SVDM_CMD_ENTER_MODE:
        if (svid == USB_SID_DISPLAYPORT && opos == 1) {
            if (dp_alt_allowed()) {
                cmdt = SVDM_CMDT_ACK;
                pd_logf("%ums DP: Partner Enter Mode request -> ACK\r\n", millis());
            } else {
                /* 反插朝向 AUX 不通: NAK，不让对端进入注定失败的 DP 会话 */
                pd_logf("%ums DP: Partner Enter Mode request -> NAK (flipped AUX)\r\n",
                        millis());
            }
        }
        break;

    case SVDM_CMD_EXIT_MODE:
        if (svid == USB_SID_DISPLAYPORT) {
            cmdt = SVDM_CMDT_ACK;
            pd_logf("%ums DP: Partner Exit Mode request -> ACK\r\n", millis());
            if (dp_step == DP_STEP_DONE || dp_started) {
                /* 对端退出 DP 模式: 本端 Alt Mode 会话结束，
                 * Crossbar 由协调器回落（DP通路 -> 经 Safe -> USB3），HPD 随之清零 */
                dp_started = false;
                dp_step = DP_STEP_NONE;
                dp_cmd_pending = 0;
                dp_hpd_state = 0xFF;
                pd_logf("%ums DP: partner exit -> fallback to USB3\r\n", millis());
            }
        }
        break;

    case SVDM_CMD_ATTENTION:
        if (svid == USB_SID_DISPLAYPORT && cnt >= 1) {
            /* Attention 为通知，无需 VDM 应答（GoodCRC 已回）。
             * 首个 Attention 到达前 HPD 保持低（防止误触发主机 DP 模式） */
            pd_logf("%ums DP: Attention received\r\n", millis());
            dp_attention_seen = true;
            dp_status_update(vdo[0]);
            /* 已完成 DP 配置时，重新查询对端状态 */
            if (dp_step == DP_STEP_DONE && dp_started) {
                dp_step_to(DP_STEP_STATUS);
                dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DP_STATUS, 1, 1);
            }
        }
        return;

    default:
        /* DP Status / Configure 等请求对本设备无意义 */
        break;
    }

    send_vdm_rsp(vh, cmdt, rsp, rsp_cnt);
}

/* ================= VDM 响应处理（我们发起的流程） ================= */

static void dp_handle_rsp(uint32_t vh, const uint32_t *vdo, uint8_t cnt) {
    uint8_t cmdt = (vh >> 6) & 0x03;
    uint8_t cmd = vh & 0x1F;
    uint8_t i;
    uint16_t svid;

    /* 只处理当前等待的命令，忽略陈旧响应 */
    if (cmd != dp_cmd_pending) {
        return;
    }
    dp_cmd_pending = 0;

    if (cmdt == SVDM_CMDT_BUSY) {
        if (dp_retries >= VDM_RETRY_MAX) {
            dp_fail("busy");
            return;
        }
        dp_busy_wait = 1;
        dp_busy_timer = millis();
        return;
    }

    if (cmdt != SVDM_CMDT_ACK) {
        dp_fail("NAK");
        return;
    }

    switch (dp_step) {
    case DP_STEP_DISC_IDENT:
        if (cnt >= 3) {
            uint32_t idh = vdo[0];
            pd_logf("%ums DP: Partner ID VID:0x%04lX PID:0x%04lX Modal:%u\r\n",
                            millis(), idh & 0xFFFF, (vdo[2] >> 16) & 0xFFFF,
                            (idh & (1u << 26)) ? 1 : 0);
        }
        dp_step_to(DP_STEP_DISC_SVID);
        dp_send_req(USB_SID_PD, 0, SVDM_CMD_DISCOVER_SVID, 0, 0);
        break;

    case DP_STEP_DISC_SVID:
        /* 遍历 SVID 列表查找 DisplayPort（每个 VDO 携带两个 SVID） */
        for (i = 0; i < cnt; i++) {
            svid = (uint16_t)(vdo[i] >> 16);
            if (svid == USB_SID_DISPLAYPORT) {
                pd_logf("%ums DP: Partner supports DisplayPort SVID\r\n", millis());
                dp_step_to(DP_STEP_DISC_MODES);
                dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DISCOVER_MODES, 0, 0);
                return;
            }
            svid = (uint16_t)(vdo[i] & 0xFFFF);
            if (svid == USB_SID_DISPLAYPORT && svid != 0) {
                pd_logf("%ums DP: Partner supports DisplayPort SVID\r\n", millis());
                dp_step_to(DP_STEP_DISC_MODES);
                dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DISCOVER_MODES, 0, 0);
                return;
            }
        }
        dp_fail("no DP SVID");
        break;

    case DP_STEP_DISC_MODES: {
        if (cnt < 1) {
            dp_fail("no mode VDO");
            break;
        }
        uint32_t cap = vdo[0];
        uint8_t partner_pins;

        dp_partner_mode_vdo = cap;

        /* UFP_D / DFP_D 两字段取并集（兼容 plug/receptacle 字段互换的设备） */
        partner_pins = (DP_CAP_UFP_D_PINS(cap) | DP_CAP_DFP_D_PINS(cap)) & PD_DP_DFP_PINS;

        /* 选择顺序: E(4-lane DP+USB2, 显示器首选) > C/D(USB3+2-lane DP) >
         * A(4-lane 纯 DP) > B(2-lane 纯 DP, 4-lane crossbar 承载) */
        if (partner_pins & DP_PIN_E) {
            dp_pin_assign = DP_PIN_E;
        } else if (partner_pins & DP_PIN_C) {
            dp_pin_assign = DP_PIN_C;
        } else if (partner_pins & DP_PIN_D) {
            dp_pin_assign = DP_PIN_D;
        } else if (partner_pins & DP_PIN_A) {
            dp_pin_assign = DP_PIN_A;
        } else if (partner_pins & DP_PIN_B) {
            dp_pin_assign = DP_PIN_B;
        } else {
            dp_fail("no common pin assignment");
            break;
        }

        pd_logf("%ums DP: Partner cap 0x%08lX cap:%u recept:%u pinsDFP:0x%02X pinsUFP:0x%02X -> pin %s\r\n",
                millis(), cap, cap & DP_CAP_CAPABILITY_MASK,
                (cap & DP_CAP_RECEPTACLE) ? 1 : 0,
                DP_CAP_DFP_D_PINS(cap), DP_CAP_UFP_D_PINS(cap),
                dp_pin_name(dp_pin_assign));
        dp_step_to(DP_STEP_ENTER);
        dp_send_req(USB_SID_DISPLAYPORT, 1, SVDM_CMD_ENTER_MODE, 0, 0);
        break;
    }

    case DP_STEP_ENTER:
        pd_logf("%ums DP: Enter Mode ACK\r\n", millis());
        dp_step_to(DP_STEP_STATUS);
        dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DP_STATUS, 1, 1);
        break;

    case DP_STEP_STATUS:
        if (cnt >= 1) {
            dp_status_update(vdo[0]);
        }
        dp_step_to(DP_STEP_CONFIGURE);
        dp_send_req(USB_SID_DISPLAYPORT, 0, SVDM_CMD_DP_CONFIGURE, dp_conf_vdo(), 1);
        break;

    case DP_STEP_CONFIGURE:
        pd_logf("%ums DP: Configure ACK - DP Alt Mode ACTIVE (pin %s)\r\n",
                        millis(), dp_pin_name(dp_pin_assign));
        dp_step = DP_STEP_DONE;
        /* 点灯由 amu_led_derive 派生: 握手 ACK 且 CrossBar 切换完成才亮粉色 */
        break;

    default:
        break;
    }
}

/* ================= 消息处理 ================= */

/* 处理 Request RDO */
static void handle_request(uint32_t rdo, uint32_t frame_ms) {
    uint8_t objpos = (rdo >> 28) & 0x07;
    uint16_t op_current = ((rdo >> 10) & 0x3FF) * 10;
    uint16_t max_current = (rdo & 0x3FF) * 10;

    if (objpos != 1) {
        pd_logf("%ums SRC: Request invalid objpos %u -> Reject\r\n", millis(), objpos);
        send_ctrl(DEF_TYPE_REJECT);
        return;
    }

    pd_logf("%ums SRC: Request(frame@%ums) 5V op:%umA max:%umA -> Accept\r\n",
            millis(), frame_ms, op_current, max_current);

    if (send_ctrl(DEF_TYPE_ACCEPT) == PD_PHY_TX_OK) {
        pd_logf("%ums SRC: Accept sent ok (resp delay %ums)\r\n", millis(), millis() - frame_ms);
        src_state = PD_SRC_STATE_ACCEPT_SENT;
        state_timer = millis();
    } else {
        /* Accept 发送失败，硬复位重新协商 */
        pd_logf("%ums SRC: Accept TX FAIL\r\n", millis());
        usb_pd_phy_send_hard_reset();
        hard_reset_recovery();
    }
}

void usb_pd_policy_handle_msg(pd_msg_t *msg) {
    uint8_t type;
    uint8_t msgid;
    uint8_t ndo;
    uint8_t rev;
    uint8_t ext;

    if (msg->dir != PD_MSG_DIR_RX) {
        return;
    }

    /* 硬复位事件（RX_RESET，无消息数据） */
    if (msg->status & IF_RX_RESET) {
        pd_logf("%ums SRC: Hard Reset received\r\n", millis());
        hard_reset_recovery();
        return;
    }

    if ((msg->status & MASK_PD_STAT) != PD_RX_SOP0 || msg->len < 2) {
        return; /* SOP'/SOP'' 不处理 */
    }

    type = msg->data[0] & 0x1F;
    rev = (msg->data[0] >> 6) & 0x03;
    msgid = (msg->data[1] >> 1) & 0x07;
    ndo = (msg->data[1] >> 4) & 0x07;
    ext = msg->data[1] >> 7;

    /* GoodCRC 由发送流程处理 */
    if (type == DEF_TYPE_GOODCRC) {
        return;
    }

    /* 跟随对端规范版本 */
    usb_pd_phy_set_spec_rev(rev);

    /* 重复消息不重复处理（GoodCRC 已由中断重发） */
    if (msgid == last_rx_msgid) {
        return;
    }
    last_rx_msgid = msgid;

    /* 扩展消息不支持 */
    if (ext) {
        send_not_supported();
        return;
    }

    switch (type) {
    case DEF_TYPE_REQUEST:
        if (ndo == 1) {
            handle_request(msg_vdo_get(msg->data, 0), msg->timestamp_ms);
        } else {
            send_ctrl(DEF_TYPE_REJECT);
        }
        break;

    case DEF_TYPE_GET_SRC_CAP:
        send_source_caps();
        break;

    case DEF_TYPE_SOFT_RESET:
        /* 软复位：重置 MessageID，合同保持 */
        usb_pd_phy_reset_msgid();
        last_rx_msgid = 0xFF;
        pd_logf("%ums SRC: Soft Reset -> Accept\r\n", millis());
        send_ctrl(DEF_TYPE_ACCEPT);
        break;

    case DEF_TYPE_DR_SWAP:
    case DEF_TYPE_PR_SWAP:
    case DEF_TYPE_VCONN_SWAP:
        pd_logf("%ums SRC: Swap request type %u -> Reject\r\n", millis(), type);
        send_ctrl(DEF_TYPE_REJECT);
        break;

    case DEF_TYPE_PING:
        /* Ping 无需响应 */
        break;

    case DEF_TYPE_VENDOR_DEFINED: {
        if (ndo < 1) {
            break;
        }
        uint32_t vh = msg_vdo_get(msg->data, 0);

        if (!(vh & (1u << 15))) {
            /* 非结构化 VDM 不支持 */
            send_not_supported();
            break;
        }

        uint8_t cmdt = (vh >> 6) & 0x03;
        uint8_t cnt = ndo - 1;
        uint32_t vdos[6];
        for (uint8_t i = 0; i < cnt && i < 6; i++) {
            vdos[i] = msg_vdo_get(msg->data, i + 1);
        }

        if (cmdt == SVDM_CMDT_REQ) {
            handle_vdm_request(vh, vdos, cnt);
        } else {
            dp_handle_rsp(vh, vdos, cnt);
        }
        break;
    }

    case DEF_TYPE_SNK_CAP:
        pd_logf("%ums SRC: SinkCap received (ignore)\r\n", millis());
        break;

    case DEF_TYPE_GET_SNK_CAP:
    case DEF_TYPE_BIST:
    case DEF_TYPE_GET_SRC_CAP_EX:
    case DEF_TYPE_GET_STATUS:
    case DEF_TYPE_GET_PPS_STATUS:
    case DEF_TYPE_GET_CTY_CODES:
    case DEF_TYPE_GET_SNK_CAP_EX:
    case DEF_TYPE_GET_SRC_INFO:
    case DEF_TYPE_GET_REVISION:
    case DEF_TYPE_DATA_RESET:
    case DEF_TYPE_ALERT:
        /* 注：EnterUSB(0x08 数据消息) 与 GetSinkCap(0x08 控制消息) 同值，
         * 走 default 分支统一 NotSupported */
        send_not_supported();
        break;

    default:
        /* 其余未知消息统一按版本回应 */
        send_not_supported();
        break;
    }
}

/* ================= 周期处理 ================= */

/**
 * @brief  当前朝向是否允许进入 DP Alt
 * @note   APU 的 DP AUX 极性固定（实测: CC2/Flipped 协商成功但无画面），
 *         反插朝向下 AUX 反极性，DP 链路必然训练失败——除非板上加装
 *         AUX 极性切换器件（APU_DP_FLIPPED_SUPPORTED 置 1）
 */
static bool dp_alt_allowed(void) {
#if APU_DP_FLIPPED_SUPPORTED
    return true;
#else
    return (APU_ORIENT_FROM_CC(amu_cc_active) == APU_ORIENT_NORMAL);
#endif
}

/* 启动 VDM 流程：Discover Identity（反插朝向且 AUX 不支持时跳过） */
static void dp_send_req_start(void) {
    dp_started = true;

    if (!dp_alt_allowed()) {
        pd_logf("%ums DP: Alt Mode skipped - flipped orientation, AUX polarity fixed\r\n",
                millis());
        return;
    }

    dp_step_to(DP_STEP_DISC_IDENT);
    dp_send_req(USB_SID_PD, 0, SVDM_CMD_DISCOVER_IDENT, 0, 0);
}

void usb_pd_policy_process(void) {
    uint32_t now = millis();

    /* AMD Crossbar 协调与 HPD 门控、DP 指示灯派生（按当前策略状态收敛） */
    amu_reconcile();
    amu_hpd_derive();
    amu_led_derive();

    switch (src_state) {
    case PD_SRC_STATE_ATTACHED:
        /* SourceCap 前先做 SOP' 线缆探测（30/60/90ms 各一次，对齐主机完整序列） */
        {
            uint32_t elapsed = now - state_timer;
            static uint8_t sop1_sent = 0;
            if (elapsed >= (uint32_t)30 * (sop1_sent + 1) && sop1_sent < 3) {
                usb_pd_phy_send_sop1_disc_ident();
                sop1_sent++;
            }
            if (elapsed >= src_cap_delay) {
                sop1_sent = 0;
                send_source_caps();
            }
        }
        break;

    case PD_SRC_STATE_SRC_CAP_SENT:
        if (now - state_timer >= SRC_CAP_STALL_MS) {
            /* SourceCap 已被 GoodCRC 但 300ms 无 Request：
             * 按用户实验要求发 HardReset 踢对端策略层重新协商 */
            pd_logf("%ums SRC: No Request after GoodCRC -> HardReset\r\n", millis());
            usb_pd_phy_send_hard_reset();
            hard_reset_recovery();
        }
        break;

    case PD_SRC_STATE_ACCEPT_SENT:
        if (now - state_timer >= SRC_TRANSITION_MS) {
            /* tSrcTransition 后发送 PS_RDY */
            if (send_ctrl(DEF_TYPE_PS_RDY) == PD_PHY_TX_OK) {
                pd_logf("%ums SRC: Contract established 5V3A\r\n", millis());
                led_strip_set_pixel_with_refresh(0, 0x0A, 0x0A, 0x0A); /* 白: PD 合同建立 */
                src_state = PD_SRC_STATE_RUNNING;
                state_timer = millis();
                /* 首次建立合同后启动 DP Alt Mode VDM 握手 */
                if (!dp_started) {
                    dp_send_req_start();
                }
            } else {
                usb_pd_phy_send_hard_reset();
                hard_reset_recovery();
            }
        }
        break;

    case PD_SRC_STATE_RUNNING:
        /* DP Alt Mode VDM 流程驱动 */
        if (dp_step == DP_STEP_NONE) {
            /* 合同建立后启动 VDM 握手 */
            if (!dp_started) {
                dp_send_req_start();
            }
        } else if (dp_step < DP_STEP_DONE) {
            if (dp_busy_wait) {
                if (now - dp_busy_timer >= VDM_BUSY_RETRY_MS) {
                    dp_busy_wait = 0;
                    dp_resend_current();
                }
            } else if ((int32_t)(now - dp_deadline) >= 0) {
                /* 响应超时：重试当前步骤，超限则失败 */
                if (dp_retries >= VDM_RETRY_MAX) {
                    dp_fail("timeout");
                } else {
                    dp_resend_current();
                }
            }
        }
        break;

    default:
        break;
    }
}

/* ================= 事件回调 ================= */

void usb_pd_policy_event_attach(uint8_t cc) {
    pd_logf("%ums SRC: Attached on CC%u, negotiating\r\n", millis(), cc);
    src_state = PD_SRC_STATE_ATTACHED;
    amu_cc_active = cc;
    dp_attention_seen = false;
    state_timer = millis();
    last_rx_msgid = 0xFF;
    dp_started = false;
    dp_step = DP_STEP_NONE;
    dp_cmd_pending = 0;
    dp_busy_wait = 0;
    dp_hpd_state = 0xFF;
    usb_pd_phy_reset_msgid();
    /* 纯 USB 设备插入: 立即请求 Crossbar 的 USB3 方向
     * （DP 通路由 Configure ACK 后的协调器接管） */
    amu_fail_ms = 0; /* 新连接给协调器一个干净的重试窗口 */
    pd_logf("%ums SRC: AMU MUX -> USB3, CC%u ORI=%s(%u)\r\n", millis(), cc,
            (APU_ORIENT_FROM_CC(cc) == APU_ORIENT_FLIPPED) ? "Flipped" : "Normal",
            APU_ORIENT_FROM_CC(cc));
}

void usb_pd_policy_event_detach(void) {
    if (src_state != PD_SRC_STATE_IDLE) {
        pd_logf("%ums SRC: Detached\r\n", millis());
    }
    src_state = PD_SRC_STATE_IDLE;
    amu_cc_active = 0;
    dp_attention_seen = false;
    amu_req_watch = false;
    amu_fail_ms = 0;
    dp_started = false;
    dp_step = DP_STEP_NONE;
    dp_cmd_pending = 0;
    dp_busy_wait = 0;
    dp_hpd_state = 0xFF;
    last_rx_msgid = 0xFF;
}

bool usb_pd_policy_contracted(void) {
    return src_state == PD_SRC_STATE_RUNNING;
}

bool usb_pd_policy_dp_active(void) {
    return dp_step == DP_STEP_DONE;
}

void usb_pd_policy_init(void) {
    src_state = PD_SRC_STATE_IDLE;
    dp_started = false;
    dp_step = DP_STEP_NONE;
}
