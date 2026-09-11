#include "apu_amu.h"

#include "apu_config.h"
#include "apu_i2c.h"
#include "apu_io.h"
#include "debug.h"
#include "millis.h"
#include "usb_pd_log.h"

/* ============================================================
 * AMD USB PD I2C Target 协议层
 *  - 通信通道模式判定（只读，Step 2）
 *  - Crossbar 命令状态机：Polling / Interrupt 双实现，
 *    由 APU_AMU_POLLING_MODE 宏选择（文档 Chapter 4.1 / 4.2）
 * ============================================================ */

/* ---------------- 通信通道模式判定（只读） ---------------- */
static apu_amu_mode_t link_mode = APU_AMU_MODE_UNKNOWN;
static bool detect_done = false;
static uint8_t detect_retries = 0;
static uint32_t retry_after_ms = 0;
static bool mode_mismatch_warned = false;

/* 判定重试冷却与上限（目标写路径复位后需要时间就绪） */
#define DETECT_RETRY_COOLDOWN_MS 200
#define DETECT_RETRY_MAX 30

/* ---------------- Crossbar 状态机簿记 ---------------- */
static apu_cb_mode_t cb_mirror_mode = APU_CB_SAFE; /* 最近一次成功完成的模式 */
static uint8_t cb_mirror_ori = APU_ORIENT_NORMAL;
static bool cb_mirror_valid = false;   /* 复位/挂起后无效，需重新同步 */
static bool link_cb_ready = false;     /* Crossbar Ready 镜像 */

static volatile bool cb_req_active = false;
static apu_cb_mode_t cb_req_mode = APU_CB_SAFE;
static uint8_t cb_req_ori = APU_ORIENT_NORMAL;
static uint8_t cb_req_cc = 0;          /* 活动 CC（1/2），仅日志 */

typedef enum {
    AMU_PH_IDLE = 0,   /* 无请求 */
    AMU_PH_SYNC,       /* 复位/恢复后首次状态同步（打印+检测，非 Safe 则写 Safe） */
    AMU_PH_CHECK,      /* 写前状态检查（Ready + 前序命令完成） */
    AMU_PH_SAFE,       /* SAFE 步已发，等完成 */
    AMU_PH_TARGET,     /* 目标步已发，等完成 */
    AMU_PH_BACKOFF,    /* 失败重试冷却 */
} amu_phase_t;

static amu_phase_t amu_phase = AMU_PH_IDLE;
static bool amu_step_safe = false;     /* 当前等待的步是否 SAFE 步 */
static bool amu_did_safe = false;      /* 本次请求是否走过 SAFE 步（84ms 停留监测用） */
static bool amu_sync_safe = false;     /* 复位后同步写 Safe（不占用用户请求字段） */
static uint8_t amu_retries = 0;
static uint32_t amu_deadline = 0;      /* 当前步 250ms 软超时 */
static uint32_t amu_next_poll_ms = 0;  /* 轮询/重试节拍 */
static uint32_t amu_safe_done_ms = 0;  /* SAFE 完成时刻（84ms 停留监测） */
static uint32_t amu_write_start_ms = 0; /* 当前命令写入时刻（in-progress 等待上限） */
static uint32_t amu_sync_fail_log_ms = 0; /* 同步失败日志节流 */

/* ================= 工具 ================= */

/* 状态读回 3 字节解析（Polling 直读与 Interrupt Type-3 同布局）:
 * B0={P1_status[1:0], P1_ctl[5:0]}  B1={P0_status[1:0], P0_ctl[5:0]}
 * B2={USB_PD_Status[6:0]<<1, Error}   CrossbarReady=B2 bit6 */
static uint8_t st_p1_status(const uint8_t *b) { return (uint8_t)(b[0] >> 6); }
static uint8_t st_p0_status(const uint8_t *b) { return (uint8_t)(b[1] >> 6); }
static uint8_t st_p0_ctl(const uint8_t *b)    { return (uint8_t)(b[1] & 0x3F); }
static bool st_ready(const uint8_t *b)        { return (b[2] & 0x40) != 0; }
static bool st_error(const uint8_t *b)        { return (b[2] & 0x01) != 0; }

static void log_status3(const char *tag, apu_i2c_result_t r, const uint8_t *b) {
    if (r != APU_I2C_OK) {
        pd_logf("%ums APU: %-10s read fail res=%d\r\n", millis(), tag, r);
        return;
    }
    pd_logf("%ums APU: %-10s [%02X %02X %02X] P0st=%u P0ctl=0x%02X P1st=%u cbReady=%u err=%u\r\n",
            millis(), tag, b[0], b[1], b[2],
            st_p0_status(b), st_p0_ctl(b), st_p1_status(b),
            st_ready(b) ? 1 : 0, st_error(b) ? 1 : 0);
}

static const char *cb_mode_name(apu_cb_mode_t m) {
    switch (m) {
    case APU_CB_SAFE: return "Safe";
    case APU_CB_USB3: return "USB3";
    case APU_CB_DP_4LANE: return "DP4";
    case APU_CB_USB3_DP_L01: return "USB3+DP";
    default: return "?";
    }
}

static const char *ori_name(uint8_t o) {
    return (o == APU_ORIENT_FLIPPED) ? "Flipped" : "Normal";
}

static const char *port_status_name(uint8_t s) {
    switch (s) {
    case 0: return "inprog";
    case 1: return "complete";
    case 2: return "timeout";
    default: return "resv";
    }
}

/* 写控制字节: bit5=LP bit4=ORI bits[3:0]=Mode */
static uint8_t cb_ctrl_byte(bool safe_step, apu_cb_mode_t mode, uint8_t ori) {
    if (safe_step) {
        return (uint8_t)(0x20u | ((ori & 1u) << 4)); /* Safe: LP=1, Mode=0 */
    }
    return (uint8_t)(((ori & 1u) << 4) | ((uint8_t)mode & 0x0F));
}

static apu_i2c_result_t cb_write_ctrl(uint8_t ctrl) {
    uint8_t payload[2] = {0x00, ctrl}; /* Index=0 -> Port 0 */
    return apu_i2c_write(APU_I2C_TARGET_ADDR_7BIT, payload, 2);
}

static bool step_timed_out(void) {
    return (int32_t)(millis() - amu_deadline) >= 0;
}

/* 步骤失败处理: 按 Table 19 重试 3 次间隔 5ms，超限放弃（镜像不变） */
static void step_fail_retry(const char *what, const char *detail) {
    if (++amu_retries <= APU_CB_RETRY_MAX) {
        pd_logf("%ums APU: CB %s %s -> retry %u/%u\r\n", millis(), what, detail,
                amu_retries, APU_CB_RETRY_MAX);
        amu_next_poll_ms = millis() + APU_CB_RETRY_BACKOFF_MS;
        amu_phase = AMU_PH_BACKOFF;
        amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
    } else {
        pd_logf("%ums APU: CB req CC%u ORI=%s MODE=%s FAILED (%s)\r\n", millis(),
                cb_req_cc, ori_name(cb_req_ori), cb_mode_name(cb_req_mode), detail);
        amu_phase = AMU_PH_IDLE;
        cb_req_active = false;
    }
}

/* 非 DP 模式完成时自动收回 HPD 请求（文档 HPD 规则的兜底） */
static void cb_apply_hpd_rule(apu_cb_mode_t done_mode) {
    if (cb_mirror_valid &&
        cb_mirror_mode != APU_CB_DP_4LANE && cb_mirror_mode != APU_CB_USB3_DP_L01) {
        apu_io_hpd_request(false);
    }
}

/* ================= 通信通道模式判定（Step 2，只读） ================= */

static uint8_t int_edges_consume(void) {
    uint32_t ts;
    return apu_io_int_consume(&ts);
}

/**
 * @brief  一次性模式判定（只读：仅状态/命令码读，不写 Crossbar）
 * @return false: 目标写路径未就绪（复位后初期的 NACK），需稍后重试
 */
static bool detect_mode_once(void) {
    const uint8_t addr = APU_I2C_TARGET_ADDR_7BIT;
    uint8_t cmd;
    uint8_t st3[3] = {0};
    uint8_t r1[3] = {0};
    uint8_t t5 = 0xFF;
    uint8_t r0_b0, t3_b0;
    apu_i2c_result_t r;
    uint8_t edges;

    pd_logf("%ums APU: === mode detect (read-only) ===\r\n", millis());

    /* R0: 直读 3 字节，顺带清掉历史 Error 位 */
    r = apu_i2c_read(addr, st3, 3);
    log_status3("plain R0", r, st3);
    if (r != APU_I2C_OK) {
        return false;
    }
    r0_b0 = st3[0];
    edges = int_edges_consume();
    if (edges) {
        pd_logf("%ums APU: INT edges after R0: %u\r\n", millis(), edges);
    }

    /* T5: 写 0xA2 + 重复启动读 1 字节（Interrupt 模式合法; Polling 非法 Index）
     * 复位刚结束时目标写路径可能未就绪（NACK），此结果不可用作判定 */
    cmd = 0xA2;
    r = apu_i2c_write_read(addr, &cmd, 1, &t5, 1);
    if (r != APU_I2C_OK) {
        pd_logf("%ums APU: T5(0xA2) fail res=%d (target write path not ready?)\r\n",
                millis(), r);
        return false;
    }
    pd_logf("%ums APU: T5(0xA2) -> %02X rtReady=%u mbox=%u err=%u cmdDone=%u cbChg=%u\r\n",
            millis(), t5, (t5 >> 7) & 1, (t5 >> 3) & 1, (t5 >> 2) & 1,
            (t5 >> 1) & 1, t5 & 1);
    edges = int_edges_consume();
    if (edges) {
        pd_logf("%ums APU: INT edges after T5: %u\r\n", millis(), edges);
    }

    /* T3: 写 0x80 + 重复启动读 3 字节（Interrupt 状态寄存器） */
    cmd = 0x80;
    r = apu_i2c_write_read(addr, &cmd, 1, st3, 3);
    log_status3("T3(0x80)", r, st3);
    if (r != APU_I2C_OK) {
        return false;
    }
    t3_b0 = st3[0];

    /* R1: 再直读 3 字节，Error 位反映 T5/T3 写入的合法性 */
    r = apu_i2c_read(addr, r1, 3);
    log_status3("plain R1", r, r1);
    if (r != APU_I2C_OK) {
        return false;
    }

    bool err_latched = st_error(r1);
    bool cb_ready = st_ready(r1);
    bool t5_rt_match = ((t5 & 0x80) != 0) == cb_ready;
    bool t5_echo = (t5 == r0_b0) || (t5 == t3_b0);

    if (err_latched || t5_echo) {
        link_mode = APU_AMU_MODE_POLLING;
    } else if (t5_rt_match) {
        link_mode = APU_AMU_MODE_INTERRUPT;
    } else {
        link_mode = APU_AMU_MODE_UNKNOWN;
    }
    return true;
}

/* ================= Polling 模式状态机（4.1） ================= */

#if APU_AMU_POLLING_MODE

/**
 * @brief  Polling 状态机：直读状态轮询，写=2字节(Index+控制)，完成靠轮询
 */
static void amu_sm_polling(void) {
    uint8_t st[3];
    apu_i2c_result_t r;
    uint8_t stt;

    switch (amu_phase) {
    case AMU_PH_IDLE:
        return;

    case AMU_PH_SYNC: {
        /* APU 复位/恢复后首次状态读取:
         * 1) 打印读回的 Crossbar 状态（P0_ctl 回显解码 ORI/MODE/LP）
         * 2) 非 Safe 且可写（Ready + 无命令进行中）时立刻写 Safe */
        if ((int32_t)(millis() - amu_next_poll_ms) < 0) {
            return;
        }
        amu_next_poll_ms = millis() + APU_CB_STATUS_POLL_MS;

        r = apu_i2c_read(APU_I2C_TARGET_ADDR_7BIT, st, 3);
        if (r != APU_I2C_OK) {
            /* 目标读路径未就绪（S5 掉电/复位初期），节流日志后继续等 */
            if (millis() - amu_sync_fail_log_ms >= 2000) {
                amu_sync_fail_log_ms = millis();
                pd_logf("%ums APU: post-RST sync read fail res=%d, waiting\r\n", millis(), r);
            }
            return;
        }

        link_cb_ready = st_ready(st);
        {
            uint8_t ctl = st_p0_ctl(st);
            uint8_t mode = (uint8_t)(ctl & 0x0F);
            uint8_t ori = (uint8_t)((ctl >> 4) & 1);
            bool lp = (ctl & 0x20) != 0;

            pd_logf("%ums APU: post-RST CB state: P0st=%s cbReady=%u ctl=0x%02X -> CC? ORI=%s(%u) MODE=%s(%u) LP=%u\r\n",
                    millis(), port_status_name(st_p0_status(st)),
                    link_cb_ready ? 1 : 0, ctl,
                    ori_name(ori), ori, cb_mode_name((apu_cb_mode_t)mode), mode, lp ? 1 : 0);

            /* 有命令进行中时不可写，等其完成后再评估 */
            if (st_p0_status(st) == 0) {
                return;
            }

            cb_mirror_mode = (apu_cb_mode_t)mode;
            cb_mirror_ori = ori;
            cb_mirror_valid = true;
            cb_apply_hpd_rule(cb_mirror_mode);

            /* 不再无条件写 Safe: 文档规定 warm reset 后 crossbar 保持最后成功
             * 状态（实测 DP4 跨重启保持），无谓的 Safe 写会打断正常显示。
             * "回 Safe"的安全基线由 reconciler 派生（IDLE->want=Safe）覆盖；
             * 有待执行请求则进入其写前检查，否则收工 */
            if (cb_req_active) {
                amu_next_poll_ms = 0;
                amu_phase = AMU_PH_CHECK;
            } else {
                amu_phase = AMU_PH_IDLE;
            }
        }
        return;
    }

    case AMU_PH_CHECK:
        if ((int32_t)(millis() - amu_next_poll_ms) < 0) {
            return;
        }
        amu_next_poll_ms = millis() + APU_CB_RETRY_BACKOFF_MS;

        r = apu_i2c_read(APU_I2C_TARGET_ADDR_7BIT, st, 3);
        if (r != APU_I2C_OK) {
            if (step_timed_out()) {
                step_fail_retry("check", r == APU_I2C_NACK ? "nack" : "timeout");
            }
            return;
        }

        link_cb_ready = st_ready(st);
        if (!link_cb_ready) {
            /* Crossbar 未就绪（SOC Reset 中/S5），持续等待直到超时 */
            if (step_timed_out()) {
                step_fail_retry("check", "crossbar not ready");
            }
            return;
        }
        if (st_p0_status(st) == 0) {
            /* 前一条命令仍在进行，禁止写。
             * SoC 控制器未就绪时（内存训练/BIOS 初始化）握手可能远超
             * 250ms——顺延 deadline 持续等待，总时长超过看门狗才放弃 */
            static uint32_t chk_raw_log_ms = 0;
            if (millis() - chk_raw_log_ms >= 1000) {
                chk_raw_log_ms = millis();
                pd_logf("%ums APU: CB check raw [%02X %02X %02X] prev-in-progress (waiting)\r\n",
                        millis(), st[0], st[1], st[2]);
            }
            if (step_timed_out()) {
                if (millis() - amu_write_start_ms < APU_CB_WAIT_MAX_MS) {
                    amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
                } else {
                    step_fail_retry("check", "prev cmd in progress");
                }
            }
            return;
        }

        /* 写下一步（镜像未知或非 Safe 且有变化时先走 Safe 步） */
        amu_step_safe = !cb_mirror_valid ||
                        ((cb_mirror_mode != APU_CB_SAFE) &&
                         (cb_req_mode != cb_mirror_mode || cb_req_ori != cb_mirror_ori));
        {
            uint8_t ctrl = cb_ctrl_byte(amu_step_safe, cb_req_mode, cb_req_ori);
            pd_logf("%ums APU: CB write [%02X %02X] CC%u ORI=%s(%u) MODE=%s(%u)%s\r\n",
                    millis(), 0x00, ctrl, cb_req_cc ? cb_req_cc : 0,
                    ori_name(cb_req_ori), cb_req_ori,
                    cb_mode_name(amu_step_safe ? APU_CB_SAFE : cb_req_mode),
                    amu_step_safe ? 0 : cb_req_mode,
                    amu_step_safe ? " [safe step]" : "");
        }
        amu_write_start_ms = millis();
        r = cb_write_ctrl(cb_ctrl_byte(amu_step_safe, cb_req_mode, cb_req_ori));
        amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
        if (r != APU_I2C_OK) {
            /* 写本身失败（NACK/超时）→ 走重试 */
            step_fail_retry("write", r == APU_I2C_NACK ? "nack" : "timeout");
            return;
        }
        amu_phase = amu_step_safe ? AMU_PH_SAFE : AMU_PH_TARGET;
        return;

    case AMU_PH_SAFE:
    case AMU_PH_TARGET:
        if ((int32_t)(millis() - amu_next_poll_ms) < 0) {
            return;
        }
        amu_next_poll_ms = millis() + APU_CB_RETRY_BACKOFF_MS;

        r = apu_i2c_read(APU_I2C_TARGET_ADDR_7BIT, st, 3);
        if (r != APU_I2C_OK) {
            if (step_timed_out()) {
                step_fail_retry("poll", r == APU_I2C_NACK ? "nack" : "timeout");
            }
            return;
        }

        link_cb_ready = st_ready(st);
        stt = st_p0_status(st);
        if (stt == 0) {
            /* 命令进行中: 顺延 deadline 持续轮询，总看门狗封顶后放弃 */
            static uint32_t poll_raw_log_ms = 0;
            if (millis() - poll_raw_log_ms >= 1000) {
                poll_raw_log_ms = millis();
                pd_logf("%ums APU: CB poll raw [%02X %02X %02X] step=%s (waiting)\r\n", millis(),
                        st[0], st[1], st[2], amu_phase == AMU_PH_SAFE ? "safe" : "target");
            }
            if (step_timed_out()) {
                if (millis() - amu_write_start_ms < APU_CB_WAIT_MAX_MS) {
                    amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
                } else {
                    step_fail_retry("poll", "cmd still in progress");
                }
            }
            return;
        }
        if (stt == 1) {
            /* 命令完成 */
            if (amu_phase == AMU_PH_SAFE) {
                amu_safe_done_ms = millis();
                amu_did_safe = true;
                cb_mirror_mode = APU_CB_SAFE; /* Crossbar 已进入 Safe */
                if (!amu_sync_safe) {
                    cb_mirror_ori = cb_req_ori; /* 用户请求: 用请求方向 */
                }
                /* 同步步: 保留 SYNC 时按当前态写入镜像的方向 */
                cb_mirror_valid = true;
                cb_apply_hpd_rule(cb_mirror_mode); /* Safe 态强制清 HPD 请求 */
                pd_logf("%ums APU: CB SAFE complete (CC%u)%s\r\n", millis(), cb_req_cc,
                        amu_sync_safe ? " (post-RST sync)" : "");
                if (amu_sync_safe) {
                    amu_sync_safe = false;
                    /* 同步完成: 有用户请求则接其目标步，否则收工 */
                    amu_phase = cb_req_active ? AMU_PH_CHECK : AMU_PH_IDLE;
                    amu_next_poll_ms = 0;
                } else if (cb_req_mode == APU_CB_SAFE) {
                    /* 用户请求目标就是 Safe: 完成 */
                    amu_phase = AMU_PH_IDLE;
                    cb_req_active = false;
                } else {
                    amu_phase = AMU_PH_CHECK; /* 下一步: 目标模式 */
                    amu_next_poll_ms = 0;     /* 立即进入 CHECK */
                }
            } else {
                uint32_t dwell = millis() - amu_safe_done_ms;
                cb_mirror_mode = cb_req_mode;
                cb_mirror_ori = cb_req_ori;
                cb_mirror_valid = true;
                pd_logf("%ums APU: CB done: CC%u ORI=%s(%u) MODE=%s(%u) P0st=01b\r\n",
                        millis(), cb_req_cc ? cb_req_cc : 0,
                        ori_name(cb_req_ori), cb_req_ori,
                        cb_mode_name(cb_req_mode), cb_req_mode);
                if (amu_did_safe && dwell > APU_CB_SAFE_DWELL_LIMIT_MS) {
                    pd_logf("%ums APU: WARN safe dwell %ums > %ums\r\n", millis(), dwell,
                            APU_CB_SAFE_DWELL_LIMIT_MS);
                }
                amu_did_safe = false;
                cb_apply_hpd_rule(cb_req_mode);
                amu_phase = AMU_PH_IDLE;
                cb_req_active = false;
            }
        } else {
            /* 10b 命令超时 / 11b 保留 -> 按失败重试 */
            step_fail_retry("poll", stt == 2 ? "cmd timeout" : "resv status");
        }
        return;

    case AMU_PH_BACKOFF:
        if ((int32_t)(millis() - amu_next_poll_ms) >= 0) {
            amu_phase = AMU_PH_CHECK;
        }
        return;

    default:
        amu_phase = AMU_PH_IDLE;
        return;
    }
}

#else /* Interrupt 模式状态机（4.2） */

typedef enum {
    AMI_PH_IDLE = 0,     /* 无请求 */
    AMI_PH_SYNC,         /* 镜像未就绪: Type-5 同步 Crossbar Ready */
    AMI_PH_WAIT_READY,   /* 等 Ready 事件（INT） */
    AMI_PH_WRITE,        /* Type-1 写（SAFE/TARGET 步） */
    AMI_PH_WAIT_INT,     /* 等完成中断（电平或边沿） */
    AMI_PH_TYPE5,        /* 读中断状态并分发 */
    AMI_PH_TYPE3,        /* 读端口状态判定 */
    AMI_PH_MBOX,         /* 读 APU 邮箱（Type-4，暂仅日志） */
    AMI_PH_BACKOFF,
} ami_phase_t;

static ami_phase_t ami_phase = AMI_PH_IDLE;
static bool ami_step_safe = false;

/* 等待 INT 的轮询间隔（共享中断线时对其他目标的中断降频应答） */
#define AMI_INT_POLL_MS 20

/**
 * @brief  读 Type-5 中断状态（0xA2 -> 1 字节）
 */
static apu_i2c_result_t int_read_type5(uint8_t *st) {
    uint8_t cmd = 0xA2;
    return apu_i2c_write_read(APU_I2C_TARGET_ADDR_7BIT, &cmd, 1, st, 1);
}

/**
 * @brief  读 Type-3 端口状态（0x80 -> 3 字节）
 */
static apu_i2c_result_t int_read_type3(uint8_t *st) {
    uint8_t cmd = 0x80;
    return apu_i2c_write_read(APU_I2C_TARGET_ADDR_7BIT, &cmd, 1, st, 3);
}

/**
 * @brief  读 Type-4 APU 邮箱（0xA0 -> 4 字节，暂仅日志）
 */
static apu_i2c_result_t int_read_type4(uint8_t *st) {
    uint8_t cmd = 0xA0;
    return apu_i2c_write_read(APU_I2C_TARGET_ADDR_7BIT, &cmd, 1, st, 4);
}

static void amu_sm_interrupt(void) {
    uint8_t st[4];
    apu_i2c_result_t r;
    uint32_t edges;
    uint8_t stt;

    switch (ami_phase) {
    case AMI_PH_IDLE:
        return;

    case AMI_PH_SYNC:
        /* 镜像失同步（初始化/复位后）: Type-5 bit7 实时 Ready 重新同步 */
        if ((int32_t)(millis() - amu_next_poll_ms) < 0) {
            return;
        }
        amu_next_poll_ms = millis() + AMI_INT_POLL_MS;

        r = int_read_type5(st);
        if (r != APU_I2C_OK) {
            if (step_timed_out()) {
                step_fail_retry("sync", r == APU_I2C_NACK ? "nack" : "timeout");
            }
            return;
        }
        link_cb_ready = (st[0] & 0x80) != 0;
        if (link_cb_ready) {
            pd_logf("%ums APU: CB mirror synced: ready=1 (T5=%02X)\r\n", millis(), st[0]);

            /* 镜像未知（复位后）: 读一次 Type-3 捕获当前 Crossbar 状态并打印；
             * 非 Safe 且可写（P0st!=00b）时立刻写 Safe */
            if (!cb_mirror_valid) {
                r = int_read_type3(st);
                if (r != APU_I2C_OK) {
                    /* 读失败则回退等待，下次再捕获 */
                    amu_next_poll_ms = millis() + AMI_INT_POLL_MS;
                    ami_phase = AMI_PH_WAIT_READY;
                    return;
                }
                log_status3("post-RST T3", r, st);
                {
                    uint8_t ctl = st_p0_ctl(st);
                    uint8_t mode = (uint8_t)(ctl & 0x0F);
                    uint8_t ori = (uint8_t)((ctl >> 4) & 1);

                    cb_mirror_mode = (apu_cb_mode_t)mode;
                    cb_mirror_ori = ori;
                    cb_mirror_valid = true;
                    cb_apply_hpd_rule(cb_mirror_mode);

                    if (mode != APU_CB_SAFE && st_p0_status(st) != 0) {
                        /* 非 Safe 且可写: 立刻写 Safe（同步步，不覆盖用户请求） */
                        uint8_t ctrl = cb_ctrl_byte(true, APU_CB_SAFE, ori);
                        pd_logf("%ums APU: post-RST CB not Safe -> write Safe now\r\n",
                                millis());
                        pd_logf("%ums APU: CB write [%02X %02X] CC? ORI=%s(%u) MODE=Safe(0)%s\r\n",
                                millis(), 0x00, ctrl, ori_name(ori), ori, " [sync step]");
                        r = cb_write_ctrl(ctrl);
                        if (r != APU_I2C_OK) {
                            ami_phase = AMI_PH_SYNC;
                            return;
                        }
                        ami_step_safe = true;
                        amu_did_safe = true;
                        amu_sync_safe = true;
                        ami_phase = AMI_PH_WAIT_INT;
                        amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
                        return;
                    }
                }
            }

            if (cb_req_active) {
                ami_phase = AMI_PH_WRITE;
                amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
            } else {
                ami_phase = AMI_PH_IDLE;
            }
        } else {
            /* 未就绪: 等 INT（Ready 事件会触发），回退等待 */
            if (step_timed_out()) {
                step_fail_retry("sync", "crossbar not ready");
                return;
            }
            ami_phase = AMI_PH_WAIT_READY;
        }
        return;

    case AMI_PH_WAIT_READY:
        edges = apu_io_int_consume(NULL);
        if (edges > 0 || apu_io_int_asserted()) {
            ami_phase = AMI_PH_TYPE5;
            amu_next_poll_ms = 0; /* 立即读 */
        } else if (step_timed_out()) {
            step_fail_retry("wait int", "no interrupt");
        }
        return;

    case AMI_PH_WRITE:
        {
            uint8_t ctrl = cb_ctrl_byte(ami_step_safe, cb_req_mode, cb_req_ori);
            pd_logf("%ums APU: CB write [%02X %02X] CC%u ORI=%s(%u) MODE=%s(%u)%s\r\n",
                    millis(), 0x00, ctrl, cb_req_cc ? cb_req_cc : 0,
                    ori_name(cb_req_ori), cb_req_ori,
                    cb_mode_name(ami_step_safe ? APU_CB_SAFE : cb_req_mode),
                    ami_step_safe ? 0 : cb_req_mode,
                    ami_step_safe ? " [safe step]" : "");
        }
        r = cb_write_ctrl(cb_ctrl_byte(ami_step_safe, cb_req_mode, cb_req_ori));
        if (r != APU_I2C_OK) {
            step_fail_retry("write", r == APU_I2C_NACK ? "nack" : "timeout");
            return;
        }
        amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
        ami_phase = AMI_PH_WAIT_INT;
        return;

    case AMI_PH_WAIT_INT:
        edges = apu_io_int_consume(NULL);
        if (edges > 0 || apu_io_int_asserted()) {
            ami_phase = AMI_PH_TYPE5;
            amu_next_poll_ms = 0;
            return;
        }
        if (step_timed_out()) {
            step_fail_retry("wait int", "handshake interrupt timeout");
        }
        return;

    case AMI_PH_TYPE5:
        if ((int32_t)(millis() - amu_next_poll_ms) < 0) {
            return;
        }
        r = int_read_type5(st);
        if (r != APU_I2C_OK) {
            if (step_timed_out()) {
                step_fail_retry("type5", r == APU_I2C_NACK ? "nack" : "timeout");
            }
            return;
        }

        pd_logf("%ums APU: T5 -> %02X rtReady=%u mbox=%u err=%u cmdDone=%u cbChg=%u\r\n",
                st[0], (st[0] >> 7) & 1, (st[0] >> 3) & 1, (st[0] >> 2) & 1,
                (st[0] >> 1) & 1, st[0] & 1);

        if (st[0] & 0x08) {
            /* APU 邮箱中断: 读 4 字节消息（日志），然后继续看端口状态 */
            ami_phase = AMI_PH_MBOX;
            amu_next_poll_ms = 0;
            return;
        }
        if ((st[0] & 0x07) || (st[0] & 0x80)) {
            /* 端口状态/错误/Crossbar 变化 -> Type-3 读 */
            if (st[0] & 0x80) {
                link_cb_ready = true; /* 实时 Ready 同步镜像 */
            }
            ami_phase = AMI_PH_TYPE3;
            amu_next_poll_ms = 0;
            return;
        }
        /* 中断来自其他共享目标: 降频回到等待 */
        amu_next_poll_ms = millis() + AMI_INT_POLL_MS;
        ami_phase = AMI_PH_WAIT_INT;
        return;

    case AMI_PH_MBOX:
        /* Type-4: 读 APU 邮箱 4 字节（当前平台暂无 Mailbox 用例，仅日志透传） */
        r = int_read_type4(st);
        if (r != APU_I2C_OK) {
            if (step_timed_out()) {
                step_fail_retry("mbox", r == APU_I2C_NACK ? "nack" : "timeout");
            }
            return;
        }
        pd_logf("%ums APU: T4 mailbox -> %02X %02X %02X %02X\r\n",
                st[0], st[1], st[2], st[3]);
        /* 邮箱读完后回到等待，继续处理可能仍挂起的端口状态中断 */
        amu_next_poll_ms = millis() + AMI_INT_POLL_MS;
        ami_phase = AMI_PH_WAIT_INT;
        return;

    case AMI_PH_TYPE3:
        r = int_read_type3(st);
        if (r != APU_I2C_OK) {
            if (step_timed_out()) {
                step_fail_retry("type3", r == APU_I2C_NACK ? "nack" : "timeout");
            }
            return;
        }

        link_cb_ready = st_ready(st);
        stt = st_p0_status(st);
        if (stt == 0) {
            /* 命令未完成（理论少见）: 继续等 INT */
            if (step_timed_out()) {
                step_fail_retry("type3", "cmd still in progress");
                return;
            }
            amu_next_poll_ms = millis() + AMI_INT_POLL_MS;
            ami_phase = AMI_PH_WAIT_INT;
            return;
        }
        if (stt == 1) {
            /* 复位后首个 Type-3: 捕获并打印当前 Crossbar 状态 */
            if (!cb_mirror_valid) {
                uint8_t ctl = st_p0_ctl(st);
                uint8_t mode = (uint8_t)(ctl & 0x0F);
                uint8_t ori = (uint8_t)((ctl >> 4) & 1);

                pd_logf("%ums APU: post-RST CB state: P0st=%s ctl=0x%02X -> ORI=%s(%u) MODE=%s(%u) LP=%u\r\n",
                        millis(), port_status_name(stt), ctl,
                        ori_name(ori), ori,
                        cb_mode_name((apu_cb_mode_t)mode), mode, (ctl & 0x20) ? 1 : 0);
            }
            if (ami_step_safe) {
                amu_safe_done_ms = millis();
                amu_did_safe = true;
                cb_mirror_mode = APU_CB_SAFE; /* Crossbar 已进入 Safe */
                if (!amu_sync_safe) {
                    cb_mirror_ori = cb_req_ori;
                }
                cb_mirror_valid = true;
                cb_apply_hpd_rule(cb_mirror_mode); /* Safe 态强制清 HPD 请求 */
                pd_logf("%ums APU: CB SAFE complete (CC%u)%s\r\n", millis(), cb_req_cc,
                        amu_sync_safe ? " (post-RST sync)" : "");
                ami_step_safe = false;
                if (amu_sync_safe) {
                    amu_sync_safe = false;
                    /* 同步完成: 有用户请求则接其目标写，否则收工 */
                    ami_phase = cb_req_active ? AMI_PH_WRITE : AMI_PH_IDLE;
                    amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
                } else if (cb_req_mode == APU_CB_SAFE) {
                    /* 用户请求目标就是 Safe: 完成 */
                    ami_phase = AMI_PH_IDLE;
                    cb_req_active = false;
                } else {
                    ami_phase = AMI_PH_WRITE; /* 下一步: 目标写 */
                    amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;
                }
            } else {
                uint32_t dwell = millis() - amu_safe_done_ms;
                cb_mirror_mode = cb_req_mode;
                cb_mirror_ori = cb_req_ori;
                cb_mirror_valid = true;
                pd_logf("%ums APU: CB done: CC%u ORI=%s(%u) MODE=%s(%u) P0st=01b\r\n",
                        millis(), cb_req_cc ? cb_req_cc : 0,
                        ori_name(cb_req_ori), cb_req_ori,
                        cb_mode_name(cb_req_mode), cb_req_mode);
                if (amu_did_safe && dwell > APU_CB_SAFE_DWELL_LIMIT_MS) {
                    pd_logf("%ums APU: WARN safe dwell %ums > %ums\r\n", millis(), dwell,
                            APU_CB_SAFE_DWELL_LIMIT_MS);
                }
                amu_did_safe = false;
                cb_apply_hpd_rule(cb_mirror_mode);
                ami_phase = AMI_PH_IDLE;
                cb_req_active = false;
            }
        } else {
            step_fail_retry("type3", stt == 2 ? "cmd timeout" : "resv status");
        }
        return;

    case AMI_PH_BACKOFF:
        if ((int32_t)(millis() - amu_next_poll_ms) >= 0) {
            ami_phase = AMI_PH_SYNC;
        }
        return;

    default:
        ami_phase = AMI_PH_IDLE;
        return;
    }
}

#endif /* APU_AMU_POLLING_MODE */

/* ================= 任务与入口 ================= */

void apu_amu_task(void) {
    /* APU_RST# 状态沿管理 + 完整周期（assert -> release）后重新判定：
     * 1) RST# 有效: 挂起 I2C 总线 + 取消进行中的切换请求（复位后镜像失效）
     * 2) RST# 释放: 恢复总线 + 重新判定 + 重新同步 Crossbar 镜像 */
    static bool rst_prev = false;
    bool rst_now = apu_io_apu_rst_asserted();

    if (rst_now && !rst_prev) {
        apu_i2c_suspend();
        if (cb_req_active) {
            pd_logf("%ums APU: CB req aborted by RST#\r\n", millis());
            cb_req_active = false;
        }
        amu_phase = AMU_PH_IDLE;
        cb_mirror_valid = false;
        link_cb_ready = false;
        pd_logf("%ums APU: RST# assert -> i2c suspended\r\n", millis());
    } else if (!rst_now && rst_prev) {
        apu_i2c_resume();
        pd_logf("%ums APU: RST# release -> i2c resumed, re-detect\r\n", millis());
        detect_done = false;
        retry_after_ms = millis() + DETECT_RETRY_COOLDOWN_MS;
    }
    rst_prev = rst_now;

    /* RST# 有效期间总线挂起: 不做任何 I2C 事务（防止读重试的恢复逻辑
     * 复活外设），等释放后的 resume 重新同步 */
    if (rst_now) {
        return;
    }

    /* 模式判定（一次性，只读） */
    if (!detect_done && apu_i2c_target_ready() && millis() >= retry_after_ms) {
        detect_done = true;
        if (!detect_mode_once()) {
            if (++detect_retries <= DETECT_RETRY_MAX) {
                detect_done = false;
                retry_after_ms = millis() + DETECT_RETRY_COOLDOWN_MS;
            } else {
                detect_retries = 0;
                pd_logf("%ums APU: mode detect gave up (target not ready)\r\n", millis());
            }
        } else {
            detect_retries = 0;
#if APU_AMU_POLLING_MODE
            apu_amu_mode_t expect = APU_AMU_MODE_POLLING;
#else
            apu_amu_mode_t expect = APU_AMU_MODE_INTERRUPT;
#endif
            if (link_mode != APU_AMU_MODE_UNKNOWN && link_mode != expect &&
                !mode_mismatch_warned) {
                mode_mismatch_warned = true;
                pd_logf("%ums APU: WARN link mode %s != config %s, check APU_AMU_POLLING_MODE\r\n",
                        millis(),
                        link_mode == APU_AMU_MODE_POLLING ? "POLLING" : "INTERRUPT",
                        expect == APU_AMU_MODE_POLLING ? "POLLING" : "INTERRUPT");
            }
            pd_logf("%ums APU: === mode verdict: %s ===\r\n", millis(),
                    link_mode == APU_AMU_MODE_POLLING ? "POLLING"
                    : link_mode == APU_AMU_MODE_INTERRUPT ? "INTERRUPT" : "UNKNOWN");
        }
    }

    /* Crossbar 状态机驱动（需目标在线） */
    if (!apu_i2c_target_ready()) {
        return;
    }
#if APU_AMU_POLLING_MODE
    /* 复位/恢复后镜像无效: 先做首次状态同步（打印+检测，非 Safe 且可写则写 Safe） */
    if (!cb_mirror_valid && amu_phase == AMU_PH_IDLE) {
        amu_phase = AMU_PH_SYNC;
        amu_next_poll_ms = 0;
    }
    amu_sm_polling();
#else
    if (!cb_mirror_valid && ami_phase == AMI_PH_IDLE) {
        ami_phase = AMI_PH_SYNC;
        amu_next_poll_ms = 0;
    }
    amu_sm_interrupt();
#endif
}

void apu_amu_request_mode(uint8_t cc_active, apu_cb_mode_t mode, uint8_t orientation) {
    /* 镜像已知且与请求一致: 无需切换（不做冗余总线写） */
    if (cb_mirror_valid && mode == cb_mirror_mode && orientation == cb_mirror_ori) {
        pd_logf("%ums APU: CB req CC%u already ORI=%s(%u) MODE=%s(%u), skip\r\n",
                millis(), cc_active ? cc_active : 0,
                ori_name(orientation), orientation, cb_mode_name(mode), mode);
        return;
    }

    cb_req_cc = cc_active;
    cb_req_mode = mode;
    cb_req_ori = orientation;
    cb_req_active = true;
    amu_retries = 0;
    amu_deadline = millis() + APU_CB_CMD_TIMEOUT_MS;

    /* SAFE 步判定: 镜像未知（复位后）或当前非 Safe 且有实际变化时必须中转
     * （文档 4.1.3.6.2/4.2.3.6.2: 模式间切换必须经 Safe State） */
    {
        bool need_safe = !cb_mirror_valid ||
                         ((cb_mirror_mode != APU_CB_SAFE) &&
                          (mode != cb_mirror_mode || orientation != cb_mirror_ori));
#if APU_AMU_POLLING_MODE
        amu_step_safe = need_safe;
#else
        ami_step_safe = need_safe;
#endif
    }

#if APU_AMU_POLLING_MODE
    amu_phase = AMU_PH_CHECK;
    amu_next_poll_ms = 0; /* 立即开始写前状态检查 */
#else
    ami_phase = AMI_PH_SYNC;
#endif
    pd_logf("%ums APU: CB req CC%u ORI=%s(%u) MODE=%s(%u)%s\r\n", millis(),
            cc_active ? cc_active : 0, ori_name(orientation), orientation,
            cb_mode_name(mode), mode,
            (cb_mirror_valid && cb_mirror_mode != APU_CB_SAFE) ? " (via Safe)" : "");
}

bool apu_amu_cb_ready(void) {
    return link_cb_ready;
}

apu_cb_mode_t apu_amu_cb_mode(void) {
    return cb_mirror_mode;
}

uint8_t apu_amu_cb_orientation(void) {
    return cb_mirror_ori;
}

bool apu_amu_cb_busy(void) {
    return cb_req_active;
}

bool apu_amu_mirror_valid(void) {
    return cb_mirror_valid;
}

bool apu_amu_dp_path_active(void) {
    return cb_mirror_valid && !cb_req_active &&
           (cb_mirror_mode == APU_CB_DP_4LANE || cb_mirror_mode == APU_CB_USB3_DP_L01);
}

apu_amu_mode_t apu_amu_mode(void) {
    return link_mode;
}

void apu_amu_init(void) {
    link_mode = APU_AMU_MODE_UNKNOWN;
    detect_done = false;
    detect_retries = 0;
    retry_after_ms = 0;
    mode_mismatch_warned = false;

    cb_mirror_mode = APU_CB_SAFE;
    cb_mirror_ori = APU_ORIENT_NORMAL;
    cb_mirror_valid = false;
    link_cb_ready = false;
    cb_req_active = false;
    amu_phase = AMU_PH_IDLE;
}
