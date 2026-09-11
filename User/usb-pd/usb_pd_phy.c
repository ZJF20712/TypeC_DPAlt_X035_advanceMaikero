#include "usb_pd_phy.h"

#include "debug.h"
#include "millis.h"
#include "usb_pd_log.h"
#include "usb_pd_message.h"

/* 本设备固定角色：SRC（电源角色）+ DFP（数据角色） */
#define PD_DATA_ROLE_DFP    1
#define PD_POWER_ROLE_SRC   1

/* Discover Identity 专用 SVID */
#define USB_SID_PD 0xFF00

/* 发送重试次数（GoodCRC 超时后重发同一 MessageID） */
#define PD_TX_RETRY_MAX 3

/* 等待 GoodCRC 的轮询次数（约 1ms） */
#define PD_TX_GCRC_POLL_CNT 600

/* 同步发送完成等待超时 (ms)，超时强制重武装接收防主循环挂死 */
#define PD_TX_SYNC_TIMEOUT_MS 10

/* 收发 DMA 缓冲区（BMC_TX_SZ 最大 34：Header2 + 数据对象28 + CRC4） */
__attribute__((aligned(4))) static uint8_t usb_pd_rx_buffer[PD_MSG_MAX_LEN];
__attribute__((aligned(4))) static uint8_t usb_pd_tx_buffer[PD_MSG_MAX_LEN];

/* GoodCRC 应答缓冲（Header 2 字节） */
static uint8_t usb_pd_ack_buffer[2];

/* 当前使用的 PD 规范版本（初始 PD2.0 保证最大兼容，收到对端消息后跟随其版本） */
static uint8_t pd_spec_rev = DEF_PD_REVISION_20;

/* 发送 MessageID（0~7 循环） */
static uint8_t tx_msg_id = 0;

/* 中断内 GoodCRC 发送进行中标志（发送完成后由 IF_TX_END 清零） */
static volatile uint8_t ack_tx_active = 0;

/* 等待 ISR 内 GoodCRC 发送完成的超时 (ms) */
#define PD_TX_ACK_WAIT_MS 5

/**
 * @brief  配置为接收模式
 */
static void phy_set_rx_mode(void) {
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    USBPD->DMA = (uint32_t)(uint8_t *)usb_pd_rx_buffer;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
    NVIC_EnableIRQ(USBPD_IRQn);
}

/**
 * @brief  在已关闭中断的前提下，等待 ISR 内 GoodCRC 发送完成并收尾
 * @note   消除主循环发送与 ISR 应答发送的竞态（关中断前 ISR 恰好开始发送）
 */
static void phy_wait_ack_tx_done(void) {
    if (!ack_tx_active) {
        return;
    }

    /* 中断已关闭，标志只能在这里收尾（TX 必然完成，无需超时） */
    while (!(USBPD->STATUS & IF_TX_END)) {
    }
    USBPD->STATUS |= IF_TX_END;
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;
    ack_tx_active = 0;
}

/**
 * @brief  发送一包 BMC 数据
 * @param  sync 1:同步等待发送完成并切回接收模式；0:异步（由 IF_TX_END 中断收尾）
 * @param  pbuf 数据缓冲（需 4 字节对齐）
 * @param  len 字节数
 * @param  sop 前导码（UPD_SOP0 / UPD_HARD_RESET 等）
 */
static void phy_send_pack(uint8_t sync, uint8_t *pbuf, uint8_t len, uint8_t sop) {
    /* 打开发送通道对应的 CC 低电压驱动 */
    if (USBPD->CONFIG & CC_SEL) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->DMA = (uint32_t)(uint8_t *)pbuf;
    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    /* 清除 BMC 辅助信息（写 0 不影响 W1C 中断标志） */
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;

    if (sync) {
        /* 等待发送完成（加超时边界: CC 线状态突变等因素会让 BMC TX 迟迟
         * 不完成，无界等待会挂死主循环并饿死 I2C/日志任务） */
        {
            uint32_t tx_start = millis();
            while (!(USBPD->STATUS & IF_TX_END)) {
                if (millis() - tx_start > PD_TX_SYNC_TIMEOUT_MS) {
                    pd_logf("%ums PHY: BMC TX not completing, force re-arm\r\n", millis());
                    break;
                }
            }
        }
        USBPD->STATUS |= IF_TX_END;

        if (USBPD->CONFIG & CC_SEL) {
            USBPD->PORT_CC2 &= ~CC_LVE;
        } else {
            USBPD->PORT_CC1 &= ~CC_LVE;
        }

        /* 切回接收模式，准备接收 GoodCRC */
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= ~PD_ALL_CLR;
        USBPD->CONTROL &= ~PD_TX_EN;
        USBPD->DMA = (uint32_t)(uint8_t *)usb_pd_rx_buffer;
        USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
        USBPD->CONTROL |= BMC_START;
    }
}

/**
 * @brief  USB PD 中断处理：接收完成自动应答 GoodCRC
 */
void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD_IRQHandler(void) {
    uint8_t status = USBPD->STATUS;
    uint16_t byte_cnt = USBPD->BMC_BYTE_CNT;

    if (status & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        /* 收到硬复位：记录事件，由策略层处理（CC 连接不因此断开） */
        save_message(status, NULL, 0, PD_MSG_DIR_RX);
    }

    if (status & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;
        uint8_t need_rearm = 1;

        if ((status & MASK_PD_STAT) == PD_RX_SOP0 && byte_cnt >= 6) {
            /* 先存入消息缓冲区用于打印/策略处理 */
            save_message(status, usb_pd_rx_buffer, byte_cnt, PD_MSG_DIR_RX);

            /* GoodCRC 本身不再应答 */
            if ((byte_cnt != 6) || ((usb_pd_rx_buffer[0] & 0x1F) != DEF_TYPE_GOODCRC)) {
                Delay_Us(30); /* 保证帧间隔 */

                /* Header: GoodCRC + DFP + 对端规范版本；回显对端 MessageID + SRC 电源角色 */
                usb_pd_ack_buffer[0] = DEF_TYPE_GOODCRC | (PD_DATA_ROLE_DFP << 5) | ((usb_pd_rx_buffer[0] >> 6) << 6);
                usb_pd_ack_buffer[1] = (usb_pd_rx_buffer[1] & 0x0E) | PD_POWER_ROLE_SRC;

                /* 存档本次应答的 GoodCRC（保持日志序列完整） */
                save_message(PD_RX_SOP0, usb_pd_ack_buffer, 2, PD_MSG_DIR_TX);

                ack_tx_active = 1;
                /* 必须使能发送完成中断：否则 IF_TX_END 不触发中断，
                 * ack_tx_active 无法清零、RX 无法重新武装（对齐 WCH EVT） */
                USBPD->CONFIG |= IE_TX_END;
                need_rearm = 0; /* 应答发送完成后由 IF_TX_END 重新武装接收 */
                phy_send_pack(0, usb_pd_ack_buffer, 2, UPD_SOP0);
            }
        } else {
            /* 诊断：短帧 / 非 SOP0 帧（SOP'/SOP''）也存档，便于定位对端行为 */
            save_message(status, usb_pd_rx_buffer, byte_cnt, PD_MSG_DIR_RX);
        }

        if (need_rearm) {
            /* PHY 收完一帧后停止，必须重新武装才能接收下一帧 */
            phy_set_rx_mode();
        }
    }

    if (USBPD->STATUS & IF_TX_END) {
        USBPD->STATUS |= IF_TX_END;

        /* 中断内 GoodCRC 发送完成：清理 CC_LVE 并立即恢复接收（消息已存缓冲区） */
        if (ack_tx_active) {
            ack_tx_active = 0;
            USBPD->PORT_CC1 &= ~CC_LVE;
            USBPD->PORT_CC2 &= ~CC_LVE;
            phy_set_rx_mode();
        }
    }
}

/**
 * @brief  初始化 USB PD 物理层
 */
void usb_pd_phy_init(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    phy_set_rx_mode();
}

/**
 * @brief  发送 PD 消息（含 Header 构建、重试与 GoodCRC 等待）
 */
uint8_t usb_pd_phy_send_msg(uint8_t msg_type, const uint32_t *vdos, uint8_t num_vdo) {
    uint8_t retry;
    uint8_t i;
    uint16_t cnt;
    uint8_t gcrc_ok = 0;
    uint8_t saw_act = 0, last_cnt = 0, last_b0 = 0, last_b1 = 0;

    if (num_vdo > 7) {
        return PD_PHY_TX_FAIL;
    }

    /* Header 低字节：消息类型 + DFP 数据角色 + 规范版本 */
    usb_pd_tx_buffer[0] = (msg_type & 0x1F) | (PD_DATA_ROLE_DFP << 5) | (pd_spec_rev << 6);
    /* Header 高字节：SRC 电源角色 + MessageID + 数据对象数量 */
    usb_pd_tx_buffer[1] = PD_POWER_ROLE_SRC | ((tx_msg_id & 0x07) << 1) | ((num_vdo & 0x07) << 4);

    /* 数据对象（32 位小端逐字节） */
    for (i = 0; i < num_vdo; i++) {
        usb_pd_tx_buffer[2 + i * 4 + 0] = (uint8_t)(vdos[i]);
        usb_pd_tx_buffer[2 + i * 4 + 1] = (uint8_t)(vdos[i] >> 8);
        usb_pd_tx_buffer[2 + i * 4 + 2] = (uint8_t)(vdos[i] >> 16);
        usb_pd_tx_buffer[2 + i * 4 + 3] = (uint8_t)(vdos[i] >> 24);
    }

    /* 等待中断内可能的 GoodCRC 发送完成，避免碰撞 */
    {
        uint32_t start = millis();
        while (ack_tx_active) {
            if (millis() - start > PD_TX_ACK_WAIT_MS) {
                pd_logf("%ums PHY: ack_tx stuck\r\n", millis());
                return PD_PHY_TX_FAIL;
            }
        }
    }

    for (retry = 0; retry < PD_TX_RETRY_MAX && !gcrc_ok; retry++) {
        NVIC_DisableIRQ(USBPD_IRQn);

        /* 若关中断前 ISR 恰好开始应答发送，等待其完成避免踩寄存器 */
        phy_wait_ack_tx_done();

        /* 同步发送（含切回接收模式），随后轮询等待对端 GoodCRC */
        phy_send_pack(1, usb_pd_tx_buffer, 2 + num_vdo * 4, UPD_SOP0);

        cnt = PD_TX_GCRC_POLL_CNT;
        while (--cnt) {
            if (USBPD->STATUS & IF_RX_ACT) {
                USBPD->STATUS |= IF_RX_ACT;
                saw_act++;
                last_cnt = USBPD->BMC_BYTE_CNT;
                last_b0 = usb_pd_rx_buffer[0];
                last_b1 = usb_pd_rx_buffer[1];
                if (USBPD->BMC_BYTE_CNT == 6 && (usb_pd_rx_buffer[0] & 0x1F) == DEF_TYPE_GOODCRC &&
                    ((usb_pd_rx_buffer[1] >> 1) & 0x07) == tx_msg_id) {
                    /* 对端 GoodCRC 确认，消息发送成功：按线上顺序存档（TX 消息 + 对端 GoodCRC） */
                    save_message(PD_RX_SOP0, usb_pd_tx_buffer, 2 + num_vdo * 4, PD_MSG_DIR_TX);
                    save_message(PD_RX_SOP0, usb_pd_rx_buffer, 6, PD_MSG_DIR_RX);
                    gcrc_ok = 1;
                    break;
                } else {
                    /* 非 GoodCRC：存档后继续等待（不应出现的时序） */
                    save_message(PD_RX_SOP0, usb_pd_rx_buffer, USBPD->BMC_BYTE_CNT, PD_MSG_DIR_RX);
                }
            }
            Delay_Us(3);
        }

        NVIC_EnableIRQ(USBPD_IRQn);
    }

    /* 无论成败都重新武装接收（PHY 收完 GoodCRC 一帧后已停止） */
    NVIC_DisableIRQ(USBPD_IRQn);
    phy_set_rx_mode();
    NVIC_EnableIRQ(USBPD_IRQn);

    if (gcrc_ok) {
        tx_msg_id = (tx_msg_id + 1) & 0x07;
        return PD_PHY_TX_OK;
    }

    /* 诊断：记录发送失败时的 MessageID，便于定位对端不应答的阶段 */
    pd_logf("%ums PHY: TX fail msgid=%u type=%02X saw=%u cnt=%u b=%02X%02X\r\n",
            millis(), tx_msg_id, msg_type, saw_act, last_cnt, last_b1, last_b0);
    return PD_PHY_TX_FAIL;
}

/**
 * @brief  发送硬复位
 */
void usb_pd_phy_send_hard_reset(void) {
    NVIC_DisableIRQ(USBPD_IRQn);
    phy_wait_ack_tx_done();
    phy_send_pack(1, NULL, 0, UPD_HARD_RESET);
    phy_set_rx_mode();
    NVIC_EnableIRQ(USBPD_IRQn);
}

/**
 * @brief  向 SOP' 发送 Discover Identity 请求（线缆探测）
 * @note   不等待 GoodCRC（线缆可能无芯片应答，发送完成即返回）
 */
void usb_pd_phy_send_sop1_disc_ident(void) {
    static __attribute__((aligned(4))) uint8_t buf[6];

    /* VDM header: SVID 0xFF00 | SVDM | ver 2.0 | REQ | Discover Identity */
    uint32_t vdo = ((uint32_t)USB_SID_PD << 16) | (1u << 15) | (1u << 13) | (0u << 6) |
                   DEF_VDM_DISC_IDENT;

    buf[0] = (DEF_TYPE_VENDOR_DEFINED & 0x1F) | (PD_DATA_ROLE_DFP << 5) | (pd_spec_rev << 6);
    buf[1] = PD_POWER_ROLE_SRC | (0u << 1) | (1u << 4);
    buf[2] = (uint8_t)(vdo);
    buf[3] = (uint8_t)(vdo >> 8);
    buf[4] = (uint8_t)(vdo >> 16);
    buf[5] = (uint8_t)(vdo >> 24);

    NVIC_DisableIRQ(USBPD_IRQn);
    phy_wait_ack_tx_done();
    phy_send_pack(1, buf, 6, UPD_SOP1);
    phy_set_rx_mode();
    NVIC_EnableIRQ(USBPD_IRQn);
}

/**
 * @brief  设置 PD 规范版本
 */
void usb_pd_phy_set_spec_rev(uint8_t rev) {
    if (rev == DEF_PD_REVISION_10 || rev == DEF_PD_REVISION_20 || rev == DEF_PD_REVISION_30) {
        pd_spec_rev = rev;
    }
}

/**
 * @brief  获取 PD 规范版本
 */
uint8_t usb_pd_phy_get_spec_rev(void) {
    return pd_spec_rev;
}

/**
 * @brief  重置 MessageID 计数器
 */
void usb_pd_phy_reset_msgid(void) {
    NVIC_DisableIRQ(USBPD_IRQn);
    if (ack_tx_active) {
        /* ISR 应答发送在此收尾，其打断了 RX，需恢复接收模式 */
        phy_wait_ack_tx_done();
        phy_set_rx_mode();
    }
    tx_msg_id = 0;
    NVIC_EnableIRQ(USBPD_IRQn);
}
