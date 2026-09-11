#include "usb_pd_message.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "millis.h"
#include "usb_pd_log.h"
#include "usb_vbus_measure.h"

static pd_msg_buffer_t msg_buffer = {0};

static struct {
    uint32_t msg_counter; // 消息计数器
} pdMessage = {0};

/* 控制消息类型描述表 */
static const pd_msg_type_desc_t ctrl_msg_desc[] = {
    {0b00001, "GoodCRC"},
    {0b00010, "GotoMin"},
    {0b00011, "Accept"},
    {0b00100, "Reject"},
    {0b00101, "Ping"},
    {0b00110, "PSRDY"},
    {0b00111, "GetSourceCap"},
    {0b01000, "GetSinkCap"},
    {0b01001, "DRSwap"},
    {0b01010, "PRSwap"},
    {0b01011, "VconnSwap"},
    {0b01100, "Wait"},
    {0b01101, "SoftReset"},
    {0b01110, "DataReset"},
    {0b01111, "DataResetComplete"},
    {0b10000, "NotSupported"},
    {0b10001, "GetSourceCapExt"},
    {0b10010, "GetStatus"},
    {0b10011, "FRSwap"},
    {0b10100, "GetPPSStatus"},
    {0b10101, "GetCountryCodes"},
    {0b10110, "GetSinkCapExt"},
    {0b10111, "GetSourceInfo"},
    {0b11000, "GetRevision"},
    {0, NULL},
};

/* 数据消息类型描述表 */
static const pd_msg_type_desc_t data_msg_desc[] = {
    {0b00001, "SourceCap"},
    {0b00010, "Request"},
    {0b00011, "BIST"},
    {0b00100, "SinkCap"},
    {0b00101, "BatteryStatus"},
    {0b00110, "Alert"},
    {0b00111, "GetCountryInfo"},
    {0b01000, "EnterUSB"},
    {0b01001, "EPRRequest"},
    {0b01010, "EPRMode"},
    {0b01011, "SourceInfo"},
    {0b01100, "Revision"},
    {0b01111, "VendorDefined"},
    {0, NULL},
};

/* 扩展消息类型描述表 */
static const pd_msg_type_desc_t ext_msg_desc[] = {
    {0x01, "SourceCapExt"},
    {0x02, "Status"},
    {0x03, "GetBatteryCap"},
    {0x04, "GetBatteryStatus"},
    {0x05, "BatteryCap"},
    {0x06, "GetMfrInfo"},
    {0x07, "MfrInfo"},
    {0x08, "SecurityReq"},
    {0x09, "SecurityResp"},
    {0x0A, "FWUpdateReq"},
    {0x0B, "FWUpdateResp"},
    {0x0C, "PPSStatus"},
    {0x0D, "CountryInfo"},
    {0x0E, "CountryCodes"},
    {0x0F, "SinkCapExt"},
    {0x10, "ExtControl"},
    {0x11, "EPRSourceCap"},
    {0x12, "EPRSinkCap"},
    {0x1F, "VendorDefinedExt"},
    {0, NULL},
};

/**
 * @brief  在消息类型描述表中查找消息类型
 * @param  type 消息类型编号
 * @param  desc_table 描述表
 * @param  prefix 消息类型前缀（Ctrl/Data/Ext）
 * @return const char* 消息类型名称
 */
static const char *find_msg_type_name(uint8_t type, const pd_msg_type_desc_t *desc_table, const char *prefix) {
    static char msg_type_buf[32];

    for (int i = 0; desc_table[i].name != NULL; i++) {
        if (desc_table[i].type == type) {
            // snprintf(msg_type_buf, sizeof(msg_type_buf), "[%02d]%s", type, desc_table[i].name);
            snprintf(msg_type_buf, sizeof(msg_type_buf), "%s", desc_table[i].name);
            return msg_type_buf;
        }
    }

    // snprintf(msg_type_buf, sizeof(msg_type_buf), "[%02d]Unknown_%s", type, prefix);
    snprintf(msg_type_buf, sizeof(msg_type_buf), "Unknown_%s", prefix);

    return msg_type_buf;
}

/**
 * @brief  获取消息类型名称
 * @param  header 消息头指针
 * @return const char* 消息类型名称
 */
static const char *get_message_type_name(pd_msg_header_t *header) {
    if (header->NumberOfDataObjects == 0) {
        return find_msg_type_name(header->MessageType, ctrl_msg_desc, "Ctrl");
    } else {
        return find_msg_type_name(header->MessageType, data_msg_desc, "Data");
    }
}

/**
 * @brief  获取 SOP 类型名称
 * @param  status USBPD->STATUS 寄存器值
 * @return const char* SOP 类型名称
 */
static const char *get_sop_type_name(uint32_t status) {
    uint32_t sop = status & MASK_PD_STAT;
    switch (sop) {
    case PD_RX_SOP0:
        return "SOP";
    case PD_RX_SOP1_HRST:
        return "SOP'";
    case PD_RX_SOP2_CRST:
        return "SOP''";
    default:
        return "???";
    }
}

/**
 * @brief  获取消息方向字符串
 * @param  status STATUS 寄存器值
 * @param  header 消息头指针
 * @return const char* 方向字符串
 */
static const char *get_direction_str(uint32_t status, pd_msg_header_t *header) {
    if (header->PortPowerRole) {
        if ((status & MASK_PD_STAT) == PD_RX_SOP1_HRST || (status & MASK_PD_STAT) == PD_RX_SOP2_CRST) {
            return "CAB→ ? ";
        }
        return "SRC→SNK";
    } else {
        if ((status & MASK_PD_STAT) == PD_RX_SOP1_HRST || (status & MASK_PD_STAT) == PD_RX_SOP2_CRST) {
            return "CAB← ? ";
        }
        return "SRC←SNK";
    }
}

/**
 * @brief  打印 PD MSG
 * @param  msg 消息指针
 */
void print_message(pd_msg_t *msg) {
    // 时间，电压，序号，SOP，原始数据
    // pd_logf("%u,%u,%u,%u,", msg->timestamp_ms, adc_raw_to_vbus_mv(msg->vbus_raw), msg->msg_id, msg->status & MASK_PD_STAT);
    // for (uint8_t i = 0; i < msg->len; i++) {
    //     pd_logf("%02X", msg->data[i]);
    // }
    // pd_logf("\n");

    pd_msg_header_t header_tmp;
    memcpy(&header_tmp, msg->data, sizeof(pd_msg_header_t));
    pd_msg_header_t *header = &header_tmp;

    bool is_goodcrc = (header->NumberOfDataObjects == 0 && header->MessageType == CTRL_GOODCRC);
    static bool last_msg_was_goodcrc = true;

    // 计算数据部分的长度（不包括 CRC32）
    uint8_t data_len = 2 + (header->NumberOfDataObjects * 4); // 头部 2 字节 + 数据对象长度

    // 时间 电压 序号 SOP
    pd_logf("%ums %05umV #%03u %-5s ", msg->timestamp_ms, adc_raw_to_vbus_mv(msg->vbus_raw), msg->msg_id, get_sop_type_name(msg->status));

    if (msg->status & IF_RX_RESET) {
        pd_logf("RX_RESET\r\n");
        return;
    }

    // 消息类型
    if (header->Extended) {
        pd_logf("%-15s ", find_msg_type_name(header->MessageType, ext_msg_desc, "Ext"));
    } else {
        pd_logf("%-15s ", get_message_type_name(header));
    }

    // 消息 ID
    pd_logf("%u ", header->MessageID);

    // 方向
    pd_logf("%s ", get_direction_str(msg->status, header));

    // 版本
    pd_logf("V%u ", header->SpecificationRevision + 1);

    // 数据对象数量
    // pd_logf("%-1u ", header->NumberOfDataObjects);

    // 消息头
    pd_logf("[H]0x%02X%02X", msg->data[1], msg->data[0]);

    // 数据对象
    if (msg->len > 2) {
        for (uint8_t i = 0; i < header->NumberOfDataObjects; i++) {
            pd_logf("[%u]0x%02X%02X%02X%02X",
                           i,
                           msg->data[2 + i * 4 + 3],
                           msg->data[2 + i * 4 + 2],
                           msg->data[2 + i * 4 + 1],
                           msg->data[2 + i * 4]);
        }
    }

    // CRC32
    if (msg->len >= data_len + 4) {
        pd_logf("[CRC]0x%02X%02X%02X%02X",
                       msg->data[data_len + 3],
                       msg->data[data_len + 2],
                       msg->data[data_len + 1],
                       msg->data[data_len]);
    }

    // 检查连续的消息类型
    if (is_goodcrc && last_msg_was_goodcrc) {
        pd_logf(" ←WARN!!");
    }
    if (!is_goodcrc && !last_msg_was_goodcrc && msg->msg_id != 1) {
        pd_logf(" ←WARN!!");
    }

    pd_logf("\r\n");

    // 更新状态
    last_msg_was_goodcrc = is_goodcrc;
}

/**
 * @brief  将消息保存到循环缓冲区

 * @param  status STATUS 寄存器值
 * @param  data 消息数据
 * @param  len 消息长度
 * @param  dir 消息方向
 */
void save_message(uint32_t status, uint8_t *data, uint8_t len, pd_msg_dir_t dir) {
    // 当前索引
    uint16_t curr_idx = msg_buffer.write_idx;
    // 下一个写入位置
    uint16_t next_idx = (curr_idx + 1) % PD_MSG_BUFFER_SIZE;

    // 缓冲区满时覆盖最旧消息
    if (next_idx == msg_buffer.read_idx) {
        msg_buffer.read_idx = (msg_buffer.read_idx + 1) % PD_MSG_BUFFER_SIZE;
    }

    // 保存
    if (data != NULL && len > 0) {
        memcpy(msg_buffer.msgs[curr_idx].data, data, len);
    }
    msg_buffer.msgs[curr_idx].len = len;
    msg_buffer.msgs[curr_idx].status = status;
    msg_buffer.msgs[curr_idx].dir = dir;
    msg_buffer.msgs[curr_idx].msg_id = ++pdMessage.msg_counter;
    msg_buffer.msgs[curr_idx].timestamp_ms = millis();
    msg_buffer.msgs[curr_idx].vbus_raw = adc_get_avg_raw();

    // 更新写指针
    msg_buffer.write_idx = next_idx;
}

/**
 * @brief  获取消息缓冲区
 * @return pd_msg_buffer_t* 消息缓冲区指针
 */
pd_msg_buffer_t *get_message_buffer(void) {
    return &msg_buffer;
}

/**
 * @brief  重置消息计数器
 */
void reset_message_counter(void) {
    pdMessage.msg_counter = 0;
}
