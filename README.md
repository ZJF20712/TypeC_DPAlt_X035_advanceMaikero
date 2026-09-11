# usb-pd-dp-amd

基于 CH32X035 的 USB-C PD Source + DisplayPort Alt Mode 适配器固件。

## 硬件

原硬件参考（本工程由其修改而来）：https://oshwhub.com/azunya/usb-pd-sniffer_v2

## 功能

- **USB PD Source 角色**（SRC / DFP）
  - CC 线 Rp-3.0A 上拉（USBPD 外设内部 330uA 恒流源），Rd attach 检测与活动 CC 选择
  - PD 2.0/3.0 收发（BMC PHY，中断内自动应答 GoodCRC，消息重试）
  - 广播 Source Capabilities：5V/3A 固定 PDO
  - 完整协商流程：SourceCap → Request → Accept → PS_RDY
  - 处理 Get_Source_Cap / Soft_Reset 等，Reject 各类 Swap
- **DisplayPort Alt Mode（VDM 握手）**
  - 主动流程：Discover Identity → Discover SVIDs → Discover Modes → Enter Mode → DP Status Update → DP Configure
  - Pin assignment C/D/E（4 lane DP），按 C > E > D 优先级与对端协商
  - 应答对端 Discover Identity / SVIDs / Modes / Enter / Exit / Attention
  - HPD 状态跟踪与打印
- USB CDC 调试日志（所有 PD 消息按帧打印，含 GoodCRC）

## 状态指示（WS2812）

| 颜色 | 含义 |
| ---- | ---- |
| 红色 | 未连接 |
| 蓝色 | CC1 连接 |
| 绿色 | CC2 连接 |
| 紫色 | PD 合同建立（5V3A） |
| 黄色 | DP Alt Mode 协商成功 |
| 红色（连接后） | DP Alt Mode 协商失败 |

## 工程结构

```
User/
├── main.c
└── usb-pd/
    ├── usb_pd_cc.c/h        # SRC CC 检测（Rp 上拉 / Rd attach）
    ├── usb_pd_phy.c/h       # PD PHY（BMC 收发、GoodCRC、重试）
    ├── usb_pd_policy.c/h    # SRC 策略机 + VDM DP Alt Mode 握手
    ├── usb_pd_monitor.c/h   # 整合层（CC + 策略 + 消息循环）
    └── usb_pd_message.c/h   # 消息缓冲与打印
```

MounRiver Studio 2 打开工程后直接编译，WCH-Link 下载。
