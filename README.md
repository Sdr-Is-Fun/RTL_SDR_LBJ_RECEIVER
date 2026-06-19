# RTL SDR_LBJ_RECEIVER

## 致谢和免责

本项目参考了 FLN1021/SX1276_Receive_LBJ。

本项目基于 GNU GPL 开源。发布或分发基于本项目的修改版本时，应遵循相同协议开源。

本项目按“原样（AS IS）”提供，不提供任何明示或暗示的担保，包括但不限于特定用途适用性及非侵权性的担保。

禁止用于非法用途。本项目仅作为学习 POCSAG 解码和 RTL-SDR 无线接收链路的示例。

在法律允许的最大范围内，项目作者或贡献者不对因使用本项目或无法使用本项目而导致的任何直接、间接、附带、特殊、惩罚性或后果性损害承担责任，包括但不限于数据丢失、法律责任等。

使用本项目即表示您已理解并同意自行承担所有使用风险。

## 开源说明

本项目基于 GNU GPL 开源。发布或分发基于本项目的修改版本时，应遵循相同协议开源。

建议发布时在仓库中同时包含 GPL 协议全文，例如 `LICENSE` 文件。

## 概述

SDR_LBJ_RECEIVER 是一个基于 RTL-SDR / rtl_tcp 的 POCSAG / LBJ 信号学习示例程序。程序使用固定 960 kS/s IQ 采样率，通过软件 DDC、信道滤波、FM 鉴频、DPLL 时钟恢复、BCH 校验/纠错和 LBJ 信息解析，实现列车相关报文显示。

本仓库代码面向学习和实验用途，适合用于理解 SDR 接收链路、POCSAG 解码流程和 RTL-SDR 数据流处理。

## 主要功能

- 固定 960 kS/s RTL-SDR 采样率
- 50 kHz 软件 DC 避让
- IQ 校正、DDC、抽取、信道滤波、FM 鉴频
- 导前码 AFC 估计与 DDC 微调
- RSSI 接收门控与保持释放
- DPLL 时钟恢复
- POCSAG 同步字检测
- BCH 校验与单比特纠错
- LBJ 报文解析与车次、方向、速度、公里标、机车、线路显示
- 多线路当前位置公里标设置
- 根据方向、速度和公里标估算到达时间
- 支持关键词过滤、错包提示、频谱显示、目标信号区域最强频率显示、运行时调频、增益、PPM、阈值调整

## 运行环境

推荐环境：

- Android / Termux + Python 3 + RTL-SDR 驱动
- Windows / Linux + Python 3 + RTL-SDR 驱动
- RTL-SDR 兼容接收设备

Python 依赖：

```bash
pip install numpy scipy
```

## RTL-SDR 驱动与连接方式

本程序通过 `rtl_tcp` 方式接收 RTL-SDR IQ 数据，默认连接地址为：

```text
127.0.0.1:1234
```

### Android 手机 / Termux

手机端需要安装 Android 版 RTL-SDR 驱动，然后在 Termux 中运行本程序。

推荐链接：

- [RTL-SDR Driver / SDR Driver（Google Play）](https://play.google.com/store/apps/details?id=marto.rtl_tcp_andro)
- [RTL-SDR Driver（F-Droid）](https://f-droid.org/en/packages/marto.rtl_tcp_andro/)
- [Android RTL-SDR Driver 源码](https://github.com/signalwareltd/rtl_tcp_andro-)

本程序在 Android / Termux 下会尝试通过 `iqsrc://` Intent 自动拉起 RTL-SDR Driver，并使用 `127.0.0.1:1234` 建立本机 TCP 连接。首次使用时请插入 RTL-SDR 设备，并在系统弹出的 USB 授权窗口中允许访问设备。

### Windows PC

Windows 下建议先安装 WinUSB 驱动，再启动 `rtl_tcp` 服务。

推荐链接：

- [RTL-SDR.com Quick Start Guide](https://www.rtl-sdr.com/rtl-sdr-quick-start-guide/)
- [Zadig WinUSB 驱动安装工具](https://zadig.akeo.ie/)
- [RTL-SDR Blog 驱动 / rtl_tcp Releases](https://github.com/rtlsdrblog/rtl-sdr-blog/releases)

### Linux PC

Linux 下可以使用发行版自带的 `rtl-sdr` 包，或从 Osmocom / GitHub 源码安装。

推荐链接：

- [Osmocom rtl-sdr 项目说明](https://osmocom.org/projects/rtl-sdr/wiki)
- [osmocom/rtl-sdr GitHub 镜像](https://github.com/osmocom/rtl-sdr)

Debian / Ubuntu 示例：

```bash
sudo apt update
sudo apt install rtl-sdr
```

## Android / Termux 安装流程

1. 安装 Termux。
2. 安装手机 RTL-SDR 驱动。
3. 打开 Termux，按顺序执行以下命令：

```bash
pkg upgrade
pkg install python3
pkg install python-numpy
pkg install python-scipy
termux-setup-storage
```

执行 `termux-setup-storage` 后，请在弹窗中允许 Termux 访问手机存储。

然后在手机共享存储中建立项目目录：

```bash
cd ~/storage/shared
mkdir sdr_lbj
```

将本项目 Python 主程序下载后放入手机存储中的 `sdr_lbj` 文件夹。建议将主程序文件名固定为：

```text
sdr_lbj.py
```

## 基本运行

### Windows / Linux

先启动 `rtl_tcp`：

```bash
rtl_tcp -a 127.0.0.1 -p 1234
```

然后另开一个终端运行本程序：

```bash
python3 sdr_lbj.py -f 821.2375 -g 15.7 -p 1
```

### Android / Termux

准备好 OTG 线，将 RTL-SDR 连接到手机，然后在 Termux 中执行：

```bash
cd ~/storage/shared/sdr_lbj
# FC0013 SDR 推荐执行以下命令，增益最大约为 15.7 dB
python3 sdr_lbj.py -f 821.2375 -g 15.7 -p 1
# 标准 RTL-SDR / R820T 推荐执行以下命令，增益设置为最大
python3 sdr_lbj.py -f 821.2375 -g 49.6 -p 1
```
执行后会弹窗询问是否运行驱动连接到 SDR 硬件，选择允许即可。

## 频谱与峰值频率显示

程序会在频谱显示中列出目标信号区域内强度最大的频率点，显示格式类似：

```text
PK:821.237500M Δ:+0.0k -35dB
```

其中：

- `PK` 表示目标信号区域内强度最大的频率点。
- `821.237500M` 表示换算后的实际 RF 频率。
- `Δ:+0.0k` 表示相对目标接收频率的偏移，单位 kHz。
- `-35dB` 表示该 FFT bin 的相对强度。

目标信号区域默认使用当前信道滤波带宽 `--bw`。当前默认信道带宽为 35.0 kHz，适合配合默认 AFC 最大修正范围 `--afc-max 8000` 使用。

## 启动参数

基本启动：

```bash
python3 sdr_lbj.py -f 821.2375
```

### 常用参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `-f, --freq` | `821.2375` | 接收频率，单位 MHz |
| `-g, --gain` | `15.7` | RTL-SDR 增益，单位 dB |
| `-p, --ppm` | `1` | RTL-SDR PPM 校正 |
| `--dc-offset` | `50` | DC 避让偏移，单位 kHz |
| `--bw` | `35.0` | 信道滤波带宽，单位 kHz |
| `--cs-threshold` | `-55` | RSSI 接收门控阈值，单位 dB |
| `--rssi-hold-ms` | `700` | RSSI 低于释放门限后的保持时间，单位 ms |
| `--afc-max` | `8000` | AFC 最大修正范围，单位 Hz |
| `--afc-gain` | `0.45` | AFC 环路增益 |
| `--my-km` | 无 | 默认当前位置公里标，单位 km |
| `--route-km` | 无 | 按线路设置当前位置公里标 |
| `--eta-max-min` | `360` | 最大显示到达时间，单位分钟 |

### 开关参数

| 参数 | 说明 |
| --- | --- |
| `--no-dc-offset` | 关闭 DC 避让 |
| `--no-rssi-gate` | 关闭 RSSI 接收门控 |
| `--afc-off` | 关闭 AFC |
| `--keep-afc-after-packet` | 接收结束后保留 AFC 补偿 |

### 多线路公里标

格式：

```bash
--route-km 线路名=xxxx.xKM
```

示例：

```bash
python3 sdr_lbj.py -f 821.2375 --route-km 京沪线=0123.4KM
```

多线路可以重复填写：

```bash
python3 sdr_lbj.py -f 821.2375 \
  --route-km 京沪线=0123.4KM \
  --route-km 沪昆线=0456.7KM
```

## 到达时间规则

方向规则固定为：

- 下行：公里标增大
- 上行：公里标减小

只有无误码或 BCH 可纠正后的可靠数据才参与线路提取和到达时间计算。不可纠正错误数据不会加入线路列表，也不会刷新 ETA。

ETA 仅作为学习和实验功能，受信号质量、报文内容、线路方向、速度变化和公里标设置影响，不应作为实际安全判断依据。

## 运行时按键

| 按键 | 功能 |
| --- | --- |
| `T` | 修改频率 |
| `G` | 修改增益 |
| `P` | 修改 PPM |
| `R` | 修改 RSSI 门控阈值 |
| `K` | 设置当前线路公里标 |
| `F` | 设置关注车次或机车 |
| `B` | 开关错包拦截 |
| `W` | 开关干扰预警 |
| `M` | 切换过滤模式 |
| `C` | 清屏 |
| `H` | 菜单 |
| `Q` | 退出 |

## 调参建议

| 现象 | 建议 |
| --- | --- |
| 一直显示 `RX:OFF` | 降低 `--cs-threshold` |
| 包中间断开 | 增大 `--rssi-hold-ms` |
| AFC 经常到最大值 | 降低 `--afc-gain`，或适当增大 `--afc-max` 并同步拓宽 `--bw` |
| DC 尖峰明显 | 增大 `--dc-offset` |
| 附近干扰多 | 减小 `--bw` |
| 频偏导致误码多 | 增大 `--bw` 或 `--afc-max` |

## AFC 与信道带宽建议

当前默认 AFC 最大修正范围为：

```text
--afc-max 8000
```

当前默认信道滤波带宽为：

```text
--bw 35.0
```

这组默认值适合频偏较大、温漂较明显的 RTL-SDR 接收场景。若现场邻频干扰较强，可以适当减小 `--bw`；若频偏仍然较大，可以在观察频谱和 `PK` 峰值频率显示后，再决定是否继续拓宽信道带宽。

不建议在干扰较强的环境下盲目增大 `--bw`，因为带宽越宽，进入解调链路的噪声和邻频干扰也会越多。
