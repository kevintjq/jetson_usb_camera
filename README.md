<h1 align="center">Jetson USB Camera</h1>

<p align="center">自动识别输入格式的 ROS1 USB 摄像头驱动</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-1-22314E?style=flat-square" alt="ROS1">
  <img src="https://img.shields.io/badge/C%2B%2B-11-00599C?style=flat-square" alt="C++11">
  <img src="https://img.shields.io/badge/Acceleration-Jetson-76B900?style=flat-square" alt="Jetson acceleration">
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="MIT License">
</p>

<p align="center">
  <a href="#快速开始">快速开始</a> ·
  <a href="#使用方式">使用方式</a> ·
  <a href="#兼容格式">兼容格式</a> ·
  <a href="#相机标定">相机标定</a> ·
  <a href="#参数参考">参数参考</a> ·
  <a href="#常见问题">常见问题</a>
</p>

---

自动探测指定 V4L2 摄像头的格式，优先选择直接输出或 Jetson 硬件处理，必要时使用软件后备。默认发布标准 `bgr8` 图像和对应的 `CameraInfo`，分辨率与帧率由用户指定。

| 能力 | 说明 |
| --- | --- |
| 自动选择 | MJPEG、常见原始格式、H.264/H.265、8 位 Bayer |
| 硬件加速 | Jetson 解码与颜色转换；无硬件插件时尝试软件路径 |
| 减少 CPU 开销 | NEON 像素打包、缓冲区安全复用、按订阅需求发布 |
| 自动恢复 | 错误、流结束或持续无有效帧时切换候选或重新探测 |

## 快速开始

需要 **Ubuntu、ROS1/catkin、GStreamer** 和 Linux V4L2 能识别的摄像头。Jetson 加速插件由 JetPack 提供；无需额外编写输入格式配置文件。

### 1. 安装依赖

<details>
<summary>首次使用：展开安装命令</summary>

在已经加载 ROS 环境的终端中执行。

**ROS 标定管理器：**

```bash
sudo apt install ros-${ROS_DISTRO}-camera-info-manager
```

**GStreamer 开发库：**

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

**解码与转换插件：**

```bash
sudo apt install \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav
```

**设备查询工具：**

```bash
sudo apt install v4l-utils
```

</details>

### 2. 获取代码

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/jetson_usb_camera.git
```

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make
```

### 4. 启动

先加载工作空间环境：

```bash
source ~/catkin_ws/devel/setup.bash
```

仓库目录名为 `jetson_usb_camera`，ROS 包名为 `jetson_gst_camera`；`roslaunch` 使用的是包名。若提示找不到包，请在当前终端重新加载上述工作空间环境，并检查：

```bash
rospack find jetson_gst_camera
```

启动摄像头：

```bash
roslaunch jetson_gst_camera camera.launch
```

> 默认设备：`/dev/video0`　｜　采集模式：**1920 × 1080 · 60 FPS**　｜　图像编码：`bgr8`

## 使用方式

### 指定采集模式

可以传入 launch 参数，或修改 [camera.launch](launch/camera.launch) 中的默认值。

```bash
roslaunch jetson_gst_camera camera.launch \
  device:=/dev/video0 \
  width:=1280 \
  height:=720 \
  framerate:=30
```

输入格式自动选择，**分辨率和帧率不会自动降低**。指定组合不受支持时，终端会提示设备能力或缺失插件。

### 查看输出

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 默认 BGR 图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 与图像时间戳一致的标定信息 |

检查图像发布频率：

```bash
rostopic hz /camera/image_raw
```

<details>
<summary>更多启动示例：RGBA 与分数帧率</summary>

**RGBA：**下游直接接受四通道图像时，可跳过 BGR 打包。

```bash
roslaunch jetson_gst_camera camera.launch output_encoding:=rgba8
```

RGBA 数据量比 BGR 多约 33%；下游若再转回 BGR，整体 CPU 开销未必更低。

**分数帧率：**60000/1001（约 59.94 FPS）与精确的 60 FPS 不同。

```bash
roslaunch jetson_gst_camera camera.launch \
  framerate:=60000 \
  framerate_denominator:=1001
```

以上示例仍需配合摄像头支持的宽度和高度。

</details>

## 兼容格式

| 摄像头输入 | 处理方式 |
| --- | --- |
| MJPEG | 优先硬件 JPEG 解码，软件解码后备 |
| YUYV/YUY2、UYVY、YVYU、NV12、I420 | 硬件或软件颜色转换 |
| BGR、BGRx、RGBA | 格式兼容时直接复制或打包 |
| RGB、RGBx、BGRA、YV12、NV21、NV16、NV24 | 根据插件能力转换 |
| GRAY8、GRAY16_LE/BE | 转换为 BGR/RGBA 显示图像 |
| H.264、H.265 | 对应解析器及硬件/软件解码器 |
| 8 位 Bayer：bggr、rggb、grbg、gbrg | 通过 `bayer2rgb` 转换 |

管道必须收到有效帧才确认可用。启动失败、EOS 或持续无有效帧时，节点会尝试其他候选；全部失败后按重连间隔重新探测。首次选择可能需要数秒，软件后备会明确提示 CPU 开销可能增加。

面向常见 Linux/UVC 设备，不保证支持需要专有 SDK 的工业相机、特殊位深 Bayer 或专用深度接口。16 位灰度转为显示图像后，不保留原始深度单位或 16 位精度。

## 相机标定

1. 用实际摄像头和目标分辨率完成标定。
2. 用结果替换 [camera_info.yaml](config/camera_info.yaml) 模板。
3. 在现有 launch 中启用以下参数：

```xml
<param name="camera_info_url"
       value="file://$(find jetson_gst_camera)/config/camera_info.yaml"/>
```

> 包内零内参是占位值。换摄像头或分辨率后，应使用对应标定。

未指定标定文件时，发布未标定信息（`K[0]=0`）。显式指定 URL 却无法加载标定时，启动失败；运行中若内参数值或尺寸不匹配，会告警并发布未标定信息。

## 参数参考

设备、宽高、帧率分子/分母和输出编码可直接作为 launch 参数传入；其余参数在现有 launch 的节点参数中修改。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device` | `/dev/video0` | 指定的 V4L2 设备 |
| `width` / `height` | `1920` / `1080` | launch 默认尺寸，分别可传参 |
| `framerate` | `60` | 帧率分子 |
| `framerate_denominator` | `1` | 帧率分母 |
| `output_encoding` | `bgr8` | `bgr8` 或 `rgba8` |
| `frame_id` | `camera` | 消息坐标系名称 |
| `image_topic` | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera_info` | 标定信息话题 |
| `camera_name` | `camera` | 标定管理器名称 |
| `camera_info_url` | 空 | 实际标定文件 URL |
| `reconnect_interval` | `2.0` | 重连间隔，秒 |
| `frame_timeout` | `3.0` | 当前候选无有效帧超时，秒 |
| `max_frame_age` | `0.5` | 最大允许帧年龄，秒 |

<details>
<summary>实现说明：性能、时钟与数据过滤</summary>

- 硬件 BGR 路径将 BGRx→BGR 与 ROS 消息写入合并，Jetson ARM 使用 NEON；软件路径直接请求 BGR。
- 仅当缓冲区不再被 ROS/进程内订阅者引用时才复用。无图像订阅者时跳过打包，但仍取样检查设备。
- appsink 和 ROS 发布队列均为 1。软件处理、ROS 传输和显示仍会消耗 CPU，不是端到端 GPU 零拷贝。
- 图像时间由 PTS、segment 和管道时钟映射到 ROS 时间；精度依赖驱动源时间戳，不保证等于硬件曝光时刻。
- 缺失、重复、倒序、过旧或损坏的帧被丢弃，不刷新健康计时。超时与重连使用单调时钟；ROS 时钟应与实时采集同步运行。

</details>

## 常见问题

### 摄像头没有图像

查看系统识别到的设备：

```bash
v4l2-ctl --list-devices
```

单独查看目标设备的可用模式：

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

核对设备路径、分辨率、帧率和终端提示的插件。设备声明支持的模式仍可能受 USB 带宽、供电或系统负载影响。

### 提示设备忙或无法打开设备

先关闭使用同一摄像头的其他驱动节点、视频软件或采集程序，再重新启动。若有多台摄像头，结合 `v4l2-ctl --list-devices` 确认目标设备，避免重复打开同一个视频接口。

### 打开 rqt 后 CPU 升高

查看器会增加传输、渲染或颜色转换开销。可降低采集分辨率/帧率、关闭不需要的查看器，或在下游支持时选择 RGBA。1080p60 BGR 原始数据量约为 373 MB/s。

### 多台摄像头的设备编号变化

可将 `device` 设为 `/dev/v4l/by-id/...` 稳定路径。节点只打开指定设备，不会自动改用另一台摄像头。

## 许可证

本项目采用 [MIT License](LICENSE)。
