# Jetson USB Camera

**自动识别输入格式的 ROS1 USB 摄像头驱动，支持 Jetson 硬件加速与软件后备。**

![ROS1](https://img.shields.io/badge/ROS-1-22314E?style=flat-square)
![C++11](https://img.shields.io/badge/C%2B%2B-11-00599C?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

自动探测摄像头格式，优先选择直接输出或硬件处理，并发布标准 ROS 图像与相机标定信息。分辨率、帧率由用户指定，无需编写输入格式配置文件。

[快速开始](#快速开始) · [启动方式](#启动方式) · [兼容格式](#兼容格式) · [相机标定](#相机标定) · [参数参考](#参数参考) · [常见问题](#常见问题)

## 功能

- **格式自动选择**：支持 MJPEG、常见原始格式、H.264/H.265 和 8 位 Bayer。
- **硬件优先**：优先使用 Jetson 加速；无法使用时尝试软件处理。
- **低开销发布**：NEON 像素打包、安全复用缓冲区，无订阅者时跳过图像打包。
- **断线恢复**：错误、流结束或持续无有效帧时切换候选管道或重新探测设备。
- **标准接口**：默认输出 `bgr8`，同时发布匹配时间戳的 `CameraInfo`。

## 快速开始

### 1. 准备环境

需要 Ubuntu、ROS1/catkin、GStreamer，以及 Linux V4L2 能识别的 USB 摄像头。Jetson 硬件插件由 JetPack 提供。

在已经加载 ROS 环境的终端中安装依赖。

**ROS 标定管理器：**

```bash
sudo apt install ros-${ROS_DISTRO}-camera-info-manager
```

**GStreamer 开发库：**

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

**GStreamer 运行插件：**

```bash
sudo apt install \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav
```

**摄像头查询工具：**

```bash
sudo apt install v4l-utils
```

### 2. 获取代码

将仓库放入 catkin 工作空间的 `src` 目录：

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

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch jetson_gst_camera camera.launch
```

默认使用 **`/dev/video0` · 1920 × 1080 · 60 FPS**。

| 输出话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 默认 `bgr8` 图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 标定信息，与图像使用相同时间戳 |

## 启动方式

### 指定设备、分辨率和帧率

可以直接传入 launch 参数，也可以修改现有 `launch/camera.launch` 中的默认值。

```bash
roslaunch jetson_gst_camera camera.launch \
  device:=/dev/video0 \
  width:=1280 \
  height:=720 \
  framerate:=30
```

> 输入格式自动选择，分辨率和帧率不会自动降低。请使用摄像头支持的组合。

### 使用 RGBA 输出

下游接受四通道图像时，可选择 `rgba8`：

```bash
roslaunch jetson_gst_camera camera.launch output_encoding:=rgba8
```

RGBA 比 BGR 多约 33% 的图像数据。如果下游仍需转换为 BGR，整体 CPU 开销未必更低。

<details>
<summary>分数帧率：例如 59.94 FPS</summary>

60000/1001 与精确的 60 FPS 是不同的模式，使用分子和分母分别指定：

```bash
roslaunch jetson_gst_camera camera.launch \
  framerate:=60000 \
  framerate_denominator:=1001
```

</details>

## 兼容格式

| 摄像头输入 | 处理方式 |
| --- | --- |
| MJPEG | 优先硬件 JPEG 解码，软件解码后备 |
| YUYV/YUY2、UYVY、YVYU、NV12、I420 | 硬件或软件颜色转换，无需视频解码 |
| BGR、BGRx、RGBA | 输出格式兼容时直接复制或打包 |
| RGB、RGBx、BGRA、YV12、NV21、NV16、NV24 | 根据可用插件转换 |
| GRAY8、GRAY16_LE/BE | 转换为 BGR/RGBA 图像 |
| H.264、H.265 | 对应解析器与硬件/软件解码器 |
| 8 位 Bayer：bggr、rggb、grbg、gbrg | 通过 `bayer2rgb` 转换 |

启动时会读取指定设备的能力和插件信息，尝试可用管道。只有收到有效帧后才确认该路径可用；失败时继续尝试其他候选，全部失败后按重连间隔重新探测。

**适用边界：**面向常见 Linux/UVC 摄像头。需要专有 SDK 的工业相机、特殊位深 Bayer、专用深度接口等可能需要独立驱动。16 位灰度转换为显示图像后不保留原始深度单位或 16 位精度。

## 相机标定

未指定标定文件时，节点发布未标定的 `CameraInfo`，其中 `K[0]=0`。

1. 使用实际摄像头和目标分辨率完成标定。
2. 用标定结果替换现有 `config/camera_info.yaml` 模板。
3. 在 `launch/camera.launch` 中启用对应参数：

```xml
<param name="camera_info_url"
       value="file://$(find jetson_gst_camera)/config/camera_info.yaml"/>
```

> 包内 YAML 的零内参是占位值，不能直接用于视觉定位或测量。换摄像头或分辨率后，应使用对应标定。

显式指定 URL 却无法加载标定时，节点启动失败。运行中若标定尺寸或数值不匹配，会告警并发布未标定信息，避免下游使用错误内参。

## 参数参考

前六项可直接作为 launch 参数传入；其余参数在现有 launch 的节点参数中修改。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device` | `/dev/video0` | 指定的 V4L2 设备 |
| `width` | `1920` | launch 默认图像宽度 |
| `height` | `1080` | launch 默认图像高度 |
| `framerate` | `60` | 帧率分子 |
| `framerate_denominator` | `1` | 帧率分母 |
| `output_encoding` | `bgr8` | `bgr8` 或 `rgba8` |
| `frame_id` | `camera` | 消息坐标系名称 |
| `image_topic` | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera_info` | 标定信息话题 |
| `camera_name` | `camera` | 标定管理器中的相机名称 |
| `camera_info_url` | 空 | 实际标定文件 URL |
| `reconnect_interval` | `2.0` | 重连间隔，秒 |
| `frame_timeout` | `3.0` | 当前管道无有效帧的超时，秒 |
| `max_frame_age` | `0.5` | 最大允许帧年龄，秒 |

<details>
<summary>性能与时间戳说明</summary>

- 硬件 BGR 路径将 BGRx→BGR 与 ROS 消息写入合并，Jetson ARM 使用 NEON；软件路径直接请求 BGR 输出。
- 仅当缓冲区不再被 ROS 或进程内订阅者引用时才复用，避免覆盖仍在使用的图像。
- 无图像订阅者时跳过打包；仍持续取样检查设备。appsink 和 ROS 发布队列均为 1。
- 软件后备、ROS 传输、图像查看器和下游转换仍会占用 CPU；不是端到端 GPU 零拷贝。
- 图像时间由 GStreamer PTS、segment 和管道时钟映射到 ROS 时间，精度依赖设备/驱动的源时间戳，不保证等于硬件曝光时间。
- 缺失、重复、倒序、过旧或损坏的帧会丢弃，不刷新健康计时。超时和重连使用单调时钟；ROS 时钟应与实时采集同步运行。

</details>

## 常见问题

### 摄像头不出图

**查看系统识别到的设备：**

```bash
v4l2-ctl --list-devices
```

**查看目标设备支持的分辨率、格式和帧率：**

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

**检查 ROS 图像发布频率：**

```bash
rostopic hz /camera/image_raw
```

若终端提示没有可用路径，先核对请求的分辨率/帧率，再检查提示中缺少的插件。自动选择可能需要等待某个候选超时，首次出图可能需要数秒。

### 打开 rqt 后 CPU 升高

查看器会增加图像传输、渲染或颜色转换开销。可降低分辨率/帧率、关闭不需要的查看器，或在下游支持时使用 RGBA。1080p60 BGR 原始数据量约为 373 MB/s。

### 多个摄像头的设备编号变化

可将 `device` 设为 `/dev/v4l/by-id/...` 稳定路径。节点只打开指定设备，不会自动切换到另一台摄像头。

## 许可证

本项目采用 [MIT License](LICENSE)。
