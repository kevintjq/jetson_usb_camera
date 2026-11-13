# Jetson USB Camera

ROS1 USB 摄像头驱动。自动探测指定 V4L2 设备的输入格式，优先使用直通或 Jetson 硬件处理，必要时回退到软件处理。默认发布 `bgr8` 图像和 CameraInfo。

目标是覆盖常见 Linux/UVC 摄像头，不代表所有 USB 相机都已实测。需要厂商 SDK、专有传输协议或多传感器专用接口的设备，不一定能使用本驱动。

## 安装与启动

需要 Ubuntu、ROS1/catkin、GStreamer 和 V4L2 摄像头。Jetson 硬件插件由 JetPack 提供；没有硬件插件时可使用软件路径。

```bash
sudo apt install ros-$ROS_DISTRO-camera-info-manager \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-libav v4l-utils
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/jetson_usb_camera.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch jetson_gst_camera camera.launch
```

默认仍为 `/dev/video0`、1920×1080、60 FPS。分辨率和帧率由你手动选择，可以修改现有 launch，也可以直接传参：

```bash
roslaunch jetson_gst_camera camera.launch device:=/dev/video0 width:=1280 height:=720 framerate:=30
```

不新增运行配置文件，不自动降低分辨率或帧率。检查摄像头实际支持的组合：

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

若设备标称帧率为 60000/1001（约 59.94 FPS），设置 `framerate:=60000 framerate_denominator:=1001`，不会将它当作精确的 60 FPS。设备声明支持的组合仍可能受 USB 带宽、供电或系统负载影响，实际输出帧率需要实测。

## 自动兼容范围

| 输入 | 处理路径 |
| --- | --- |
| MJPEG | 优先 `jpegparse → nvv4l2decoder → nvvidconv`，也尝试 `nvjpegdec`；失败时回退 `jpegdec → videoconvert` |
| YUYV（GStreamer 名称 YUY2）、UYVY、YVYU、NV12、I420 等 | 不经过解码器，按插件能力尝试硬件或软件颜色转换 |
| BGR、BGRx、RGBA | 与目标输出兼容时直接取样/打包，避免额外转换 |
| RGB、RGBx、BGRA、YV12、NV21、NV16、NV24、GRAY8、GRAY16_LE/BE | 根据可用插件选择转换路径 |
| H.264 / H.265 | 对应解析器加硬件解码；也提供 `avdec_h264` / `avdec_h265` 软件后备 |
| 8 位 Bayer（bggr/rggb/grbg/gbrg） | `bayer2rgb → videoconvert`，需要该插件 |

灰度/16 位输入转换成 BGR/RGBA 显示图像，不提供原始深度单位、16 位精度输出或专用深度相机接口。其他位深 Bayer、厂商特殊格式及专有控制功能不在上述兼容范围内。

### 选择与重连

1. 在 READY 状态打开你指定的设备，读取实际 caps，筛选你请求的宽度、高度和帧率。
2. 检查所需插件，优先选择直接输出、硬件处理，再选择软件后备；不能只按设备名称猜格式。
3. 管道成功创建不代表可用，必须收到通过格式、尺寸、时间戳和映射检查的帧才确认。
4. 启动候选发生错误、EOS 或持续无有效帧时，尝试下一个候选。软件路径会明确打印 CPU 可能升高的警告。
5. 所有候选失败后，等待 `reconnect_interval` 再重新探测。已正常工作的设备断开后，也会重新读取能力，而不是永久沿用旧设备格式。

不存在支持你请求组合的可用路径时，会打印设备 caps 和缺失插件信息，不悄悄缩小图像或更改请求帧率。每个候选都可能花费一次 `frame_timeout` 才确认失败；首次自动选择/硬件失败回退可能需要数秒。

`device` 仍由你指定，不会扫描并替换成另一台摄像头。有多台设备时可使用稳定的 `/dev/v4l/by-id/...` 路径。

## CPU 与输出

默认接口保持 `/camera/image_raw`、`bgr8`。

- 硬件 BGR 路径去掉独立 `videoconvert`，BGRx→BGR 与 ROS 图像写入合并；Jetson ARM 使用 NEON。
- 软件路径直接请求 BGR 输出，不先生成 BGRx 再重复转换。摄像头原生 BGR 可直接复制。
- 缓冲区不再被 ROS/进程内订阅者引用时复用，避免不必要的逐帧分配。
- 无图像订阅者时跳过图像打包和发布，仍取样检查设备；CameraInfo 按自身订阅者独立发布。
- appsink 和 ROS 发布队列均为 1，优先最新数据；正确处理行填充和不对齐地址。

下游直接接受四通道时，可以使用：

```bash
roslaunch jetson_gst_camera camera.launch output_encoding:=rgba8
```

RGBA 字节数比 BGR 多约 33%；若下游又转回 BGR，整体 CPU 未必更低。软件后备也可能比硬件路径明显更耗 CPU。1080p60 BGR 原始数据约 373 MB/s，rqt 等查看器还会增加传输和显示开销；这不是端到端 GPU 零拷贝，未实测 CPU 降幅。

## 时间戳与标定

图像和 CameraInfo 使用同一时间戳。通过 buffer PTS、segment running-time、pipeline base-time 和 GStreamer clock 映射到 ROS 时间，不把解码后的处理时刻直接当作采集时刻。精度仍依赖设备/驱动源时间戳，不等于硬件曝光时间保证，也不替代相机/IMU 时间标定。

缺失、重复/倒序、过旧、明显超前、损坏或布局非法的帧会丢弃，不能刷新健康计时。默认最大帧年龄为 0.5 秒；持续无有效帧 3 秒后尝试其他路径或重连。时间戳不默默回退为当前时间。超时和重连使用单调时钟；ROS 时钟应与实时采集同步，任意倍率或暂停的模拟 `/clock` 未验证。

不提供 `camera_info_url` 时发布未标定 CameraInfo（`K[0]=0`）。现有 `config/camera_info.yaml` 只是零内参模板，使用前必须替换为真实标定结果，再启用 launch 中的路径。

明确指定 URL 却无法加载标定时，启动报错退出。运行中若内参数值或尺寸检查失败，会告警并发布未标定信息，不把旧摄像头或其他分辨率的内参套过来，也不猜测缩放、裁剪或 binning 后的内参。换摄像头应使用对应标定。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device` | `/dev/video0` | 用户指定的 V4L2 设备 |
| `width` / `height` | launch 为 `1920` / `1080` | 手动指定尺寸，不自动降级 |
| `framerate` | `60` | 帧率分子 |
| `framerate_denominator` | `1` | 帧率分母 |
| `output_encoding` | `bgr8` | `bgr8` 或 `rgba8` |
| `frame_id` | `camera` | 消息坐标系标签 |
| `image_topic` | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera_info` | 默认由图像话题推导 |
| `camera_name` | `camera` | 标定管理器名称 |
| `camera_info_url` | 空 | 实际标定文件 URL |
| `reconnect_interval` | `2.0` | 全部候选失败/断线后的重试间隔（秒） |
| `frame_timeout` | `3.0` | 当前候选无有效帧超时（秒） |
| `max_frame_age` | `0.5` | 采集至处理的最大允许延迟（秒） |

输入格式和加速路径自动选择，不需要额外的格式配置文件。非法参数类型、非正尺寸/帧率、非有限或非正超时、未知输出编码会导致启动失败。

## 验证范围

仓库不包含测试文件。开发时在仓库外验证候选策略、像素复制、尺寸/分数帧率 caps 匹配，并用本机 GStreamer 的合成视频检查了 MJPEG、原始格式和 8 位 Bayer 的软件/直通管道。

当前没有实体摄像头、ROS/GStreamer 开发库或 Jetson。完整节点编译、真实 V4L2 枚举、USB 拔插、H.264/H.265 软件解码、Jetson 硬件/NEON 路径和实际 CPU 占用仍需目标环境验证，不能据此保证所有型号可用。

参考：[GStreamer v4l2src](https://gstreamer.freedesktop.org/documentation/video4linux2/v4l2src.html)、[GStreamer 时钟](https://gstreamer.freedesktop.org/documentation/application-development/advanced/clocks.html)、[NVIDIA Jetson 加速 GStreamer](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)。

## 许可证

MIT，见 [LICENSE](LICENSE)。
