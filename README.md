# Jetson USB Camera

ROS1 USB 摄像头驱动。使用 Jetson GStreamer 插件解码 MJPEG，并发布 `sensor_msgs/Image` 和 `CameraInfo`。

## 编译与启动

需要 Ubuntu、ROS1/catkin、JetPack 提供的 `nvv4l2decoder` / `nvvidconv` 和支持目标分辨率/帧率的 MJPEG USB 摄像头。

```bash
sudo apt install ros-$ROS_DISTRO-camera-info-manager \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev v4l-utils
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/jetson_usb_camera.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch jetson_gst_camera camera.launch
```

默认 launch 配置为 `/dev/video0`、1920×1080、60 FPS，输出 `/camera/image_raw` 和 `/camera/camera_info`。先确认设备支持所选组合：

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

## CPU 与内存优化

默认仍输出 `bgr8`，保持现有 OpenCV 接口：

```text
v4l2src → jpegparse → nvv4l2decoder → nvvidconv → BGRx → 直接打包到 ROS BGR 图像
```

移除独立 `videoconvert` 和中间 BGR 图像。BGRx→BGR 与写入 ROS 消息合并，Jetson ARM 上使用 NEON SIMD，其他平台保留标量实现。这样减少一次独立转换后的整帧复制；缓冲区不再被 ROS 或进程内订阅者引用时复用，避免不必要的逐帧分配和初始化。处理行步长和非 16 像素倍数的尾部，不能假定图像紧密排列。

没有图像订阅者时，跳过图像打包、消息分配和发布；仍持续取样验证流，以保留设备健康检查和快速接入。CameraInfo 按自身订阅者数量独立发布。ROS 发布队列和 appsink 队列均为 1，优先最新数据。

如果下游直接接受四通道图像，可进一步跳过 BGR 打包：

```bash
roslaunch jetson_gst_camera camera.launch output_encoding:=rgba8
```

此模式由 `nvvidconv` 输出 RGBA，直接复制到 ROS。它的图像字节数比 BGR 多约 33%；若下游又转回 BGR，整体 CPU 未必更低。`bgr8` 仍需要一次像素打包，`rgba8` 仍需要复制与 ROS 传输，均不是端到端 GPU 零拷贝。

硬件解码和 `nvvidconv` 的可用能力取决于 JetPack。1080p60 的 BGR 原始数据约 373 MB/s；打开 rqt 等查看器仍会增加传输、渲染或下游颜色转换开销。当前未在 Jetson 上测量优化前后 CPU，不能给出降幅百分比。

## 时间戳与健康检查

图像和 CameraInfo 使用同一时间戳。图像时间通过 buffer PTS、sample segment 的 running-time、pipeline base-time 和 GStreamer clock 映射到 ROS 时间；不会把解码后处理时刻直接当成采集时刻。时间含义仍取决于 V4L2 驱动提供的源时间戳，不能保证等于硬件曝光时刻，也不能替代相机/IMU 时间标定。

缺失、重复/倒序、过旧或明显超前的时间戳会丢弃，默认最大帧年龄为 0.5 秒；不默默回退为当前时间。ROS 时间必须已经有效，映射面向与实时采集同步运行的 ROS 时钟，未验证任意倍率/暂停的模拟 `/clock`。微小未来偏差最多容忍 10 ms，按零年龄处理。

只有格式、时间戳和映射布局检查通过的帧才刷新健康计时。持续收到格式错误或无法映射的 sample，也会在 `frame_timeout` 后重连。GStreamer ERROR/EOS 同样触发重连。超时和重连间隔使用单调时钟，等待重连期间仍处理 ROS 回调。持续时间倒退会导致丢帧并最终通过超时重建管道。

## CameraInfo

不提供 `camera_info_url` 时，发布尺寸与图像一致的未标定 CameraInfo（`K[0]=0`）。包内 `config/camera_info.yaml` 是零内参占位模板，必须用真实标定替换后才能启用 launch 中的路径。

显式指定标定 URL 却无法加载有效标定时，节点启动报错退出。运行时检查标定尺寸、矩阵数值和焦距：与实际图像不匹配或明显无效时，告警并发布未标定信息，避免把错误内参交给下游。不自动猜测缩放、裁剪或 binning 后的内参。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device` | `/dev/video0` | 摄像头设备 |
| `width` / `height` | launch 为 `1920` / `1080` | 采集尺寸 |
| `framerate` | `60` | 帧率 |
| `output_encoding` | `bgr8` | `bgr8` 或 `rgba8` |
| `frame_id` | `camera` | 消息坐标系标签 |
| `image_topic` | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera_info` | 默认由图像话题推导 |
| `camera_name` | `camera` | 标定管理器中的名称 |
| `camera_info_url` | 空 | 实际标定文件 URL |
| `reconnect_interval` | `2.0` | 重连间隔（秒） |
| `frame_timeout` | `3.0` | 无可用帧超时（秒） |
| `max_frame_age` | `0.5` | 采集时间至取样处理的最大允许延迟（秒） |

参数类型错误、非正尺寸/帧率、非有限或非正超时、未知编码会导致启动失败。

## 验证范围

仓库不包含测试文件。修改时在仓库外检查像素通道、行填充、不对齐内存、尾部尺寸、时间戳计算和过期/未来时间拒绝。当前环境没有 ROS、GStreamer 开发库或 Jetson，完整节点编译、NEON 路径、硬件解码、USB 拔插恢复和实际 CPU 降幅仍需在目标设备验证。

实现依据：[GStreamer 时钟与同步](https://gstreamer.freedesktop.org/documentation/application-development/advanced/clocks.html)、[NVIDIA Jetson 加速 GStreamer](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)。

## 许可证

MIT，见 [LICENSE](LICENSE)。
