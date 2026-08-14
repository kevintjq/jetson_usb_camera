# Jetson USB Camera

ROS1 USB 摄像头驱动，使用 GStreamer 和 Jetson 硬件解码，将 MJPEG 视频发布为 ROS 图像和 `CameraInfo`。

## 特性

- Jetson NVIDIA 硬件解码
- 发布 `sensor_msgs/Image`，编码为 `bgr8`
- 发布 `/camera/camera_info`
- GStreamer 错误、EOS 或持续无帧时自动重连
- 支持通过 `camera_info_manager` 加载标定 YAML

## 环境要求

- Ubuntu + ROS1 / catkin
- NVIDIA Jetson
- GStreamer 1.0
- Jetson GStreamer 插件
- `camera_info_manager`
- 支持 MJPEG 输出的 USB 摄像头

安装常用依赖：

```bash
sudo apt install \
  ros-$ROS_DISTRO-camera-info-manager \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  v4l-utils
```

`nvv4l2decoder` 等 Jetson 插件由 NVIDIA JetPack 提供。

## 获取与编译

本仓库是一个标准 ROS 包，不是完整 catkin 工作区。将它克隆到工作区的 `src/` 目录：

```bash
mkdir -p ~/catkin_ws/src
git clone https://github.com/kevintjq/jetson_usb_camera.git \
  ~/catkin_ws/src/jetson_usb_camera

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 启动

```bash
roslaunch jetson_gst_camera camera.launch
```

默认配置：

```text
device:    /dev/video0
size:      1920x1080
framerate: 60 FPS
image:     /camera/image_raw
info:      /camera/camera_info
```

启动前检查摄像头能力：

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

当前 pipeline 主要支持 MJPEG 输入，摄像头必须支持所配置的分辨率和帧率。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `device` | `/dev/video0` | 摄像头设备 |
| `width` | `1920` | 图像宽度 |
| `height` | `1080` | 图像高度 |
| `framerate` | `60` | 帧率 |
| `frame_id` | `camera` | 图像坐标系名称 |
| `image_topic` | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `/camera/camera_info` | CameraInfo 话题 |
| `camera_info_url` | 空 | 标定 YAML URL |
| `reconnect_interval` | `2.0` | 重连间隔，单位秒 |
| `frame_timeout` | `3.0` | 无帧超时时间，单位秒 |

## 摄像头标定

没有标定文件时，节点仍会发布 `CameraInfo`，但内参和畸变参数无效。

完成标定后，在 `launch/camera.launch` 中配置：

```xml
<param name="camera_info_url"
       value="file:///home/nvidia/.ros/camera_info/camera.yaml"/>
```

不要使用未经实际标定生成的 YAML，否则会影响视觉定位、测距和图像校正结果。

## 话题检查

```bash
rostopic list
rostopic hz /camera/image_raw
rostopic echo /camera/camera_info
```

## 项目结构

```text
jetson_usb_camera/
  CMakeLists.txt
  package.xml
  src/jetson_gst_camera.cpp
  launch/camera.launch
  README.md
  LICENSE
```

## 许可证

MIT License，详见 [LICENSE](LICENSE)。
