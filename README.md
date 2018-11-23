# AutoCar
[![Build Status](https://travis-ci.org/hackath/AutoCar.svg?branch=master)](https://travis-ci.org/hackath/AutoCar)
[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

This program is detect the armor of robomaster robot, for Robomaster2018.

## Get started
开源的版本：

[西安电子科技大学Robomaster机器人队RM2018开源](https://github.com/hackath/AutoCar/releases)

- 小装甲识别耗时20ms以下, 平均16ms, 偶尔会有跳变,不会超过30ms 
- 大装甲识别耗时平均在20ms, 产生跳变后, 耗时控制在 35ms以下

目前的版本：
- 持续更新，侧重点不再是提高识别性能，最终效果待定

## Software Requirements

- Ubuntu14.04 or Ubuntu16.04
- CMake 3.1 or higher
- OpenCV 3

## Build and Run

```shell
mkdir build && cd build
cmake ..
make
./autocar 
```
## Documents
构建/自定义[Doxygen](http://www.doxygen.org/)文档，需要以下组件：
- Doxygen
- GraphViz
- Html Help

查看由[Doxygen](http://www.doxygen.org/)自动生成的代码文档：
```shell
doxygen .\Doxyfile
```

## Modules
### common
---

这是程序主框架, 包含程序入口和线程控制, 在main.cpp中构造 ImageConsProd 对象, 每个对象都对应一个摄像头, 传入xml配置文件即可, 加入std::thread实现多摄像头读取.

### detect_factory
---

这是检测算法模块, 包括 : 
- struct armor_param (装甲板参数, 全部是检测算法的阈值)
- struct armor_info (装甲板结构体, 包括目标区域和移动速度, 用于后续弹道预测),
- struct armor_pos (装甲板的姿态, 包括角度和距离信息, 用于角度解算)
- class armor_detect (检测算法的主体部分)

说明:

armor_detect在 ImageConsProd 主线程中调用 detect 方法 , 传入 cv::Mat src 与 装甲板vector容器, 检测到灯条会返回1 , 无灯条会返回0 , 该返回值暂时无用途, 只是预留.

### slover
---

这是解算位姿信息的模块, 包括:
- angle_solver (角度解算器, 需要传入摄像头的一系列参数构造, 包括相机云台的坐标系关系) 
- angle_solver_factory (角度解算工厂, 用不同的解算器构造)
- armor_recorder (记录历史识别信息, 然后对当前识别信息做筛选)

说明:

由detect_factory模块得到的装甲板vector容器需要传入armor_recorder进行整合,最后返回一个armor_pos即最终结果.(多装甲板的决策算法还需要改进).

### driver
---

驱动模块:
- camera_driver (构造 ImageConsProd 对象时传入的xml配置文件会构造camera实例, 其中会包括相机云台的坐标系关系, /dev/video* 的绑定方法见 /doc/*.ruls示例 )
- serial (串口模块, listen以5ms频率接收云台陀螺仪信息, publish随主线程发送)

串口接送:

|0xDA  | angle_yaw | angle_pitch | 0xDB    |
|------|-----------|-------------|---------|

串口发送:

|0xFF |state | angle_x | angle_y | angle_z| 0xFE|
|-----|------|---------|---------|--------|-----|

### utility
---

程序的调试模块,包括记录运行时间 time_cost, 绘制调试信息draw, 录制视频video_recorder.

### extra
---

#### armor_sample
运行该程序获取装甲板的训练集
这是一个曾经迭代版本的装甲识别算法,删减了部分检测算法,使得它可以在高识别armor的同时,误识别很多作为负样本进行训练.

#### train
设置一些参数后,运行该程序模块通过armor训练集得到最后的分类器.

## Copyright and License

Copyright (c) 2018 Robomaster, Xidian University. AutoCar is provided under the [BSD-2](https://opensource.org/licenses/BSD-2-Clause).
