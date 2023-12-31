# 视觉练习项目Todo List

## 第一阶段: 制作四轮小车

### 利用stm32等开发板为核心制作一个四轮小车，利用遥控器控制小车行动轨迹

- **所需时间**: 2-4周
- **工具链**: 
  - STM32开发板
  - 遥控器
  - 电机驱动模块
- **材料**: 
  - 机械结构（四轮小车底盘、轮子、电机等）
  - 电子元件（电池、电机、线缆等）
- **技术栈**:
  - 嵌入式编程（C/C++）
  - 电机控制
  - 遥控器通信

**知识点和知识体系**:
- 理解STM32微控制器的基础概念
- 掌握C/C++编程语言
- 学习电机控制和PWM（脉冲宽度调制）
- 了解遥控器通信协议，例如RC遥控器的信号解析

## 第二阶段: 循线运动能力

### 在已做好的小车上添加摄像头模块，利用opencv实现循线运动能力

- **所需时间**: 2-4周
- **工具链**:
  - STM32开发板
  - 摄像头模块
  - OpenCV
- **材料**:
  - 视觉传感器
- **技术栈**:
  - 图像处理
  - 循线算法

**知识点和知识体系**:
- 学习图像获取和摄像头控制
- 掌握图像预处理技术，如颜色过滤和边缘检测
- 理解循线算法，包括PID控制算法和轨迹跟踪

## 第三阶段: 改造为平衡小车

### 将四轮小车改造为两轮平衡小车，功能不变，并实现动态调节参数

- **所需时间**: 2-4周
- **工具链**:
  - STM32开发板
- **材料**:
  - 平衡系统（陀螺仪、加速度计等）
- **技术栈**:
  - 平衡控制算法

**知识点和知识体系**:
- 学习平衡系统的原理，如倒立摆
- 掌握PID控制在平衡中的应用
- 理解传感器数据的处理和反馈，以保持平衡

## 第四阶段: 简单目标识别

### 在小车已有功能的基础上实现在简单环境下的目标识别，实现小车移动向特定目标（较易识别的简单几何体）

- **所需时间**: 3-6周
- **工具链**:
  - STM32开发板
  - 摄像头模块
  - OpenCV
- **材料**:
  - 颜色传感器或深度摄像头
- **技术栈**:
  - 目标检测
  - 运动控制

**知识点和知识体系**:
- 学习物体检测算法，如Haar级联分类器和YOLO
- 掌握特定颜色或形状的物体识别技术
- 理解如何移动小车以追踪和向目标移动的路径规划

## 第五阶段: 跟随动态目标

### 在小车已有功能的基础上实现小车跟随特定动态目标（较易识别的简单几何体）

- **所需时间**: 3-6周
- **工具链**:
  - STM32开发板
  - 摄像头模块
  - 视觉跟踪算法
- **材料**:
  - 动态目标（例如移动的球）
- **技术栈**:
  - 目标跟踪算法

**知识点和知识体系**:
- 学习目标跟踪算法，包括单目标和多目标跟踪
- 掌握卡尔曼滤波或其他滤波方法，以实现目标位置的估计
- 理解如何检测和识别移动的目标，以及如何追踪它们的轨迹

## 第六阶段: 静态目标击中

### 在小车已有功能的基础上实现静态小车识别特定静态目标并发射物件击中该目标（较易识别的简单几何体）

- **所需时间**: 4-8周
- **工具链**:
  - STM32开发板
  - 摄像头模块
  - 弹丸发射机构
- **材料**:
  - 靶标或靶物
- **技术栈**:
  - 弹道计算
  - 目标击中算法

**知识点和知识体系**:
- 学习目标识别算法，以实现静态目标的检测和识别
- 掌握弹丸速度和弹道计算，以确定打击角度和位置
- 理解如何实现准确的目标击中算法

## 第七阶段: 动态目标击中

### 在小车已有功能的基础上实现小车动态移动并识别特定动态目标并发射物件击中该目标（较易识别的简单几何体）

- **所需时间**: 4-8周
- **工具链**:
  - STM32开发板
  - 摄像头模块
  - 高级目标追踪和打击算法
- **材料**:
  - 移动的目标（例如移动的机器人）
- **技术栈**:
  - 高级目标追踪
  - 动态目标打击算法

**知识点和知识体系**:
- 学习高级目标追踪算法，以实现复杂目标的追踪和预测
- 掌握多传感器融合技术，提高目标追

踪的准确性
- 理解动态目标运动模型和自适应控制策略，以实现弹丸击中动态目标的算法

## 需要掌握的算法清单：

- 图像处理算法（用于摄像头数据的预处理和分析）
- 循线算法（用于四轮小车的自动导航）
- PID控制算法（用于小车平衡和轨迹控制）
- 目标检测算法（用于识别目标物体）
- 目标跟踪算法（用于追踪目标物体的运动）
- 弹道计算算法（用于计算击中目标所需的弹道）
- 卡尔曼滤波或其他滤波算法（用于目标位置估计）
- 高级目标追踪算法（用于复杂目标的追踪和预测）
- 动态目标打击算法（用于弹丸击中动态目标）
