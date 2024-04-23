---
title: "gelsight成像与仿真+实际数据拟合方法"
date: 2023-10-21
description: "在gazebo仿真下的gelsight数据采集环境"
type: "post"
tags: ["blog","ros相关"]
---


# gelsight成像与仿真+实际数据拟合方法

- 论文原文: Generation of GelSight Tactile Images for Sim2Real Learning
- 项目源码： [https://danfergo.github.io/gelsight-simulation](https://danfergo.github.io/gelsight-simulation)

## 1. 引言

1. 为什么机器人用到触觉传感器：
    1. 视觉传感器在遮挡、不同光照条件下，效果不佳；
    2. 机器人操作环境中，避免不了出现手臂或手掌遮挡目标的情况；
2. 为什么要做sim2real：
    1. 节省时间与硬件资源，在仿真中训练，在实际环境中微调；
    2. rl类型训练需要试错积累，防止造成硬件损坏；
3. gelsight 这种visual-based 触觉传感器优势在哪：
    1. 相比于传统使用触觉单元的触觉传感器，其分辨率更高；
4. 如何为gelsight做sim2real：
    1. real world下，gelsight通过内部相机捕获膜形变的形式来回传触觉数据；
    2. 在gazebo1仿真中，用深度相机捕获接触物体表面深度图，再近似形变为膜形变的高度图：
        - 使用Bivariate Gaussian filtering（双变量高斯滤波）
        - 使用Phong着色模型渲染传感器内部照明
5. 方法评估与效果：
    1. 评估方法：**评估个毛,直接用**

可以扩展到unity和pybullet环境下；

## 2. gelsight工作原理具体说明

实际环境下sensor的工作原理：

1. 组成部分： 凝胶、透明硬玻璃板、相机、内部照明设备；
2. 相机监测凝胶形变，产生触觉图片； 内部照明有单光类型，也有RGB类型；
3. 仿真下sensor通过 Phong着色模型渲染，可以选定基础图片颜色；

## 3. 论文中用到的仿真模型说明

仿真中sensor的数据采集原理：

- 直接从**深度图生成触觉图像**，流程如下图所示：
    
    1.1 elastomer的高度图基于深度相机捕获的深度图生成（也就是形变图），
    
    1.2 再通过Gaussian 滤波平滑化，
    
    2.1 计算表明法线为离散导数，
    
    2.2 再应用Phong模型进行着色（计算内部照明）；
    

![Untitled](gelsight%E6%88%90%E5%83%8F%E4%B8%8E%E4%BB%BF%E7%9C%9F+%E5%AE%9E%E9%99%85%E6%95%B0%E6%8D%AE%E6%8B%9F%E5%90%88%E6%96%B9%E6%B3%95%20ef28dd2ddd0c4027a663280bfe9bf382/Untitled.png)

### 弹性体（凝胶）的高度图如何从深度图转换而来：

- 在仿真中，把一个基于结构光的深度相机放在实际中RGB相机位置上，如下图：
    
    ![Untitled](gelsight%E6%88%90%E5%83%8F%E4%B8%8E%E4%BB%BF%E7%9C%9F+%E5%AE%9E%E9%99%85%E6%95%B0%E6%8D%AE%E6%8B%9F%E5%90%88%E6%96%B9%E6%B3%95%20ef28dd2ddd0c4027a663280bfe9bf382/Untitled%201.png)
    
    - 仿真中直接获取弹性体与物体的接触点深度；
    - 根据弹性体能够接触到的最大深度$d_{max}$，对深度图做阈值化处理，得到形变高度图$H_0$；
        - $H_0(x, y)=\left\{
        \begin{aligned}
        D(x,y) & &ifD(x,y)<=D_{max} \\
        d_{max} & & otherwise \\
        \end{aligned}
        \right.$
        - 这样获得了物体接触凝胶导致的初始形变深度；
    - 但是实际环境下的凝胶形变有更平滑的边缘，在接触压痕周围产生颜色梯度，使用2D高斯滤波对$H_{0}$做优化：
    
    ![Untitled](gelsight%E6%88%90%E5%83%8F%E4%B8%8E%E4%BB%BF%E7%9C%9F+%E5%AE%9E%E9%99%85%E6%95%B0%E6%8D%AE%E6%8B%9F%E5%90%88%E6%96%B9%E6%B3%95%20ef28dd2ddd0c4027a663280bfe9bf382/Untitled%202.png)
    
    - 利用Gaussian difference 计算最终的高度差：
        - $H_{DoG}=2H_{narrow}-H_{wide}$；

重点：这里可以看出，实际得到的形变数据和最初的深度信息不太一样，**每个像素点的深度不能和触觉传感器形变全部对应上**；

### 弹性体的内部照明模拟

- 涉及物理回头再看吧
