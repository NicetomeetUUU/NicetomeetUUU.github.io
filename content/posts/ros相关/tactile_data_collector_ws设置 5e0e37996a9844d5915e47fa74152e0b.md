---
title: "自建触觉数据采集功能包"
date: 2023-10-16
description: "在gazebo仿真下的gelsight数据采集环境"
type: "post"
tags: ["blog","ros相关"]
---

# tactile_data_collector_ws设置

Tags: Gazebo, ROS
Person: COO Ni

# gazebo_tactile_data_simulator功能包

这个功能包主要完成 **ur10e arm和 gelsight在 gazebo中的连通, 末端位姿控制与触觉数据采集**；

**直接上结论:**

默认需要在同workspace src目录下放置 universal_robot 功能包和 gelsight_description 功能包；

## 1. 首先将gelsight模型搭载到ur10e的末端 tool0 关节, 修改ur_gazebo下的urdf/ur_macro.xacro文件如下:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

<!--前面都跳过 -->
  <xacro:include filename="$(find ur_description)/urdf/inc/ur_macro.xacro"/>

  <!-- Add the gelsight on the end of the robot -->
  <xacro:include filename="$(find gelsight_description)/urdf/gelsight2014.xacro"/>

  <joint name="hook_gelsight_printer" type="fixed">
      <origin xyz="0 0.007 0.005" rpy="0 0 ${pi/2}"/>
      <parent link="tool0"/>
      <child link="gelsight_base"/>
  </joint>

<!--后面也都跳过,不需要修改 -->
  </xacro:macro>
</robot>
```

## 2. 写launch文件, 启动ur10e_bringup.launch 及控制节点等

launch文件1: arm_bringup.launch; 

launch文件2: tactile_data_sim_world.launch

```xml
<?xml version="1.0"?>

<launch>
    <arg name="gazebo_world" default="$(find gazebo_tactile_data_simulator)/worlds/tactile_data_sim.world"/>
    
    <!-- 在gazebo下, 加载ur10e机械臂 -->
    <include file="$(find ur_gazebo)/launch/ur10e_bringup.launch">
        <arg name="gazebo_world" value="$(arg gazebo_world)" />
        <arg name="paused" value="true"/>
    </include>
		
</launch>
```

```xml
<?xml version="1.0"?>
<launch>
    <arg name="sim" default="true"/>

    <!-- 修改gazebo的pause状态 -->
    <node name="start_gazebo_sim"
        pkg="gazebo_tactile_data_simulator"
        type="start_gazebo.py"/>

    <!-- 启动ur10e机械臂控制节点 -->
    <include file="$(find ur10e_moveit_config)/launch/moveit_planning_execution.launch">
        <arg name="sim" value="$(arg sim)"/>
    </include>
    <include file="$(find gazebo_tactile_data_simulator)/launch/moveit_rviz.launch"/>
    
    <!-- 起gelsight tactile data 渲染节点 -->
    <group if="$(arg sim)">
    <node   name="gelsight_sim"
            pkg="gelsight_gazebo"
            type="gelsight_driver.py"/>
    </group>

</launch>
```

这里调用了 gelsight_driver节点, 节点通过深度图拟合触觉数据 pub出来, 后面要订阅并存储触觉数据

## 3. 新版ur功能包使用pid控制关节, 导致gazebo load model 会发生 模型先因重力偏转, 后上力而改变初始位姿的现象 以及 末端挂载传感器质量过大 末端振动现象；

解决办法: 

- 修改gelsight_description中模型描述文件, 将其质量惯性等设置为0.001；
- 在ur10e_bringup.launch 中, 以paused模式启动, 防止重力先于力矩作用；
- 写一个节点 修改 gazebo的 pause参数, 在ur机械臂的运动前(tactile_data_sim_world.launch最先调用)；
- 设置机械臂运动到home位姿, 便于下一步规划

```xml
<!--模型描述文件自己改去, 注意有四处mass需要修改 -->
```

**用于start gazebo的节点内容如下:**

```python
#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Float64

def set_gazebo_pause(pause):
    if pause:
        service_name = '/gazebo/pause_physics'
    else:
        service_name = '/gazebo/unpause_physics'

    rospy.wait_for_service(service_name)
    try:
        change_pause = rospy.ServiceProxy(service_name, Empty)
        change_pause()
        rospy.loginfo("Gazebo pause status set to: {}".format(pause))
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call Gazebo pause service: {}".format(e))

def set_arm_joint_position(joint_name, position):
    topic_name = '/arm_controller/{}_position_controller/command'.format(joint_name)
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    pub.publish(position)
    rospy.loginfo("Set joint {} position to: {}".format(joint_name, position))

def Move_to_home():
    arm = moveit_commander.MoveGroupCommander('manipulator')
    end_effector_link = arm.get_end_effector_link()

    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)

    arm.allow_replanning(True)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    arm.set_named_target('manipulator_home')
    arm.go()
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('external_node')
    # 暂缓3s，等待gazebo仿真环境启动
    rospy.sleep(3)
    # 设置暂停属性为False，即启动仿真
    set_gazebo_pause(False)
    rospy.sleep(1)
    # 暂缓1s, 等待力矩与关节角度稳定
    Move_to_home()
```

## 4. 调整spawn_model为机械臂添加初始关节位姿和拿到touch数据的目标物体

调整xacro文件, 设定可修改参数, 自动调用并集成ur机械臂和gelsight传感器以及目标物体；

(为了防止物体因碰撞运动, 将其设定为关节类型物体, 连接在baselink上)

- 解决办法:
    - 通过xacro文件层层递进传递参数, 这里为了方便调整, **后期需要修改launch文件内容**:
    - **修改后再添加, 先完成后面部分内容.**

调整发现深度相机的点云无法对齐, 需要找到一个正确的frame进行映射, 深度图像由gelsight_camera节点发布, 理论上应该在该link处定义一个朝向正确的link作为点云图像的对齐基准.

- 解决办法:
    - 这里在gelsight_camera.xacro文件中, 设定一个link命名为gelsight_link, 用于对齐点云；
    - rpy分别代表xyz三个轴上旋转角, 在rviz中, 红x绿y蓝z, 为右手坐标系, 逆时针旋转为正向旋转角. (基于自身坐标系)
    - 将深度图映射到旋转link上即可, 代码如下(在z轴上还是有点偏移, 可能是**相机内参没有设置好,** 后面读数据再调整一下):
    
    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://wiki.ros.org/xacro">
        <xacro:macro name="gelsight_camera">
            <link name='gelsight_camera'></link>
            <link name='gelsight_link' />
            <!-- cannot find the gelsight_link, so we set new gelsight_link as the cloud frame -->
            <joint name='gelsight_link_to_camera' type='fixed'>
                <!-- ${pi} -->
                <parent link="gelsight_camera"/>
                <child link="gelsight_link"/>
                <origin xyz="0 0 0" rpy="${-pi/2} 0.0 ${-pi/2}"/>
            </joint>
            <gazebo reference="gelsight_camera">
                <sensor type="depth" name="gelsight_camera_sensor">
                    <always_on>1</always_on>
                    <update_rate>30.0</update_rate>
                    <visualize>1</visualize>
                    <topic>/depth_camera</topic>
                    <camera name="head">
    <!--                    <horizontal_fov>1.04719755</horizontal_fov>-->
                        <horizontal_fov>0.5</horizontal_fov>
                        <image>
                            <width>640</width>
                            <height>480</height>
                            <format>R8G8B8</format>
                        </image>
                        <clip>
                            <near>0.0001</near>
                            <far>300</far>
                        </clip>
                        <depth_camera>
                            <output>depths</output>
                        </depth_camera>
                    </camera>
                    <plugin name="gelsight_link_controller" filename="libgazebo_ros_openni_kinect.so">
                        <baseline>0.2</baseline>
                        <alwaysOn>true</alwaysOn>
                        <updateRate>30.0</updateRate>
                        <cameraName>gelsight_ir</cameraName>
                        <imageTopicName>/gelsight/image/image_raw</imageTopicName>
                        <cameraInfoTopicName>/gelsight/depth/camera_info</cameraInfoTopicName>
                        <depthImageTopicName>/gelsight/depth/image_raw</depthImageTopicName>
                        <depthImageInfoTopicName>/gelsight/depth/camera_info</depthImageInfoTopicName>
                        <pointCloudTopicName>/gelsight/depth/points</pointCloudTopicName>
                        <frameName>gelsight_link</frameName>
                        <!-- <frameName>gelsight_base</frameName> -->
                        <pointCloudCutoff>0.0</pointCloudCutoff>
                        <distortionK1>0</distortionK1>
                        <distortionK2>0</distortionK2>
                        <distortionK3>0</distortionK3>
                        <distortionT1>0</distortionT1>
                        <distortionT2>0</distortionT2>
                        <CxPrime>0</CxPrime>
                        <Cx>0</Cx>
                        <Cy>0</Cy>
                        <focalLength>0</focalLength>
                        <hackBaseline>0</hackBaseline>
                    </plugin>
                </sensor>
            </gazebo>
        </xacro:macro>
    </robot>
    ```
    

## 5. 写python版本节点(为了避免代码异构), 这部分写一个pose_publisher发布数据采集目标位姿:

### 在scripts路径下新建 move_target_publisher.py 文件:

- 实例化一个publisher, 方便后期RL训练直接调用

```python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('move_target_publisher')
    pub = rospy.Publisher('/move_target/pose', Pose, queue_size=1)
    target_pose = Pose()
    target_pose.position.x = 0.86
    target_pose.position.y = 0.36
    target_pose.position.z = 0.34
    target_pose.orientation.x = -0.707
    target_pose.orientation.y = 0.707
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0

    target_pose1 = Pose()
    target_pose1.position.x = 0.7
    target_pose1.position.y = 0.37
    target_pose1.position.z = 0.15
    target_pose1.orientation.x = -0.707
    target_pose1.orientation.y = 0.707
    target_pose1.orientation.z = 0
    target_pose1.orientation.w = 0
    while not rospy.is_shutdown():
       input("Press Enter to continue...")
       pub.publish(target_pose)
       rospy.sleep(0.5)
```

## 6. 根据gelsight_simulation下的data_conllection.py, 写tactile data collector.py

### 首先需要看一下成像原理与sim2real具体方法:

[gelsight成像与仿真+实际数据拟合方法](tactile_data_collector_ws%E8%AE%BE%E7%BD%AE%205e0e37996a9844d5915e47fa74152e0b/gelsight%E6%88%90%E5%83%8F%E4%B8%8E%E4%BB%BF%E7%9C%9F+%E5%AE%9E%E9%99%85%E6%95%B0%E6%8D%AE%E6%8B%9F%E5%90%88%E6%96%B9%E6%B3%95%20ef28dd2ddd0c4027a663280bfe9bf382.md)

- 论文中提到形变是通过深度图经过2D Gaussian filter 平滑化得到的，**不确定是否可以直接使用depth中读到的**。
- tactile_data_[collector.py](http://collector.py) 预计功能:
    - 需要根据当前采集坐标, 沿tool0的z轴运动, **采集多张数据**；
    - 或者按照当前最大深度与最小深度区间**过滤点云数量**, 数量达标则直接存储当前gelsight_base位姿+截取的点云数据+触觉图像；
- 功能实现具体要做的:
    - 设计回调函数, 通过订阅RL节点 pub出来的 gelsight_base末端位姿, 调用**反解算法移动到目标点**；
    - 末端位姿**步进式数据采集**, 点云数量到达阈值才存储；
    - 触觉图像采集, 已经给出初始的图像, 后面可能需要**添加褶皱噪声**方便sim2real训练；
    - 局部点云采集, 根据截断深度采集当前点云数据, 可能需要转换矩阵转换到base link下:
        - 坐标系间的变换矩阵:
            - 可以通过transforms3d, 订阅tf下的左边转换关系, 计算得到转换矩阵
                
                ```bash
                #依赖包: transforms3d
                pip install transforms3d
                ```
                
            - 将当前已达到合理点云数量的gelsight_link位姿存储, 并存储tactile_image和point_cloud；
            - 再额外写一个转换脚本, 根据gelsight_link的pose和point_cloud数据, 计算转换矩阵并将点云转换到base_link下, 这样两个数据都采集到了；
- tactile image获得方式**并非只通过截取深度实现**, 根据其源码可以分析实现方式如下:
    - 这里需要分析一下gelsight_driver.py脚本如何发布tactile_image:
        
        [gelsight_driver.py文件解析](tactile_data_collector_ws%E8%AE%BE%E7%BD%AE%205e0e37996a9844d5915e47fa74152e0b/gelsight_driver%20py%E6%96%87%E4%BB%B6%E8%A7%A3%E6%9E%90%20aee450b879884d5fa5922b4576aba811.md)
        
    1. 通过文件定义可以看到:
        1. 凝胶厚度定义为0.004, 最小深度为0.026, 只需要截取 0.026-0.004区间内的点云数据；
        
        ```python
        px2m_ratio = 5.4347826087e-05
        elastomer_thickness = 0.004
        min_depth = 0.026  # distance from the image sensor to the rigid glass outer surface
        texture_sigma = 0.00001
        ```
        

### tactile_data_collector.py如下:

### 主要功能:

1. 采集tactile_image；
2. 采集并规范化 camera_link 下的 点云数据；
3. 订阅pose_publisher, 移动到该位姿并执行data_collect()；

### 现存问题:

- [x]  机械臂末端反解规划, x, y轴精度可达0.5mm, z轴精度为3mm, 无法达成步进采集效果;
- [x]  由于转轴上角度也有误差, 持续的反解由于z轴与转角误差, 会导致末端无法按照直线运动；
- [ ]  数据采集函数执行问题, 跟不上point_cloud回调函数刷新率, 导致移动位置有问题；

### 解决办法:

1. 仿真中z轴误差由重力引起, 采集数据不需要重力, 直接置零；
    1. 置零办法是将启动时选定的world文件内部 <gravity> 条目由 0 0 -9.8  置为 0 0 0；
2. 重力删除后, 转轴误差也很小, 没有问题；
3. 回调函数只用于修改初始点云, 在每次调用步进之前执行点云筛选, 在运动前保证获取稳定的点云数据；

## 7. 环境控制节点 env_controller.py

### 本节点主要用于切换环境模型, 为机械臂发布采集位姿与控制信号；

1. 实现了在gazebo下导入或者导出sdf和urdf类型模型的功能；
2. 实现了为监听节点发布目标位姿的功能；

### 现存问题:

- [x]  gazebo下模型会因无重力而在受到碰撞后匀速飘走；
- [x]  采集功能还有待完善, 速度过慢, 是个问题；
- [x]  点云深度最小值有问题,需要过滤；
- [x]  连续控制理论上应该使用service通信模式, 而非topic模式；
- [ ]  gazebo2rviz中marker并不能实现避障, 且marker坐标不正确；

### 解决办法:

1. gazebo中导入模型可以**将  static  属性设置为 true** , 这样就不会因外力干扰而产生运动了；
    
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="abc">
        <static>true</static>  <!--这里即为static声明-->
        <link name="link">
          <pose>0 0 0.036 0 0 0</pose>
          <inertial>
            <mass>0.5167</mass>
            <inertia>
              <ixx>0.0008611666666666669</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.0008611666666666669</iyy>
              <iyz>0</iyz>
              <izz>0.0008611666666666669</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <mesh>
                <uri>model://abc/meshes/abc.dae</uri>
                <scale>0.5 0.5 0.5</scale>
              </mesh>
            </geometry>
            <surface>
              <friction>
                <torsional>
                  <coefficient>1.0</coefficient>
                  <use_patch_radius>0</use_patch_radius>
                  <surface_radius>0.01</surface_radius>
                </torsional>
              </friction>
            </surface>
          </collision>
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>model://abc/meshes/abc.dae</uri>
                <scale>0.5 0.5 0.5</scale>
              </mesh>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    ```
    
2. 采集速度提升上想直接运动到相距最近的物体表面, 不过这种实现方式对初始的探索位姿要求较高；
3. 点云深度最小值直接在点云集合内获取, 暂时未发现问题
4. Service通信可以让客户端了解当前服务端是否成功实现服务, 防止重复请求相同的服务；
5. 放弃gazebo2rviz节点使用, 直接拿moveit_commander命令添加dae类型的模型到世界中
    - 遇到问题:
        - 中心点问题, dae模型和sdf模型的中心点不是对齐的, 导致物体位置有问题；
    - 解决办法:
        - 追本溯源还是要让机械臂规划到球面位姿的过程中不与目标物体产生碰撞, 那么可以在规划到球面位姿的时候加载一个长方体替代目标物体, 在开始采集后将该长方体移除, 也可实现想要的效果；

## debug记录(从异步停止机械臂运动版本开始记录):

### 20240318:

1. 机械臂在地卡尔空间坐标间规划路径, maxaccelerate和maxvelocity设置
2. 现在异步停止机械臂有问题，gazebo中未实现move过程中异步stop的操作，现回溯到最开始的工作模式。