---
id: sensor-simulation
title: Sensor Simulation
---

# Module 2: Digital Twin - Sensor Simulation

A robot is only as good as its sensors. To create a truly useful digital twin, we need to be able to simulate the sensors that the robot uses to perceive its environment. This section covers how to simulate common robotic sensors in Gazebo.

## Why Simulate Sensors?

Simulating sensors is crucial for developing and testing a wide range of robotics applications, especially those involving perception and navigation.
*   **Perception Algorithms:** You can test algorithms for object detection, lane following, and other vision-based tasks without needing a physical camera.
*   **Navigation and Mapping:** You can develop and test SLAM (Simultaneous Localization and Mapping) algorithms using simulated LIDAR and IMU data.
*   **Safety Systems:** You can test obstacle avoidance and other safety-critical systems in a controlled, repeatable environment.

## Simulating Sensors in Gazebo

Gazebo provides a variety of plugins for simulating common robotic sensors. These plugins are added to your robot's URDF or SDF file.

### Cameras

Gazebo can simulate both RGB and depth cameras. The camera plugin publishes images to a ROS2 topic, which you can then view in RViz2 or use in your perception nodes.

Here is an example of a camera plugin in a URDF file:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>my_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
    </plugin>
  </sensor>
</gazebo>
```

### LIDAR

LIDAR (Light Detection and Ranging) is a crucial sensor for navigation and mapping. Gazebo can simulate both 2D and 3D LIDAR sensors. The LIDAR plugin publishes `sensor_msgs/LaserScan` messages containing range data.

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="head_hokuyo_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### IMU (Inertial Measurement Unit)

An IMU measures a robot's orientation, angular velocity, and linear acceleration. The Gazebo IMU plugin publishes `sensor_msgs/Imu` messages.

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
            <namespace>/demo</namespace>
            <remapping>~/out:=imu</remapping>
        </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Visualizing Sensor Data in RViz2

RViz2 is the perfect tool for visualizing the data from your simulated sensors. You can add displays for:
*   **Images:** To see the output of a simulated camera.
*   **LaserScans:** To visualize the points from a LIDAR sensor.
*   **IMU:** To display the orientation of the robot.

## Noise Models

Real-world sensors are noisy. To create a more realistic simulation, Gazebo allows you to add noise to your sensor data. You can add a `<noise>` block to your sensor's description to specify the type and parameters of the noise.

```xml
<sensor type="ray" name="head_hokuyo_sensor">
  ...
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>
  </noise>
  ...
</sensor>
```

By accurately simulating your robot's sensors, you can build and test a complete autonomy stack, from perception to control, all within the safe and repeatable environment of your digital twin. This is an indispensable part of the modern robotics development workflow.