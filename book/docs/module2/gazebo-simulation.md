# Gazebo Simulation

## What is Gazebo?

Gazebo is a powerful physics simulation environment that allows you to test robot behaviors in realistic virtual environments. It simulates:
- **Physics**: Rigid body dynamics, collisions, friction
- **Gravity**: Realistic gravitational effects
- **Sensors**: LiDAR, cameras, IMUs, force sensors
- **Actuators**: Motors, servos, pneumatic systems

## Setting Up Gazebo

### Installation

```bash
# Ubuntu 22.04
sudo apt-get update
sudo apt-get install gazebo11 libgazebo11-dev

# Or use ROS 2 Humble (includes Gazebo)
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Launching Gazebo

```bash
# Empty world
gazebo

# With a specific world
gazebo worlds/empty.world

# From ROS 2
ros2 launch gazebo_ros gazebo.launch.py
```

## URDF and SDF Formats

### URDF (Unified Robot Description Format)
XML format for describing robot structure:
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### SDF (Simulation Description Format)
More advanced format used by Gazebo:
```xml
<sdf version="1.6">
  <model name="my_robot">
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

## Physics Simulation

### Physics Engines

Gazebo supports multiple physics engines:
- **ODE** (Open Dynamics Engine): Default, fast, good for most cases
- **Bullet**: Better for complex collisions
- **Simbody**: More accurate, slower

### Configuring Physics

```xml
<physics name="default_physics" default="0" type="ode">
  <gravity>0 0 -9.8066</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

## Simulating Sensors

### LiDAR Simulation

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.10</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser_controller.so"/>
</sensor>
```

### Camera Simulation

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so"/>
</sensor>
```

### IMU Simulation

```xml
<sensor name="imu" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
    </angular_velocity>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so"/>
</sensor>
```

## Creating Environments

### Building Worlds

```xml
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="obstacle">
      <pose>5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## ROS 2 Integration

### Spawning Robots

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -file robot.urdf \
  -x 0 -y 0 -z 0.5
```

### Controlling Robots

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        self.publisher = self.create_publisher(
            Float64,
            '/joint_controller/command',
            10)
    
    def move_joint(self, position):
        msg = Float64()
        msg.data = position
        self.publisher.publish(msg)
```

## Best Practices

1. **Start Simple**: Begin with basic shapes before complex models
2. **Test Physics**: Verify physics parameters match real hardware
3. **Optimize Performance**: Use simplified collision models
4. **Validate Sensors**: Ensure sensor data matches real sensors
5. **Version Control**: Keep world files in version control

