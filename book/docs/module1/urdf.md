# Understanding URDF (Unified Robot Description Format)

## What is URDF?

URDF (Unified Robot Description Format) is an XML format for describing the physical structure of a robot, including:
- **Links**: Physical parts of the robot (segments)
- **Joints**: Connections between links
- **Visual**: How the robot looks
- **Collision**: How the robot interacts with the environment
- **Inertial**: Physical properties (mass, inertia)

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Joints -->
  <joint name="torso_to_head" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>
</robot>
```

## Link Elements

### Visual
Describes how the robot appears in visualization:
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://robot_description/meshes/head.dae"/>
  </geometry>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
</visual>
```

### Collision
Defines collision geometry (can be simpler than visual):
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.2 0.2 0.3"/>
  </geometry>
</collision>
```

### Inertial
Physical properties for physics simulation:
```xml
<inertial>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <mass value="5.0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

## Joint Types

### Fixed
No movement between links:
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

### Revolute
Rotation around a single axis:
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="2.0"/>
</joint>
```

### Prismatic
Linear movement along an axis:
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.0" effort="50" velocity="0.5"/>
</joint>
```

## Humanoid Robot Example

### Torso and Head
```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.4 0.3 0.6"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="20.0"/>
    <inertia ixx="0.5" ixy="0.0" ixz="0.0"
             iyy="0.5" iyz="0.0" izz="0.5"/>
  </inertial>
</link>

<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.35" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="50" velocity="1.0"/>
</joint>
```

### Arms and Legs
Similar structure for limbs with multiple joints (shoulder, elbow, wrist, etc.)

## Using URDF in ROS 2

### Loading URDF
```python
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_file('robot.urdf')
```

### Publishing Robot Description
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.publisher = self.create_publisher(String, 'robot_description', 10)
        with open('robot.urdf', 'r') as f:
            urdf_content = f.read()
        msg = String()
        msg.data = urdf_content
        self.publisher.publish(msg)
```

## Best Practices

1. **Modularity**: Break complex robots into separate URDF files
2. **Xacro**: Use Xacro macros for reusable components
3. **Collision Simplification**: Use simpler geometry for collision
4. **Inertial Properties**: Always include inertial properties for physics
5. **Joint Limits**: Set realistic joint limits based on hardware

