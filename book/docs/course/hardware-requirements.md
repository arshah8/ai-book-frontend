# Hardware Requirements

## Overview

This course is technically demanding. It sits at the intersection of three heavy computational loads:
- **Physics Simulation** (Isaac Sim/Gazebo)
- **Visual Perception** (SLAM/Computer Vision)
- **Generative AI** (LLMs/VLA)

## Option 1: The "Digital Twin" Workstation (Required per Student)

This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities.

### GPU (The Bottleneck)
- **Minimum**: NVIDIA RTX 4070 Ti (12GB VRAM)
- **Ideal**: RTX 3090 or 4090 (24GB VRAM) for smoother "Sim-to-Real" training

### CPU
- **Minimum**: Intel Core i7 (13th Gen+) or AMD Ryzen 9
- **Why**: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive

### RAM
- **Minimum**: 32 GB DDR5
- **Recommended**: 64 GB DDR5 (32 GB may crash during complex scene rendering)

### OS
- **Required**: Ubuntu 22.04 LTS
- **Note**: While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory.

## Option 2: The "Physical AI" Edge Kit

Since a full humanoid robot is expensive, students learn "Physical AI" by setting up the nervous system on a desk before deploying it to a robot.

### The Brain
- **NVIDIA Jetson Orin Nano** (8GB) or **Orin NX** (16GB)
- **Role**: Industry standard for embodied AI. Students deploy ROS 2 nodes here to understand resource constraints.

### The Eyes (Vision)
- **Intel RealSense D435i** or **D455**
- **Role**: Provides RGB and Depth data. Essential for VSLAM and Perception modules.

### The Inner Ear (Balance)
- **Generic USB IMU (BNO055)**
- **Note**: Often built into RealSense D435i or Jetson boards, but a separate module helps teach IMU calibration.

### Voice Interface
- **USB Microphone/Speaker array** (e.g., ReSpeaker)
- **Role**: For "Voice-to-Action" Whisper integration.

## Option 3: The Robot Lab

### Option A: The "Proxy" Approach (Recommended for Budget)
- **Robot**: Unitree Go2 Edu (~$1,800 - $3,000)
- **Pros**: Highly durable, excellent ROS 2 support, affordable
- **Cons**: Not a biped (humanoid)

### Option B: The "Miniature Humanoid" Approach
- **Robot**: Unitree G1 (~$16k) or Robotis OP3 (~$12k)
- **Budget Alternative**: Hiwonder TonyPi Pro (~$600)
- **Warning**: Cheap kits usually run on Raspberry Pi, which cannot run NVIDIA Isaac ROS efficiently

### Option C: The "Premium" Lab
- **Robot**: Unitree G1 Humanoid
- **Why**: One of the few commercially available humanoids that can actually walk dynamically

## Option 4: Cloud-Native Lab (High OpEx)

### Cloud Workstations
- **Instance Type**: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge
- **Cost**: ~$1.50/hour (spot/on-demand mix)
- **Usage**: 10 hours/week × 12 weeks = 120 hours
- **Total Cloud Bill**: ~$205 per quarter

### Local "Bridge" Hardware
- **Edge AI Kits**: Still needed for physical deployment ($700)
- **Robot**: Still needed for final demo ($3,000 for Unitree Go2)

## The Economy Jetson Student Kit

Best for: Learning ROS 2, Basic Computer Vision, and Sim-to-Real control.

| Component | Model | Price (Approx.) |
|-----------|-------|----------------|
| The Brain | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 |
| The Eyes | Intel RealSense D435i | $349 |
| The Ears | ReSpeaker USB Mic Array v2.0 | $69 |
| Wi-Fi | (Included in Dev Kit) | $0 |
| Power/Misc | SD Card (128GB) + Jumper Wires | $30 |
| **TOTAL** | | **~$700** |

## Summary

To teach this successfully, your lab infrastructure should include:

| Component | Hardware | Function |
|-----------|----------|----------|
| Sim Rig | PC with RTX 4080 + Ubuntu 22.04 | Runs Isaac Sim, Gazebo, Unity, trains LLM/VLA models |
| Edge Brain | Jetson Orin Nano | Runs the "Inference" stack |
| Sensors | RealSense Camera + Lidar | Connected to Jetson for real-world data |
| Actuator | Unitree Go2 or G1 (Shared) | Receives motor commands from Jetson |

