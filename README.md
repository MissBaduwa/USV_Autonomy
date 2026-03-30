
# 🚤 Robotic USV Autonomy - Water Surface Garbage Collection

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-34a853?logo=ros)](https://docs.ros.org/en/humble/)
[![Jetson](https://img.shields.io/badge/Jetson-Nano-76b900?logo=nvidia)](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Object%20Detection-00FFFF?logo=ultralytics)](https://github.com/ultralytics/ultralytics)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 🎯 Overview

An autonomous Unmanned Surface Vehicle (USV) system for water surface garbage collection using YOLOv8 object detection, GPS navigation, and vision-based verification. Deployed on NVIDIA Jetson Nano.

### 📊 Performance Metrics
- **Detection Accuracy**: mAP@50 of **68.68%** (exceeds target of 65%)
- **Classes Detected**: garbage (27,554 samples), aquatic_animal (1,726), plants (1,670)
- **Inference Speed**: 10-15 FPS with TensorRT FP16
- **Navigation**: GPS + Accelerometer (no IMU required!)

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🎯 **Object Detection** | YOLOv8m trained on 10,154 images for 3 classes |
| 🗺️ **GPS Navigation** | Waypoint following with coverage path planning |
| 🎥 **Camera-Based Obstacle Avoidance** | No ultrasonic sensor needed! |
| ✅ **Vision Verification** | Confirms collection by comparing before/after frames |
| 🔄 **Retry Logic** | Up to 3 attempts per garbage item |
| 📡 **ROS 2 Integration** | Modular architecture with custom messages |

## 🚀 Quick Start (Jetson Nano)

### One-Command Install:
```bash
curl -s https://raw.githubusercontent.com/MissBaduwa/Robotic_USV_Autonomy/main/scripts/quick_install.sh | bash
