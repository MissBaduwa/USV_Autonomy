#!/bin/bash
# ACCELEROMETER SETUP FOR JETSON NANO

echo "📊 Setting up Accelerometer (MPU6050)"
echo "===================================="

# Enable I2C
echo "🔧 Enabling I2C..."
sudo apt-get install -y i2c-tools

# Check I2C devices
echo "🔍 Scanning I2C bus..."
sudo i2cdetect -r 1

# Install Python library
echo "📦 Installing MPU6050 driver..."
pip3 install mpu6050-raspberrypi

# Create test script
cat > /tmp/test_accel.py << 'PY_EOF'
import mpu6050
import time

try:
    mpu = mpu6050.mpu6050(0x68)
    print("✅ MPU6050 found!")
    accel = mpu.get_accel_data()
    print(f"Acceleration: x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f}")
except Exception as e:
    print(f"❌ MPU6050 not found: {e}")
    print("Check wiring: VCC→3.3V, GND→GND, SDA→Pin3, SCL→Pin5")
PY_EOF

python3 /tmp/test_accel.py

echo ""
echo "✅ Accelerometer setup complete!"
