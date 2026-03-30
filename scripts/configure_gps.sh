#!/bin/bash
# GPS CONFIGURATION FOR JETSON NANO

echo "📍 Configuring GPS Module for USV"
echo "================================"

# Find GPS device
echo "🔍 Finding GPS device..."
GPS_DEV=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1)

if [ -z "$GPS_DEV" ]; then
    echo "❌ No GPS device found!"
    echo "Check connections: ls /dev/ttyUSB* or /dev/tTYACM*"
    exit 1
fi

echo "✅ Found GPS at: $GPS_DEV"

# Set permissions
echo "🔧 Setting permissions..."
sudo chmod 666 $GPS_DEV

# Test GPS connection
echo "📡 Testing GPS connection..."
timeout 5 cat $GPS_DEV | head -10

# Configure GPS for optimal settings (NEO-6M/7M/8M)
echo "⚙️ Configuring GPS settings..."
echo -e "\$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" > $GPS_DEV
echo -e "\$PMTK220,1000*1F\r\n" > $GPS_DEV  # 1Hz update rate

echo "✅ GPS configured!"
echo ""
echo "📊 To test GPS output:"
echo "   ros2 topic echo /gps/fix"
