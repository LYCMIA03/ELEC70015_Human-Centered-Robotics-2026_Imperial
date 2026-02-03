#!/bin/bash

# SLAM 地图保存脚本
# Usage: ./save_map.sh [map_name]

MAP_DIR="$(rospack find p3at_nav)/maps"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_NAME=${1:-"map_${TIMESTAMP}"}

echo "========================================"
echo "   SLAM Map Saver - P3AT Navigation"
echo "========================================"
echo ""
echo "Saving map to: ${MAP_DIR}/${MAP_NAME}"

# 创建目录
mkdir -p ${MAP_DIR}

# 保存地图
rosrun map_server map_saver -f ${MAP_DIR}/${MAP_NAME}

echo ""
echo "Map saved successfully!"
echo "Files created:"
echo "  - ${MAP_DIR}/${MAP_NAME}.pgm"
echo "  - ${MAP_DIR}/${MAP_NAME}.yaml"
echo ""
echo "To use this map for navigation, update your launch file with:"
echo "  <arg name=\"map_file\" default=\"\$(find p3at_nav)/maps/${MAP_NAME}.yaml\"/>"
echo "========================================"
