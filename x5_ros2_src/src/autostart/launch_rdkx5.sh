#!/usr/bin/env bash
set -euo pipefail
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
WORKSPACE_DIR=$(cd "$(dirname "$0")/.." && pwd)
if [ -f "/opt/ros/humble/setup.bash" ]; then source "/opt/ros/humble/setup.bash"; fi
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then source "$WORKSPACE_DIR/install/setup.bash"; fi
ARGS=""
if [ -n "${USART_PORT_NAME:-}" ]; then ARGS="$ARGS usart_port_name:=$USART_PORT_NAME"; fi
if [ -n "${SERIAL_BAUD_RATE:-}" ]; then ARGS="$ARGS serial_baud_rate:=$SERIAL_BAUD_RATE"; fi
exec ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py $ARGS