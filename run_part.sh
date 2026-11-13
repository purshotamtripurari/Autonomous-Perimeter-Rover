#!/usr/bin/env bash
# ==============================================================================
# Autonomous Perimeter Rover - Multi-Part Terminal Launcher
# Usage:
#   ./run_part.sh 1   -> Part 1: Benchmark Arena Baseline (4 corners, SLAM)
#   ./run_part.sh 2   -> Part 2: Static Intruder Yard (Halt & Hold on Human)
#   ./run_part.sh 3   -> Part 3: Dynamic Obstacle Yard (Yield & Pass Maneuver)
# ==============================================================================

PART=${1:-3}
WORKSPACE_DIR="/home/purshotam/autonomous-perimeter-rover"

echo "======================================================================"
echo "🚀 AUTONOMOUS PERIMETER ROVER LAUNCHER - PART ${PART}"
echo "======================================================================"

# Quick cleanup of any lingering ROS 2 simulation processes
echo "🧹 Performing quick cleanup of old background nodes..."
killall -9 gz sim rviz2 component_container_isolated surveillance_node patrol_node dynamic_obstacle_node web_controller_node 2>/dev/null
pkill -f "ros2 run perimeter_rover" 2>/dev/null || true
pkill -f "web_controller_node" 2>/dev/null || true
pkill -f "patrol_node" 2>/dev/null || true
pkill -f "surveillance_node" 2>/dev/null || true
pkill -f "dynamic_obstacle_node" 2>/dev/null || true
fuser -k 5000/tcp 2>/dev/null || true
sleep 1

SETUP_CMD="export DISPLAY=\${DISPLAY:-:0} && source /opt/ros/jazzy/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && export TURTLEBOT3_MODEL=waffle"

if [ "$PART" == "1" ]; then
    echo "▶️ Launching PART 1: Benchmark Arena Baseline..."
    gnome-terminal --title="[Part 1] Sim & Nav2" -- bash -c "${SETUP_CMD}; ros2 launch nav2_bringup tb3_simulation_launch.py world:=${WORKSPACE_DIR}/src/perimeter_rover/worlds/perimeter_arena.sdf slam:=True x_pose:=-2.0 y_pose:=-0.5 yaw:=0.0; exec bash" &
    gnome-terminal --title="[Part 1] Camera Bridge" -- bash -c "sleep 4; ${SETUP_CMD}; ros2 run ros_gz_bridge parameter_bridge --ros-args --params-file ${WORKSPACE_DIR}/src/perimeter_rover/config/camera_bridge.yaml; exec bash" &
    gnome-terminal --title="[Part 1] Surveillance Node" -- bash -c "sleep 6; ${SETUP_CMD}; ros2 run perimeter_rover surveillance_node --ros-args -p arena_type:=benchmark -p avoidance_mode:=false; exec bash" &
    gnome-terminal --title="[Part 1] Patrol Node" -- bash -c "sleep 8; ${SETUP_CMD}; ros2 run perimeter_rover patrol_node --ros-args -p arena_type:=benchmark -p avoidance_mode:=false; exec bash" &
    gnome-terminal --title="[Web Center] Web Controller & Teleop" -- bash -c "sleep 5; ${SETUP_CMD}; ros2 run perimeter_rover web_controller_node; exec bash" &

elif [ "$PART" == "2" ]; then
    echo "▶️ Launching PART 2: Outdoor Yard - Static Intruder Protocol (Halt & Hold)..."
    gnome-terminal --title="[Part 2] Sim & Nav2" -- bash -c "${SETUP_CMD}; ros2 launch perimeter_rover outdoor_yard_simulation.launch.py; exec bash" &
    gnome-terminal --title="[Part 2] Web Controller & Teleop" -- bash -c "sleep 4; ${SETUP_CMD}; ros2 run perimeter_rover web_controller_node; exec bash" &
    gnome-terminal --title="[Part 2] Surveillance Sentinel" -- bash -c "sleep 5; ${SETUP_CMD}; ros2 run perimeter_rover surveillance_node --ros-args -p arena_type:=yard -p avoidance_mode:=false; exec bash" &
    gnome-terminal --title="[Part 2] Patrol Navigator" -- bash -c "sleep 7; ${SETUP_CMD}; ros2 run perimeter_rover patrol_node --ros-args -p arena_type:=yard -p avoidance_mode:=false; exec bash" &

elif [ "$PART" == "3" ]; then
    echo "▶️ Launching PART 3: Dynamic Obstacle Avoidance (Yield, Move Aside & Resume)..."
    gnome-terminal --title="[Part 3] Sim & Nav2" -- bash -c "${SETUP_CMD}; ros2 launch perimeter_rover outdoor_yard_simulation.launch.py; exec bash" &
    gnome-terminal --title="[Part 3] Web Controller & Teleop" -- bash -c "sleep 4; ${SETUP_CMD}; ros2 run perimeter_rover web_controller_node; exec bash" &
    gnome-terminal --title="[Part 3] Surveillance Sentinel" -- bash -c "sleep 5; ${SETUP_CMD}; ros2 run perimeter_rover surveillance_node --ros-args -p arena_type:=yard -p avoidance_mode:=true; exec bash" &
    gnome-terminal --title="[Part 3] Dynamic Rover" -- bash -c "sleep 6; ${SETUP_CMD}; ros2 run perimeter_rover dynamic_obstacle_node; exec bash" &
    gnome-terminal --title="[Part 3] Patrol Navigator" -- bash -c "sleep 8; ${SETUP_CMD}; ros2 run perimeter_rover patrol_node --ros-args -p arena_type:=yard -p avoidance_mode:=true; exec bash" &

else
    echo "❌ Unknown part '${PART}'. Please choose 1, 2, or 3."
    echo "Usage: ./run_part.sh [1|2|3]"
    exit 1
fi

echo "======================================================================"
echo "✅ ALL SYSTEMS & TERMINALS OPENED SUCCESSFULLY!"
echo "🌐 MISSION CONTROL WEB DASHBOARD: http://localhost:5000"
echo "   - Dual Mode: Autonomous Patrol vs. Manual Joystick"
echo "   - Emergency Stop & Total System Reset"
echo "   - Real-time Velocity Telemetry (Linear & Angular)"
echo "   - Live Detection Events Table (Date, Time, Distance)"
echo "======================================================================"
