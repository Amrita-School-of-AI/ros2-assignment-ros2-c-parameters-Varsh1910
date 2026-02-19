#!/bin/bash

# 1. Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# 2. Source the local workspace (if it was built)
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "❌ Error: install/setup.bash not found. Did the build fail?"
  exit 1
fi

# 3. Test with default parameters
echo "Testing with default parameters..."
timeout 5s ros2 run ros2_parameters param_node > output_default.log 2>&1 &
NODE_PID=$!
sleep 3

# Check parameters via CLI
PARAMS=$(ros2 param list /param_node 2>/dev/null)
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

if echo "$PARAMS" | grep -q "robot_name" && echo "$PARAMS" | grep -q "max_speed" && echo "$PARAMS" | grep -q "enabled"; then
  echo "✅ All parameters declared correctly"
else
  echo "❌ Parameters not declared correctly"
  echo "Found parameters: $PARAMS"
  exit 1
fi

# 4. Test with custom parameters
echo "Testing with custom parameters..."
timeout 5s ros2 run ros2_parameters param_node --ros-args -p robot_name:=TestBot -p max_speed:=3.0 -p enabled:=false > output_custom.log 2>&1 &
NODE_PID=$!
sleep 3

kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

# 5. Check output
echo "Output with custom parameters:"
cat output_custom.log

if grep -q "Robot:" output_custom.log && grep -q "Max Speed:" output_custom.log && grep -q "Enabled:" output_custom.log; then
  echo "✅ Node correctly logs parameter values"
  exit 0
else
  echo "❌ Node does not correctly log parameter values"
  exit 1
fi
