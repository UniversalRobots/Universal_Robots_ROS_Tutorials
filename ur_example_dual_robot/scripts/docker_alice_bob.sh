#!/bin/bash
cleanup() {
  docker kill alice
  docker kill bob
  exit 0
}

docker run --rm \
  -d \
  -v $(rospack find ur_example_dual_robot)/resources/ursim/urcaps:/urcaps \
  -v $(rospack find ur_example_dual_robot)/resources/ursim/programs_alice:/ursim/programs \
  --name alice \
  -e ROBOT_MODEL=UR10e \
  --ip 172.17.0.2 \
  universalrobots/ursim_e-series

docker run --rm \
  -d \
  -v $(rospack find ur_example_dual_robot)/resources/ursim/urcaps:/urcaps \
  -v $(rospack find ur_example_dual_robot)/resources/ursim/programs_bob:/ursim/programs \
  --name bob \
  -e ROBOT_MODEL=UR3e \
  --ip 172.17.0.3 \
  universalrobots/ursim_e-series


trap cleanup INT

echo "Press [CTRL+C] to kill alice and bob"
while :
do
  sleep 1
done
