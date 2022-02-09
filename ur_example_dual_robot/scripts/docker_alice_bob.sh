#!/bin/bash
docker-compose -p ursim_alice_bob -f $(rospack find ur_example_dual_robot)/resources/docker-compose.yml up
