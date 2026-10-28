#!/usr/bin/env bash
# Pulizia dei processi fantasma (Sez. 3.3): termina brain_node e bridge rimasti
# attivi da sessioni precedenti e riavvia il daemon ROS 2.
pkill -f brain_node && echo "brain_node terminato"
pkill -f metadrive_bridge && echo "metadrive_bridge terminato"
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
echo "Publisher attivi su /cmd_vel: $(ros2 topic info /cmd_vel 2>/dev/null | grep -i 'publisher count' || echo 0)"
