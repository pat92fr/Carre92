#!/bin/sh
sleep 10
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'

