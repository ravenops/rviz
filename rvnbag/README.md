# Raven[Ops] ROS Bag Player

*Author:* Iain Brookshaw, Delaney Gillilam

Copytright (c) 2018, Raven Ops Inc. All Rights Reserved

## Summary
We require more control over bag publication than is usually provided by the `rosbag` tool suite
This player responds to dbus signals to generate "frames" -- all messages bagged within a defined
time interval