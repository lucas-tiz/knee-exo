# Knee Exoskeleton Control
High/low-level control of a pneumatically-actuated knee exoskeleton.

## ROS nodes
*exo_control*:
* ```fsr_gait_est_control.py``` estimates gait phase based on heel-strike detection via force-sensing resistors (FSRs), and activates knee assistance torque at appropriate time in gait phase cycle.
* ```pres_time_sequence.py``` just runs a simple pressure vs. time sequence for testing/debugging.

*mcu_uart*:
* ```uart_comm.cpp``` handles communication between high-level controller and MSP microcontroller.
* ```config_vals.py``` updates control parameters on microcontroller from YAML config file.

## ROS launch
Launch files run everything locally on PC or with controller on Raspberry Pi (integrated into exo control backpack). Another launch file is used to update microcontroller control params on the fly with system already running.

## Microcontroller workspace
*mcu_ws* contains microcontroller code for sensing & control.