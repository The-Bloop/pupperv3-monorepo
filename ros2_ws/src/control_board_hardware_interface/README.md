# control_board_hardware_interface

This repository provides a ros2_control hardware interface for the Pupper v3 control board.

## Prerequisites
- [Pupper v3 control board](https://oshwlab.com/gabrael_3548/pupper-v3-control-power-boards)
- Raspberry Pi 5 running Ubuntu 24.04 and ROS 2 Jazzy ([flash the custom image](https://pupper-v3-documentation.readthedocs.io/en/latest/guide/software_installation.html))
- Recommended: configure ros2_control for realtime operation as described [here](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#determinism)
- Create a workspace `/home/pi/ros2_ws`

## How to configure
- An example URDF is contained in `test_state_publisher.urdf.xacro`.
```
<ros2_control name="control_board_hardware_interface" type="system">
    <hardware>
        <plugin>control_board_hardware_interface/ControlBoardHardwareInterface</plugin>
        <param name="imu_mounting_deg.yaw">0</param>
        <param name="imu_mounting_deg.pitch">0</param>
        <param name="imu_mounting_deg.roll">0</param>
    </hardware>

    <joint name="joint_1">
        <param name="can_channel">1</param>
        <param name="can_id">1</param>
        <param name="can_protocol">cheetah</param>

        <param name="position_scale">95.5</param>
        <param name="velocity_scale">30.0</param>
        <param name="effort_scale">18.0</param>
        <param name="kp_scale">500.0</param>
        <param name="kd_scale">5.0</param>
        <param name="axis_direction">-1</param>
        <param name="position_offset">0.0</param>

        <param name="position_min">-1.0</param>
        <param name="position_max">1.0</param>
        <param name="velocity_max">1.0</param>
        <param name="effort_max">1.0</param>
        <param name="kp_max">1.0</param>
        <param name="kd_max">1.0</param>

        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="effort"/>
        <command_interface name="kp"/>
        <command_interface name="kd"/>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>

    <sensor name="imu_sensor">
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <state_interface name="angular_velocity.x"/>
        <state_interface name="angular_velocity.y"/>
        <state_interface name="angular_velocity.z"/>
        <state_interface name="linear_acceleration.x"/>
        <state_interface name="linear_acceleration.y"/>
        <state_interface name="linear_acceleration.z"/>
    </sensor>
</ros2_control>
```
### Explanation of parameters
- `imu_mounting_deg.*`: Mounting angles for the IMU relative the the IMU link, measured in degrees
- `can_channel`: Which CANbus channel the actuator is connected to
- `can_id`: CANbus ID of the actuator (must be 1 or greater)
