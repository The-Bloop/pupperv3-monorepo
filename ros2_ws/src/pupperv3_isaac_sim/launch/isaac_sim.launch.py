"""Run neural_controller against Isaac Sim.

Thin wrapper: the bridge startup + controller stack now live in neural_controller's
launch.py (gated on backend:=isaac), so this just forwards into it. Kept so the documented
entry point keeps working:

    ros2 launch pupper_isaac_sim isaac_sim.launch.py
    ros2 launch pupper_isaac_sim isaac_sim.launch.py torque_mode:=passthrough headless:=false teleop:=true

Equivalent to:

    ros2 launch neural_controller launch.py backend:=isaac [torque_mode:=... headless:=... teleop:=...]

Must run inside the pupper-jazzy distrobox: Isaac needs its bundled ROS2 (see
run_isaac_bridge.sh), and jazzy is only installed there.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_torque_mode = DeclareLaunchArgument(
        name="torque_mode",
        default_value="model",
        choices=["model", "passthrough"],
        description=(
            "Where the motor model runs. 'model': isaac_bridge.py runs it on fresh state at "
            "the physics rate, mirroring MuJoCo's update_ctrl_ before every mj_step. "
            "'passthrough': the plugin ran it already and the bridge applies tau verbatim. "
            "MUST match the torque_source param in pupper_v3_isaac.ros2_control.xacro."
        ),
    )

    declare_headless = DeclareLaunchArgument(
        name="headless", default_value="false", choices=["true", "false"]
    )

    declare_startup_delay = DeclareLaunchArgument(
        name="startup_delay",
        default_value="25.0",
        description="Seconds to wait for Isaac to boot before starting the controller stack.",
    )

    declare_teleop = DeclareLaunchArgument(
        name="teleop",
        default_value="false",
        choices=["true", "false"],
        description="Run teleop_twist_joy for joystick driving.",
    )

    neural_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("neural_controller"), "launch", "launch.py"]
            )
        ),
        launch_arguments={
            "backend": "isaac",
            "torque_mode": LaunchConfiguration("torque_mode"),
            "headless": LaunchConfiguration("headless"),
            "startup_delay": LaunchConfiguration("startup_delay"),
            "teleop": LaunchConfiguration("teleop"),
        }.items(),
    )

    return LaunchDescription(
        [
            declare_torque_mode,
            declare_headless,
            declare_startup_delay,
            declare_teleop,
            neural_controller,
        ]
    )
