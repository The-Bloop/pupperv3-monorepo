import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    #
    # 1. Declare a boolean launch argument 'sim' that controls whether to run the robot in the simulator or not
    #
    declare_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="False",
        description=(
            "Run `ros2 launch neural_controller launch.py sim:=True` to run the robot "
            "in the Mujoco simulator, otherwise the default value of False will run the real robot. "
            "Kept for backwards compatibility; `backend` supersedes it and wins when both are set."
        ),
    )

    #
    # 1b. `backend` picks which hardware_interface backs the controller. There are now three,
    #     all exposing the identical interface from components.xacro (60 command / 34 state),
    #     so neural_controller itself is unchanged across them:
    #       real   -> control_board_hardware_interface  (SPI to the motor drivers)
    #       mujoco -> mujoco_hardware_interface         (in-process libmujoco)
    #       isaac  -> isaac_hardware_interface          (topics to isaac_bridge.py)
    #
    #     'auto' preserves the original two-way behaviour driven by `sim`, so existing
    #     invocations keep working untouched.
    #
    #     For isaac this launch now starts isaac_bridge.py itself (gated on backend), so a
    #     single `backend:=isaac` brings up bridge + controller. The controller stack is held
    #     back by `startup_delay` so it does not configure hardware before Isaac is up. See
    #     the isaac_bridge / headless / torque_mode args below and run_isaac_bridge.sh.
    #
    declare_backend_arg = DeclareLaunchArgument(
        name="backend",
        default_value="auto",
        choices=["auto", "real", "mujoco", "isaac"],
        description=(
            "Which hardware interface to run against: real | mujoco | isaac. "
            "Default 'auto' derives it from `sim` (True -> mujoco, False -> real)."
        ),
    )

    declare_teleop_arg = DeclareLaunchArgument(
        name="teleop",
        default_value="True",
        description=(
            "Run `ros2 launch neural_controller launch.py teleop:=True` to enable teleop, "
            "otherwise the default value of False will not run teleop."
        ),
    )

    declare_bag_recorder_arg = DeclareLaunchArgument(
        name="bag_recorder",
        default_value="True",
        description=(
            "Run `ros2 launch neural_controller launch.py bag_recorder:=True` to enable bag recording, "
            "otherwise the default value of False will not run bag recorder."
        ),
    )

    # --- isaac-only args (ignored for real/mujoco) -------------------------------------
    declare_headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="false",
        choices=["true", "false"],
        description="isaac only: run isaac_bridge.py headless (no GUI window). Ignored otherwise.",
    )

    declare_torque_mode_arg = DeclareLaunchArgument(
        name="torque_mode",
        default_value="model",
        choices=["model", "passthrough"],
        description=(
            "isaac only: where the motor model runs. 'model' -> isaac_bridge.py runs it on fresh "
            "state at the physics rate (mirrors MuJoCo); 'passthrough' -> the plugin already ran "
            "it and the bridge applies tau verbatim. MUST match torque_source in "
            "pupper_v3_isaac.ros2_control.xacro."
        ),
    )

    declare_startup_delay_arg = DeclareLaunchArgument(
        name="startup_delay",
        default_value="25.0",
        description=(
            "isaac only: seconds to hold the controller stack back while Isaac boots. Applied "
            "only when backend resolves to isaac; other backends start with zero delay."
        ),
    )

    # --- T-Monitor: capture applied joint torque for MuJoCo-vs-Isaac comparison -----------
    declare_t_monitor_arg = DeclareLaunchArgument(
        name="t_monitor",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Capture the torque actually applied to each joint (the actuator-model output) at "
            "physics rate and publish it on /applied_torque. The correct per-sim emitter is "
            "selected automatically: mujoco -> the C++ plugin, isaac -> isaac_bridge.py. See "
            "t_monitor_output for where it is stored."
        ),
    )

    declare_t_monitor_output_arg = DeclareLaunchArgument(
        name="t_monitor_output",
        default_value="both",
        choices=["csv", "rosbag", "both"],
        description=(
            "T-Monitor sink(s), only used when t_monitor:=true. 'csv' -> torque_recorder writes "
            "results/t_monitor/<backend>_<ts>.csv; 'rosbag' -> ros2 bag record /applied_torque; "
            "'both' -> both. The topic is always published when t_monitor is on."
        ),
    )

    #
    # 2. Construct the path to the URDF file using IfElseSubstitution

    # Resolve backend once: 'auto' falls back to the legacy `sim` boolean.
    resolved_backend = PythonExpression(
        [
            "('",
            LaunchConfiguration("backend"),
            "') if ('",
            LaunchConfiguration("backend"),
            "' != 'auto') else ('mujoco' if (",
            LaunchConfiguration("sim"),
            ") else 'real')",
        ]
    )

    # use_sim_time ONLY on the isaac backend. The isaac bridge runs Isaac below realtime on this
    # CPU, so it publishes /clock and the controller stack follows sim-time -- otherwise the
    # wall-clock 520 Hz controller outruns the physics and the feet slip. MuJoCo is wall-clock
    # paced at 1x realtime and publishes NO /clock, so it MUST stay on wall-clock (use_sim_time
    # false); real hardware likewise. Gating on isaac keeps the controller_manager from blocking
    # forever waiting for a /clock that only the bridge publishes.
    use_sim_time = {
        "use_sim_time": ParameterValue(
            PythonExpression(["'", resolved_backend, "' == 'isaac'"]), value_type=bool
        )
    }

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("pupper_v3_description"),
            "description",
            PythonExpression(
                [
                    "{'real': 'pupper_v3.urdf.xacro',",
                    " 'mujoco': 'pupper_v3_mujoco.urdf.xacro',",
                    " 'isaac': 'pupper_v3_isaac.urdf.xacro'}['",
                    resolved_backend,
                    "']",
                ]
            ),
        ]
    )

    #
    # 3. Create the robot_description using xacro
    #
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file])
    # value_type=str is required, not cosmetic: launch_ros otherwise runs yaml.safe_load on
    # the URDF to infer the type, and any ": " sequence inside an XML comment makes YAML try
    # to read it as a mapping and fail with "Unable to parse the value of parameter
    # robot_description as yaml". That makes the URDF's prose comments load-bearing, which is
    # a trap for whoever edits a xacro next.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    #
    # 4. Robot State Publisher
    #
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time],
    )

    #
    # 5. Common controller parameters
    #
    node_parameters = ParameterFile(
        PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "config.yaml"]),
        allow_substs=True,
    )

    #
    # 6. Nodes from your original launch files
    #
    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     parameters=[node_parameters],
    #     output="both",
    # )
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        parameters=[node_parameters],
        output="both",
        name="joy_linux_node",
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[node_parameters],
        output="both",
        condition=IfCondition(LaunchConfiguration("teleop")),
        remappings=[("cmd_vel", "teleop_cmd_vel")],
    )

    # T-Monitor enable flag, passed as env to the sim processes. MuJoCo's plugin reads
    # getenv("T_MONITOR"); isaac_bridge.py reads it too (mirrors the CLI --t-monitor). "1"/"0"
    # so a plain string compare works on both sides.
    t_monitor_env = PythonExpression(
        ["'1' if '", LaunchConfiguration("t_monitor"), "' == 'true' else '0'"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[node_parameters, use_sim_time],
        output="both",
        additional_env={"T_MONITOR": t_monitor_env},
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    three_legged_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller_three_legged",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    # Forward command controllers for animation system (inactive by default)
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    forward_kp_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_kp_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    forward_kd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_kd_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--inactive",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="both",
    )

    joy_util_node = Node(
        package="joy_utils",
        executable="estop_controller",
        parameters=[node_parameters],
        output="both",
        name="joy_util_node",
    )

    # Real hardware only. Was UnlessCondition(sim), which silently meant "not mujoco" and so
    # would have started a camera node under isaac too; key it off the resolved backend
    # instead, since neither sim backend has a camera.
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="both",
        parameters=[node_parameters],
        condition=IfCondition(PythonExpression(["'", resolved_backend, "' == 'real'"])),
    )

    cmd_vel_mux_node = Node(
        package="cmd_vel_mux",
        executable="cmd_vel_mux_node",
        parameters=[node_parameters],
        output="both",
    )

    bag_recorder_node = Node(
        package="bag_recorder",
        executable="bag_recorder_node",
        name="bag_recorder",
        output="both",
        parameters=[node_parameters],
        condition=IfCondition(LaunchConfiguration("bag_recorder")),
    )

    imu_to_tf_node = Node(
        package="imu_to_tf",
        executable="imu_to_tf_node",
        output="both",
    )

    animation_controller_py_node = Node(
        package="animation_controller_py",
        executable="animation_controller_py",
        name="animation_controller_py",
        parameters=[node_parameters],
        output="both",
    )

    # Provides a throttled version of /joint_states to reduce CPU usage in animation_controller_py
    joint_state_throttler = Node(
        package="topic_tools",
        executable="throttle",
        name="joint_state_throttler",
        parameters=[node_parameters],
        arguments=["messages"],
        output="both",
    )

    hailo_detection_node = Node(
        package="hailo",
        executable="hailo_detection",
        output="both",
        parameters=[node_parameters],
    )

    person_following_node = Node(
        package="person_follower",
        executable="person_follower_node",
        output="both",
        parameters=[node_parameters],
    )

    #
    # 6b. Isaac bridge (isaac backend only). isaac_bridge.py runs under Isaac's own python
    #     3.11 via run_isaac_bridge.sh, so it is an ExecuteProcess, not a Node (ros2 run would
    #     use jazzy's 3.12 and fail to import isaacsim). Two variants so no empty '--headless'
    #     arg is ever emitted: a bare '' would reach argparse and be rejected.
    #
    bridge_script = PathJoinSubstitution(
        [
            FindPackageShare("pupper_v3_description"),
            "description",
            "isaac_sim",
            "run_isaac_bridge.sh",
        ]
    )
    bridge_tail = ["--torque-mode", LaunchConfiguration("torque_mode")]
    is_isaac = PythonExpression(["'", resolved_backend, "' == 'isaac'"])
    is_isaac_headless = PythonExpression(
        ["'", resolved_backend, "' == 'isaac' and '", LaunchConfiguration("headless"), "' == 'true'"]
    )
    is_isaac_gui = PythonExpression(
        ["'", resolved_backend, "' == 'isaac' and '", LaunchConfiguration("headless"), "' != 'true'"]
    )
    # T-Monitor is passed to the bridge via env (not a CLI flag): a conditional flag would need
    # 4 bridge variants, and an empty '' arg would trip argparse -- the same trap the headless
    # split already avoids. isaac_bridge.py reads T_MONITOR from the environment.
    isaac_bridge_headless = ExecuteProcess(
        cmd=["bash", bridge_script, "--headless", *bridge_tail],
        condition=IfCondition(is_isaac_headless),
        output="both",
        name="isaac_bridge",
        additional_env={"T_MONITOR": t_monitor_env},
    )
    isaac_bridge_gui = ExecuteProcess(
        cmd=["bash", bridge_script, *bridge_tail],
        condition=IfCondition(is_isaac_gui),
        output="both",
        name="isaac_bridge",
        additional_env={"T_MONITOR": t_monitor_env},
    )

    # T-Monitor sinks (only when t_monitor:=true). resolved_backend names the run; each launch
    # records exactly one sim (they never run together), so the two sims are compared by running
    # each in turn and diffing the two files/bags on sim_time.
    t_monitor_on = PythonExpression(["'", LaunchConfiguration("t_monitor"), "' == 'true'"])
    want_csv = PythonExpression(
        [t_monitor_on, " and '", LaunchConfiguration("t_monitor_output"), "' in ('csv', 'both')"]
    )
    want_bag = PythonExpression(
        [t_monitor_on, " and '", LaunchConfiguration("t_monitor_output"), "' in ('rosbag', 'both')"]
    )
    torque_recorder_node = Node(
        package="neural_controller",
        executable="torque_recorder.py",
        name="torque_recorder",
        output="both",
        arguments=["--backend", resolved_backend, "--out-dir", "results/t_monitor"],
        condition=IfCondition(want_csv),
    )
    launch_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    torque_bag = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "/applied_torque", "-o",
            # single concatenated argument: results/t_monitor/<backend>_<ts>_bag
            [TextSubstitution(text="results/t_monitor/"), resolved_backend,
             TextSubstitution(text=f"_{launch_ts}_bag")],
        ],
        condition=IfCondition(want_bag),
        output="both",
        name="torque_bag",
    )

    # Hold the controller stack back only when isaac: it must not configure hardware before
    # the bridge has Isaac up. Zero for real/mujoco, so their timing is unchanged.
    controller_delay = PythonExpression(
        [LaunchConfiguration("startup_delay"), " if '", resolved_backend, "' == 'isaac' else 0.0"]
    )

    #
    # 7. Put them all together
    #
    nodes = [
        robot_state_publisher,
        control_node,
        robot_controller_spawner,
        three_legged_robot_controller_spawner,
        # Forward command controllers for animation
        forward_position_controller_spawner,
        forward_kp_controller_spawner,
        forward_kd_controller_spawner,
        joint_state_broadcaster_spawner,
        # Comment/uncomment as needed:
        imu_sensor_broadcaster_spawner,
        foxglove_bridge,
        joy_util_node,
        # joy_node,
        joy_linux_node,
        teleop_twist_joy_node,
        camera_node,
        cmd_vel_mux_node,
        bag_recorder_node,
        imu_to_tf_node,
        animation_controller_py_node,
        joint_state_throttler,
        # Detection
        hailo_detection_node,
        # Person following
        person_following_node,
        # T-Monitor sinks (self-gated on t_monitor / t_monitor_output)
        torque_recorder_node,
        torque_bag,
    ]

    #
    # 8. Return the LaunchDescription with the declared arg + all nodes
    #
    return LaunchDescription(
        [
            declare_sim_arg,
            declare_backend_arg,
            declare_teleop_arg,
            declare_bag_recorder_arg,
            declare_headless_arg,
            declare_torque_mode_arg,
            declare_startup_delay_arg,
            declare_t_monitor_arg,
            declare_t_monitor_output_arg,
            # bridge starts immediately (isaac only); controller stack waits controller_delay.
            isaac_bridge_headless,
            isaac_bridge_gui,
            TimerAction(period=controller_delay, actions=nodes),
        ]
    )
