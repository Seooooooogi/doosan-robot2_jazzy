# 
#  dsr_moveit2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2025 Doosan Robotics
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# 

import os
import re
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from dsr_bringup2.controller_config import adjust_dsr_controller_yaml, parse_joints_from_urdf
from dsr_bringup2.utils import read_update_rate

# Generate robot_description and select controller YAML based on the URDF model.
def generate_robot_description_action(context, *args, **kwargs):
    dynamic_yaml = LaunchConfiguration('dynamic_yaml').perform(context).lower() == 'true'
    model = LaunchConfiguration('model').perform(context)
    color = LaunchConfiguration('color').perform(context)
    name = LaunchConfiguration('name').perform(context)
    host = LaunchConfiguration('host').perform(context)
    left_host = LaunchConfiguration('left_host').perform(context)
    right_host = LaunchConfiguration('right_host').perform(context)
    rt_host = LaunchConfiguration('rt_host').perform(context)
    port = LaunchConfiguration('port').perform(context)
    left_port = LaunchConfiguration('left_port').perform(context)
    right_port = LaunchConfiguration('right_port').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    update_rate = read_update_rate() # get update_rate from yaml

    if model == "r100_m1013_dual":
        # Build robot_description from the dual MoveIt wrapper that instantiates the composite macro.
        urdf_xml = Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("dsr_moveit_config_r100_m1013_dual"),
                "config",
                "r100_m1013_dual.urdf.xacro"
            ]),
            " ",
            "color:=", color,
            " ",
            "arm_prefix:=", "arm_",
            " ",
            "arm2_prefix:=", "arm2_",
            " ",
            "enable_ros2_control:=", "true",
            " ",
            "host:=", left_host,
            " ",
            "port:=", left_port,
            " ",
            "rt_host:=", rt_host,
            " ",
            "mode:=", mode,
            " ",
            "model:=", "m1013",
            " ",
            "update_rate:=", str(update_rate),
            " ",
            "arm2_host:=", right_host,
            " ",
            "arm2_port:=", right_port,
            " ",
            "arm2_rt_host:=", rt_host,
            " ",
            "arm2_mode:=", mode,
            " ",
            "arm2_model:=", "m1013",
            " ",
            "arm2_update_rate:=", str(update_rate),
        ])
        active_joints, passive_joints = [], []
        print(f"[DEBUG] model={model}, color={color}, name={name}, host={host}, left_host={left_host}, right_host={right_host}, rt_host={rt_host}, port={port}, left_port={left_port}, right_port={right_port}, mode={mode}, update_rate={update_rate}")
        print(f"[DEBUG] active_joints={active_joints}")
        print(f"[DEBUG] passive_joints={passive_joints}")
    else:
        # Parse URDF to extract active and passive joints
        urdf_xml, active_joints, passive_joints = parse_joints_from_urdf(model, color, name, host, rt_host, port, mode, update_rate)
        print(f"[DEBUG] model={model}, color={color}, name={name}, host={host}, rt_host={rt_host}, port={port}, mode={mode}, update_rate={update_rate}")
        print(f"[DEBUG] active_joints={active_joints}")
        print(f"[DEBUG] passive_joints={passive_joints}")

    # Decide controller YAML
    # For r100_m1013_dual, use the local ros2_controllers.yaml (no dsr_controller2)
    if model == "r100_m1013_dual":
        adjusted_yaml = os.path.join(
            get_package_share_directory(f"dsr_moveit_config_{model}"),
            "config",
            "ros2_controllers.yaml"
        )
        print(f"[INFO] Using r100_m1013_dual controller.yaml: {adjusted_yaml}")
    elif dynamic_yaml:
        original_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            "dsr_controller2.yaml"
        )
        adjusted_yaml = adjust_dsr_controller_yaml(original_yaml, active_joints, passive_joints)
        print(f"[INFO] Using dynamically generated controller.yaml: {adjusted_yaml}")
    else:
        static_yaml = os.path.join(
            get_package_share_directory("dsr_controller2"),
            "config",
            f"dsr_controller2_{model}.yaml"
        )
        if os.path.exists(static_yaml):
            adjusted_yaml = static_yaml
            print(f"[INFO] Using static controller.yaml: {adjusted_yaml}")
        else:
            adjusted_yaml = os.path.join(
                get_package_share_directory("dsr_controller2"),
                "config",
                "dsr_controller2.yaml"
            )
            print(f"[WARN] Model-specific YAML not found. Using default: {adjusted_yaml}")

    return [
        SetLaunchConfiguration('robot_description', urdf_xml),
        SetLaunchConfiguration('controller_yaml', adjusted_yaml),
    ]

def rviz_and_move_group_fn(context):
    model_value = LaunchConfiguration('model').perform(context)
    ns_value = LaunchConfiguration('name').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    package_name = f"dsr_moveit_config_{model_value}"
    package_path = FindPackageShare(package_name).perform(context)
    print("MoveIt Config Package:", package_name)
    print("Package Path:", package_path)

    # 
    pipelines = ["ompl", "chomp"]

    moveit_config = (
        MoveItConfigsBuilder(model_value, "robot_description", package_name)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(file_path="config/dsr.srdf.xacro", mappings={'gripper': LaunchConfiguration('gripper')})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=pipelines,      # List of planning pipelines to load (each loaded from config/<name>_planning.yaml)
                            default_planning_pipeline="ompl", # Name of the default planning pipeline (used if none is explicitly selected)
                            load_all= False                   # If pipelines is None: True loads all from config/default packages; False loads only from config package
                            )
        .to_moveit_configs()
    )
    
    move_group_params = [
        moveit_config.to_dict(),  # robot_description & robot_description_semantic from MoveitConfigbuilder
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
        {"publish_robot_description": True},
        {"publish_robot_description_semantic": True},
    ]
    
    rviz_params = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        moveit_config.robot_description_semantic,
        {"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns_value,
        output="screen",
        parameters=move_group_params,
    )

    rviz_base = os.path.join(get_package_share_directory(package_name), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_full_config, "--ros-args", "--log-level", "warn"],
        parameters=rviz_params,
        remappings=[
            ("goal_pose", "/goal_pose"),
            ("initialpose", "/initialpose"),
        ],
    )
    actions = [run_move_group_node]
    if gui:
        actions.append(rviz_node)
    return actions

# sets up the parameters for the controller manager node, if 'gripper' argument is setted, it additionally loads the 'gripper_controller.yaml' file
def control_node_fn(context):
    model_value = LaunchConfiguration('model').perform(context)
    if model_value == "r100_m1013_dual":
        controller_yaml = os.path.join(
            get_package_share_directory("dsr_moveit_config_r100_m1013_dual"),
            "config",
            "ros2_controllers.yaml"
        )
        print(f"[INFO] Using r100_m1013_dual controller.yaml in controller_manager: {controller_yaml}")
        params = [{"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}, controller_yaml]
    else:
        params = [{"robot_description": ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}, LaunchConfiguration('controller_yaml')]

    if LaunchConfiguration('gripper').perform(context) == 'robotiq_2f85':
        pkg_share = get_package_share_directory("dsr_controller2")
        gripper_yaml = os.path.join(pkg_share, "config", "gripper_controller.yaml")
        params.append(gripper_yaml)
        print(f"[INFO] Including gripper YAML in controller_manager: {gripper_yaml}")

    node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=params,
        output="both",
    )
    return [node]

def gripper_spawner_fn(context):
    if LaunchConfiguration('gripper').perform(context) != 'robotiq_2f85':
        return []

    return [Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "-c", "controller_manager",
        ],
        output="screen",
    )]

def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='r100_m1013_dual', description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('left_host', default_value=LaunchConfiguration('host'), description='LEFT_ROBOT_IP'),
        DeclareLaunchArgument('right_host', default_value=LaunchConfiguration('host'), description='RIGHT_ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345', description='ROBOT_PORT'),
        DeclareLaunchArgument('left_port', default_value=LaunchConfiguration('port'), description='LEFT_ROBOT_PORT'),
        DeclareLaunchArgument('right_port', default_value='12348', description='RIGHT_ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual', description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='r100_m1013_dual', description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
        DeclareLaunchArgument('gui',   default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('gz',    default_value='false', description='USE GAZEBO SIM'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('dynamic_yaml', default_value='true', description='Use dynamic controller.yaml'),
        DeclareLaunchArgument('gripper', default_value='none', description='GRIPPER (none|robotiq_2f85)'),
        DeclareLaunchArgument('left_init_on_start', default_value='true', description='Move left arm J1 to 180deg once at startup (dual only)'),
    ]

    # Build robot_description and select controller YAML
    robot_description_action = OpaqueFunction(function=generate_robot_description_action)

    # Run emulator for left arm
    left_run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": "left_m1013",
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('left_host'),
            "port": LaunchConfiguration('left_port'),
            "mode": LaunchConfiguration('mode'),
            "model": "m1013",
            "gripper": LaunchConfiguration('gripper'),
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])),
        output="screen",
    )

    # Run emulator for right arm
    right_run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[{
            "name": "right_m1013",
            "rate": 100,
            "standby": 5000,
            "command": True,
            "host": LaunchConfiguration('right_host'),
            "port": LaunchConfiguration('right_port'),
            "mode": LaunchConfiguration('mode'),
            "model": "m1013",
            "gripper": LaunchConfiguration('gripper'),
            "mobile": "none",
            "rt_host": LaunchConfiguration('rt_host'),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])),
        output="screen",
    )

    joint_states_remap = PythonExpression([
        "'/' + '", LaunchConfiguration('name'), "' + '/joint_states' if '",
        LaunchConfiguration('name'), "' != '' else '/joint_states'"
    ])

    # Run robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('joint_states', joint_states_remap),
        ],
        parameters=[{
            'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)
        }],
    )

    # Run ros2_control_node(controller_manager)
    control_node = OpaqueFunction(function=control_node_fn)

    is_r100_m1013_dual = PythonExpression(["'", LaunchConfiguration('model'), "' == 'r100_m1013_dual'"])
    controller_manager_path = PythonExpression([
        "'/' + '", LaunchConfiguration('name'), "' + '/controller_manager' if '",
        LaunchConfiguration('name'), "' != '' else '/controller_manager'"
    ])

    # Spawn joint_state_broadcaster (normal path)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
        condition=UnlessCondition(is_r100_m1013_dual),
    )

    # Spawn dsr_controller2 (skip for r100_m1013_dual to avoid interface conflicts)
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
        condition=UnlessCondition(is_r100_m1013_dual),
    )

    # Spawn dsr_moveit_controller
    dsr_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "dsr_moveit_controller",
            "-c", controller_manager_path,
            "--activate",
            "--controller-manager-timeout", "120"
        ],
    )

    # Spawners for r100_m1013_dual (no conditional on the nodes themselves)
    r100_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
    )

    r100_dual_moveit_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "dual_dsr_moveit_controller",
            "-c", controller_manager_path,
            "--activate",
            "--controller-manager-timeout", "120"
        ],
    )

    # For r100_m1013_dual: also spawn dsr_controller2 as active (same policy as single-arm path).
    r100_robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
    )

    r100_diff_drive_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "-c", controller_manager_path,
            "--controller-manager-timeout", "120"
        ],
    )

    # MoveGroup + (optional) RViz
    rviz_and_move_group = OpaqueFunction(function=rviz_and_move_group_fn)

    # A) Once joint_state_broadcaster is active, spawn dsr_controller2 (arm controller).
    delay_robot_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] joint_state_broadcaster active. Starting dsr_controller2..."),
                robot_controller_spawner
            ],
        )
    ,
        condition=UnlessCondition(is_r100_m1013_dual),
    )

    # B) After dsr_controller2 becomes active, (conditionally) spawn the gripper position controller.
    delay_gripper_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 2] dsr_controller2 active. (cond) starting gripper_position_controller..."),
                OpaqueFunction(function=gripper_spawner_fn),
            ],
        )
    )

    # C) After dsr_controller2 becomes active, spawn dsr_moveit_controller (MoveIt-compatible trajectory controller).
    delay_dsr_moveit_controller_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 3 COMPLETED] dsr_controller2 active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        )
    )

    # For r100_m1013 (single-arm legacy path): start dsr_moveit_controller right after joint_state_broadcaster
    delay_dsr_moveit_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 1 COMPLETED] joint_state_broadcaster active. Starting dsr_moveit_controller..."),
                dsr_moveit_controller_spawner,
            ],
        ),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('model'), "' == 'r100_m1013'"])),
    )

    # For r100_m1013_dual: delay spawners to ensure controller_manager is up
    delayed_r100_spawners = TimerAction(
        period=2.0,
        actions=[
            r100_joint_state_broadcaster_spawner,
            r100_robot_controller_spawner,
            r100_dual_moveit_controller_spawner,
            r100_diff_drive_controller_spawner,
        ],
        condition=IfCondition(is_r100_m1013_dual),
    )

    # For r100_m1013_dual: one-shot initial pose (left arm J1 = 180deg) before UI starts.
    left_init_motion_once = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            PythonExpression(["'/' + '", LaunchConfiguration('name'), "' + '/left/motion/move_joint'"]),
            "dsr_msgs2/srv/MoveJoint",
            "{pos: [180.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 60.0, time: 2.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 1}",
        ],
        output="log",
    )

    delay_left_init_motion_after_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_robot_controller_spawner,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[left_init_motion_once],
                )
            ],
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('model'), "' == 'r100_m1013_dual' and '",
            LaunchConfiguration('left_init_on_start'),
            "'.lower() in ['true','1']"
        ])),
    )

    # D) After dsr_moveit_controller is active, start MoveGroup (and RViz if gui=true).
    delay_rviz_after_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dsr_moveit_controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        )
    , condition=UnlessCondition(is_r100_m1013_dual))

    # For r100_m1013_dual + no init hook: start MoveGroup + RViz after dual moveit controller
    delay_rviz_after_r100_moveit_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=r100_dual_moveit_controller_spawner,
            on_exit=[
                LogInfo(msg=">> [STEP 4 COMPLETED] dual moveit controller active. Launching MoveGroup (+ RViz if gui=true)..."),
                rviz_and_move_group
            ],
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('model'), "' == 'r100_m1013_dual' and '",
            LaunchConfiguration('left_init_on_start'),
            "'.lower() not in ['true','1']"
        ])),
    )

    # For r100_m1013_dual + init hook: start MoveGroup + RViz after initial left-arm move completes
    delay_rviz_after_left_init_motion = RegisterEventHandler(
        OnProcessExit(
            target_action=left_init_motion_once,
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[
                        LogInfo(msg=">> [STEP 4 COMPLETED] left init motion done. Launching MoveGroup (+ RViz if gui=true)..."),
                        rviz_and_move_group
                    ],
                )
            ],
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('model'), "' == 'r100_m1013_dual' and '",
            LaunchConfiguration('left_init_on_start'),
            "'.lower() in ['true','1']"
        ])),
    )

    nodes = [
        LogInfo(msg=">> [START] Launching Doosan Robot Bringup with MoveIt2..."),
        robot_description_action,
        left_run_emulator_node,
        right_run_emulator_node,
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_after_joint_state,
        delay_gripper_after_robot_controller,
        delay_dsr_moveit_controller_after_robot_controller,
        delay_dsr_moveit_controller_after_joint_state,
        delayed_r100_spawners,
        delay_left_init_motion_after_robot_controller,
        delay_rviz_after_moveit_controller,
        delay_rviz_after_r100_moveit_controller,
        delay_rviz_after_left_init_motion,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
