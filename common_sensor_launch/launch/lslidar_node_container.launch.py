import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
import yaml


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result
    
    # Pointcloud preprocessor parameters
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_correction_node_param_path").perform(context),
        allow_substs=True,
    )

    nodes = []

    nodes.append(
        ComposableNode(
            package="glog_component",
            plugin="GlogComponent",
            name="glog_component",
        )
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "pointcloud_raw"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_raw"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_corrector_node_param],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Ring Outlier Filter is the last component in the pipeline, so control the output frame here
    if LaunchConfiguration("output_as_sensor_frame").perform(context):
        ring_outlier_filter_parameters = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        ring_outlier_filter_parameters = {
            "output_frame": ""
        }  # keep the output frame as the input frame
    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "pointcloud_before_sync"),
            ],
            parameters=[ring_outlier_filter_parameters],
            extra_arguments=[
                {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
            ],
        )
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    return [container]

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    common_sensor_share_dir = get_package_share_directory("common_sensor_launch")
    
    add_launch_arg("msop_port", "2368", "msop port number")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("setup_sensor", "True", "configure sensor")
    add_launch_arg("sensor_ip", "192.168.100.11", "device ip address")
    add_launch_arg("host_ip", "255.255.255.0", "host ip address")
    add_launch_arg("scan_phase", "0.0")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("min_range", "0.3", "minimum view range for Velodyne sensors")
    add_launch_arg("max_range", "300.0", "maximum view range for Velodyne sensors")
    add_launch_arg("cloud_min_angle", "0", "minimum view angle setting on device")
    add_launch_arg("cloud_max_angle", "120", "maximum view angle setting on device")
    add_launch_arg("data_port", "2368", "device data port number")
    add_launch_arg("gnss_port", "2380", "device gnss port number")
    add_launch_arg("packet_mtu_size", "1500", "packet mtu size")
    add_launch_arg("rotation_speed", "600", "rotational frequency")
    add_launch_arg("dual_return_distance_threshold", "0.1", "dual return distance threshold")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("pcl_type", "PointXYZI", "point cloud type")
    add_launch_arg("add_multicast", "False", "add multicast")
    add_launch_arg("group_ip", "224.1.1.1", "group ip")
    add_launch_arg("angle_disable_max", "0", "maximum angle to disable")
    add_launch_arg("angle_disable_min", "0", "minimum angle to disable")
    add_launch_arg("horizontal_angle_resolution", "0.2", "horizontal angle resolution")
    add_launch_arg("use_time_service", "True", "use time service")
    add_launch_arg("echo_num", "0", "echo number")
    add_launch_arg("publish_scan", "True", "publish scan")
    add_launch_arg("channel_num", "8", "channel number")
    add_launch_arg("difop_port", "2369", "difop port number")
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("lidar_container_name", "lslidar_node_container")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg(
        "distortion_correction_node_param_path",
        os.path.join(
            common_sensor_share_dir,
            "config",
            "distortion_corrector_node.param.yaml",
        ),
        description="path to parameter file of distortion correction node",
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
