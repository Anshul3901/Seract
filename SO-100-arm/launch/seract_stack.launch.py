from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    run_scanning_arg = DeclareLaunchArgument(
        'run_scanning',
        default_value='true',
        description='Start scanning_motion node'
    )
    run_vision_arg = DeclareLaunchArgument(
        'run_vision',
        default_value='true',
        description='Start vision_inspection node'
    )
    run_policy_bridge_arg = DeclareLaunchArgument(
        'run_policy_bridge',
        default_value='false',
        description='Start seract_policy_control policy_bridge node'
    )
    run_web_ui_arg = DeclareLaunchArgument(
        'run_web_ui',
        default_value='true',
        description='Start web UI (bridge node and Flask server)'
    )

    # --- so_100_arm hardware.launch.py ---
    so100_share = get_package_share_directory('so_100_arm')
    hardware_launch_path = os.path.join(so100_share, 'launch', 'hardware.launch.py')

    so100_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch_path)
    )

    usb_camera = Node(
        package='seract_pipeline',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[
            {'video_device': '/dev/video_opencv_side'},
            {'image_topic': '/camera2/color/image_raw'},
        ],
    )

    obj_bbox = Node(
    package='obj_bbox',
    executable='obj_bbox_node',
    name='obj_bbox_node',
    output='screen',
    parameters=[{
        'image_topic': '/camera2/color/image_raw',

        # empty-table reference
        'reference_image_path': '/home/gixadmin/empty_table.png',
        'save_reference_on_start': False,
        'reference_save_path': '/tmp/empty_table_reference.png',

        # MobileSAM
        'model_name': 'mobile_sam.pt',
        'sam_enabled': True,

        # ROI of table
        'roi_x': 0,
        'roi_y': 0,
        'roi_w': 0,
        'roi_h': 0,

        # foreground subtraction sensitivity
        'diff_threshold': 10,
        'blur_kernel': 5,
        'use_hsv': True,

        # allow smaller detections
        'min_contour_area_px': 150.0,
        'min_bbox_w_px': 15.0,
        'min_bbox_h_px': 15.0,

        # allow small objects
        'min_area_ratio': 0.0005,
        'max_area_ratio': 0.95,

        # reduce morphology so objects don't disappear
        'morph_open_kernel': 3,
        'morph_close_kernel': 3,

        # optional table mask
        'table_mask_path': '',
    }],
)
#     obj_bbox = Node(
#         package='obj_bbox',
#         executable='yolo_bbox_node',
#         name='yolo_bbox',
#         output='screen',
#         parameters=[{
#             'image_topic': '/camera2/color/image_raw',

#             # YOLO model path
#             'model_path': '/home/gixadmin/seract_orch_ws/src/yolo.pt',

#             # inference settings
#             'device': 'cpu',          # change to 'cuda' if GPU works
#             'conf_threshold': 0.10,
#             'iou_threshold': 0.45,
#             'imgsz': 640,
#             'max_detections': 10,

#             # ROI (set if you want only the table region)
#             'roi_x': 0,
#             'roi_y': 0,
#             'roi_w': 0,
#             'roi_h': 0,

#             # allow small detections
#             'min_bbox_w_px': 8.0,
#             'min_bbox_h_px': 8.0,
#             'min_area_ratio': 0.00005,
#             'max_area_ratio': 0.98,
#         }],
#     )

    # ✅ Decision action server (DecisionAct)
    decision_action_server = Node(
        package='so101_motion',
        executable='decision_movement_action_server',
        name='decision_movement_action_server',
        output='screen',
        # parameters=[  # (optional) add params if your server uses them
        #     {'decision_path': '/home/gixadmin/seract_orch_ws/src/so101_motion/so101_motion/decision.json'},
        # ],
    )

    scan_motion = Node(
        condition=IfCondition(LaunchConfiguration('run_scanning')),
        package='so101_motion',
        executable='scan_motion',
        name='scan_motion',
        output='screen',
    )

    vision_inspection = Node(
        condition=IfCondition(LaunchConfiguration('run_vision')),
        package='vision_inspection',
        executable='vision_inspection_node',
        name='vision_inspection_node',
        output='screen',
    )

    policy_bridge = Node(
        condition=IfCondition(LaunchConfiguration('run_policy_bridge')),
        package='seract_policy_control',
        executable='policy_bridge',
        name='policy_bridge',
        output='screen',
    )

    # Web UI Bridge Node (ROS2)
    web_ui_bridge = Node(
        condition=IfCondition(LaunchConfiguration('run_web_ui')),
        package='web_ui',
        executable='web_ui_bridge',
        name='web_ui_bridge',
        output='screen',
    )

    # Web UI Flask Server (separate process)
    web_ui_server = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('run_web_ui')),
        cmd=['ros2', 'run', 'web_ui', 'web_ui_server'],
        output='screen',
    )

    return LaunchDescription([
        run_scanning_arg,
        run_vision_arg,
        run_policy_bridge_arg,
        run_web_ui_arg,
        so100_hardware,
        usb_camera,
        obj_bbox,
        decision_action_server,
        scan_motion,
        vision_inspection,
        policy_bridge,
        web_ui_bridge,
        web_ui_server,
    ])