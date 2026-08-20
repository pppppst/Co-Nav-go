import argparse
import torch


def get_args():
    parser = argparse.ArgumentParser(
        description='Multi-Agent-Semantic-Exploration')

    # General Arguments
    parser.add_argument('--seed', type=int, default=1,
                        help='random seed (default: 1)')
    # Logging, loading models, visualization
    parser.add_argument('--log_interval', type=int, default=10,
                        help="""log interval, one log per n updates
                                (default: 10) """)
    parser.add_argument('-d', '--dump_location', type=str, default="./tmp",
                        help='path to dump models and log (default: ./tmp/)')
    parser.add_argument('--exp_name', type=str, default="exp1",
                        help='experiment name (default: exp1)')
    parser.add_argument('-v', '--visualize', type=int, default=0,
                        help="""1: Render the observation and
                                   the predicted semantic map
                                (default: 0)""")
    parser.add_argument('--print_images', type=int, default=0,
                        help='1: save visualization as images')

    # Environment, dataset and episode specifications
    parser.add_argument('-fw', '--frame_width', type=int, default=640,
                        help='Frame width (default:160)')
    parser.add_argument('-fh', '--frame_height', type=int, default=480,
                        help='Frame height (default:120)')
    parser.add_argument("--task_config", type=str,
                        default="multi_objectnav_hm3d.yaml",
                        help="path to config yaml containing task information")
    parser.add_argument('--hfov', type=float, default=79.0,
                        help="horizontal field of view in degrees")

    # Model Hyperparameters
    parser.add_argument('--agent', type=str, default="sem_exp")
    parser.add_argument('--num_local_steps', type=int, default=25,
                        help="""Number of steps the local policy
                                between each global step""")
    parser.add_argument('-n', '--num_processes', type=int, default=1)
    parser.add_argument('--rank', type=int, default=0)
    parser.add_argument('--gpu_id', type=int, default=0)

    parser.add_argument('--map_resolution', type=int, default=5)
    parser.add_argument('--map_size_cm', type=int, default=2400)
    parser.add_argument('--map_height_cm', type=int, default=130)
    parser.add_argument('--sem_threshold', type=float, default=0.85)
    parser.add_argument('--object_goal_dilation_cells', type=int, default=4,
                        help='goal dilation radius in map cells after the target object is detected')
    parser.add_argument('--frontier_goal_dilation_cells', type=int, default=12,
                        help='goal dilation radius in map cells while exploring frontiers')
    parser.add_argument('--target_confirm_frames', type=int, default=3,
                        help='number of target detections required before committing object point cloud')
    parser.add_argument('--target_min_points', type=int, default=50,
                        help='minimum valid depth points required for a target detection')
    parser.add_argument('--target_max_mask_ratio', type=float, default=0.45,
                        help='ignore target masks covering more than this fraction of the image')
    parser.add_argument('--target_debug', type=int, default=0,
                        help='1: print target detection filtering details')
    parser.add_argument('--target_stop_distance', type=float, default=0.8,
                        help='stop only when the confirmed target is within this horizontal distance in meters')
    parser.add_argument('--target_distance_percentile', type=float, default=50.0,
                        help='percentile of target point distances used for stop-distance estimation')
    parser.add_argument('--target_visual_servo', type=int, default=1,
                        help='1: immediately servo toward a visible target before frontier/FMM actions')
    parser.add_argument('--target_center_tolerance', type=float, default=0.18,
                        help='normalized image-center tolerance for direct target servoing')
    parser.add_argument('--target_visual_min_points', type=int, default=10,
                        help='minimum valid depth points for immediate visual target servoing')
    parser.add_argument('--target_confirm_miss_tolerance', type=int, default=20,
                        help='missing target frames tolerated before clearing pending target confirmations')
    parser.add_argument('--target_immediate_confidence', type=float, default=0.9,
                        help='commit target immediately when confidence is at least this value')
    parser.add_argument('--initial_scan_steps', type=int, default=35,
                        help='number of initial rotate-in-place steps before normal navigation if no target is found')
    parser.add_argument('--num_agents', type=int, default=2)
    parser.add_argument('--cmd_transport', type=str, default='ros',
                        choices=['ros', 'zmq', 'both'],
                        help='command transport for real robot control')
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel_bridge',
                        help='ROS Twist topic used when cmd_transport is ros or both')
    parser.add_argument('--cmd_zmq_addr', type=str, default='tcp://192.168.123.18:5557',
                        help='ZMQ speedctl address used when cmd_transport is zmq or both')
    parser.add_argument('--rgb_topic', type=str, default='/camera/camera/color/image_raw',
                        help='RGB image topic')
    parser.add_argument('--depth_topic', type=str, default='/camera/camera/depth/image_rect_raw',
                        help='Depth image topic')
    parser.add_argument('--camera_info_topic', type=str, default='/camera/camera/color/camera_info',
                        help='CameraInfo topic for RGB camera intrinsics')
    parser.add_argument('--rgbd_sync_slop', type=float, default=0.1,
                        help='ApproximateTimeSynchronizer slop for RGB and depth messages')
    parser.add_argument('--input_debug', type=int, default=1,
                        help='1: print which RGB-D/CameraInfo input is still missing')
    
    
    # train_se_frontier
    parser.add_argument('--nav_mode', type=str, default="gpt",
                        choices=['nearest', 'co_ut', 'fill', "gpt"])
    parser.add_argument('--fill_mode', type=int, default=0)
    parser.add_argument('--gpt_type', type=int, default=2,
                        help="""0: text-davinci-003
                                1: gpt-3.5-turbo
                                2: gpt-4o
                                3: gpt-4o-mini
                                (default: 2)""")
                                   
    # parse arguments
    args = parser.parse_args()

    args.cuda = torch.cuda.is_available()

    return args
