import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, 
                       HistoryPolicy, ReliabilityPolicy)

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point32, Polygon
from airsim_interfaces.msg import CollisionState
from airsim_interfaces.srv import VoxelGrid
from adk_node.msg import TargetPerception, WaypointStatus, WaypointPath, AdkStatus
from perception_interfaces.msg import TargetBelief, BeliefMap
from projectairsim_ros.msg import CollisionInfo, Kinematics

import os
import json
import time
import numpy as np
from cv_bridge import CvBridge

from .mission import Mission
from .odometry import OdometryManager

class TerminateException(Exception):
    pass

class ManeuverNode(Node):
    def __init__(self):
        super().__init__('maneuver_node')

        drone_name = "Drone1"
        
        self.br = CvBridge()
        self.last_collision = None

        self.acceptance_threshold = 0.99
        self.perception_input = "ground_truth"
        self.rotation_implementation = os.environ.get("IMPLEMENTATION", "adk")

        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.waypointPublisher = self.create_publisher(WaypointPath, f'/adk_node/input/{drone_name}/waypoints', self.qos_profile)
        self.publish_freq = 0.1
        self.waypointTimer = self.create_timer(self.publish_freq, self.waypoint_callback)
        
        # Planning node services subscriptions
        ## Started ADK 
        self.startingSubscription = self.create_subscription(
            AdkStatus, 'adk_node/output/scenario_starting', self.starting_callback, self.qos_profile)
        ## Odometry 
        self.odometrySubscription = self.create_subscription(
            Odometry, f'/airsim_node/{drone_name}/odom_local_ned', self.odometry_callback, 5)
        
        ## Perception
        perception_topic = "/perception_node/output/target_perception" if self.perception_input != "ground_truth" else 'adk_node/ground_truth/perception'
        self.perceptionSubscription = self.create_subscription(
            TargetPerception, perception_topic, self.perception_callback, 1)
        ## Collisions
        self.collisionStatusSubscription = self.create_subscription(
            CollisionInfo, f'/airsim_node/{drone_name}/collision_state', self.collision_state_callback, 0)  
        ## Waypoint Status 
        self.waypointStatusSubscription = self.create_subscription(
            WaypointStatus, f'adk_node/output/{drone_name}/waypoint_status', self.waypoint_status_callback, self.qos_profile)
        ## End the node
        self.terminate_subscription = self.create_subscription(Empty, '/adk_node/input/end_scenario', self.terminate_callback, 1)
        self.terminate_publisher = self.create_publisher(Empty, '/adk_node/input/end_scenario', 1)
        ## Investigation Trigger
        self.targetBeliefSubscription = self.create_subscription(
            TargetBelief, '/perception_node/output/target_belief', self.target_belief_updated_callback, 0)
        ## Belief map
        self.targetBeliefSubscription = self.create_subscription(
            BeliefMap, '/perception_node/output/belief_map', self.belief_map_updated_callback, 0)

        # Planning node services
        self.voxelGridService = self.create_client(VoxelGrid, 'adk_node/VoxelGrid')

        # Load Mission
        self.saveDir = f"/data/default"
        # self.saveDir = f"/data/{int(time.time())}"
        os.makedirs(f"{self.saveDir}/perceptionGT", exist_ok=True)
        os.makedirs(f"{self.saveDir}/beliefs", exist_ok=True)
        os.makedirs(f"{self.saveDir}/interruptions", exist_ok=True)
        os.makedirs(f"{self.saveDir}/waypoints", exist_ok=True)
        os.makedirs(f"{self.saveDir}/high_level_planning", exist_ok=True)
        
        # Mission
        self.mission = Mission(description_folder_dir="/mission_briefing", 
                               environment_folder_dir="/environmentConfigFiles", 
                               saveDir=self.saveDir, logger=self.get_logger())
         # Odometry  
        self.odometry_manager = OdometryManager(self.saveDir, self.get_logger())
        self.wayponint_poses = None

        self.going = True
        self.current_waypoint = None

    def starting_callback(self, data):
        self.odometry_manager.modules_initialization(data)

    def belief_map_updated_callback(self, data):
        pass

    def target_belief_updated_callback(self, data):
        pass
        
    def odometry_callback(self, data):
        self.odometry_manager.update(data)

    def waypoint_status_callback(self, data):
        current_waypoint = int(data.current_point_on_path)
        self.get_logger().info(f"Current Waypoint: {data}")

    def collision_state_callback(self, data):

        collision_point = np.array([data.impact_point.x, data.impact_point.y, data.impact_point.z])

        header = data.header
        time = header.stamp.sec + header.stamp.nanosec / (1e+9)
        collision_log = f"{time}, {collision_point[0]}, {collision_point[1]}, {collision_point[2]}\n"

        with open(f"{self.saveDir}/collision.txt", 'a') as f:
            f.write(collision_log)    

        if not self.odometry_manager.takeoff_complete or self.odometry_manager.position is None or collision_point[2]>-5:
            return

        if self.last_collision is not None and np.linalg.norm(collision_point - self.last_collision) < 3:
            return
        
        self.get_logger().info("\n\nCollision: [{:.2f}, {:.2f}, {:.2f}]\n\n".format(*collision_point.tolist()))

        self.last_collision = collision_point

        
    def perception_callback(self, data):

        pose = data.position
        attributes = {attr.key: attr.value for attr in data.attributes}
        
        fileName = f"{data.camera}_{data.detection_time.sec}_{str(data.detection_time.nanosec).zfill(9)}.png"
        # image = self.br.imgmsg_to_cv2(data.image)
        # cv2.imwrite(f"{self.saveDir}/perceptionGT/{fileName}", image)

        metadata = {
            "entity_id": data.entity_id,
            "enter_or_leave": data.enter_or_leave,
            "camera": data.camera,
            "probability": data.probability,
            "detection_time": data.detection_time.sec + data.detection_time.nanosec / (1e+9),
            "position_detected": [pose.position.x, pose.position.y, pose.position.z],
            "orientation": [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
            "bounding_box3d": [data.bounding_box3d.maxx, data.bounding_box3d.maxy, data.bounding_box3d.maxz, 
                               data.bounding_box3d.minx, data.bounding_box3d.miny, data.bounding_box3d.minz],
            "bounding_box2d": [data.bounding_box2d.maxx, data.bounding_box2d.maxy, data.bounding_box2d.minx, 
                               data.bounding_box2d.miny],
            "attributes": attributes,
            "detected": False,
            "image": fileName,
        }

        entity_id = attributes["mission_entity_id"] if "mission_entity_id" in attributes else data.entity_id
        
        fileName = "{}_{}".format(metadata["detection_time"], entity_id)
        with open(f"{self.saveDir}/perceptionGT/{fileName}.json", 'w') as f:
            json.dump(metadata, f, indent=4)   

        #Check if the detected target is inside or not
        position_detected = np.array(metadata["position_detected"])
        if entity_id in self.mission.entity_ids and "location_ground_truth" in self.mission.entities[entity_id]:
            gt_coord = self.mission.entities[entity_id]["location_ground_truth"]
            if np.linalg.norm(np.array(gt_coord)-position_detected) < 15: #The target is near the Ground Truth
                with open(f"{self.saveDir}/detected_entity.txt", 'a') as f:
                    f.write(f"{[entity_id, self.odometry_manager.currentTime]}\n")        

        if entity_id in self.mission.entity_ids and not self.mission.entities[entity_id]["detected"]:
            self.mission.entities[entity_id]["position_detected"] = position_detected
            self.mission.entities[entity_id]["probability"] = metadata["probability"]

            if metadata["probability"] > self.acceptance_threshold and not self.mission.entities[entity_id]["detected"]:
                self.mission.entities[entity_id]["detected"] = True
                metadata["detected"] = True

        self.get_logger().info(f'\n\n\n Perception:'
                               f'\n\tEntity: {entity_id}'
                               f'\n\tProbability: {metadata["probability"]}'
                               f'\n\tPosition: {position_detected}'
                               f'\n\tNew Detection: {metadata["detected"]}'
                               '\n\n\n')    
            
        if all([e["detected"] for e in self.mission.entities.values()]):
            self.get_logger().info("\n\nAll targets detected..")
            self.terminate_node()

    def waypoint_publish(self, path, direction=None, drive_train_type=1, wait_on_last_task=False):
        
        velocity = 6.0

        waypoint = WaypointPath()
        # Publish Waypoints
        waypoint = WaypointPath()
        waypoint.velocity = velocity
        waypoint.wait_on_last_task = wait_on_last_task
        waypoint.lookahead = -1.0
        waypoint.adaptive_lookahead = 0.0
        waypoint.drive_train_type = 1
        waypoint.timeout_sec = 100000.0

        for i, pathpoint in enumerate(path):

            pose = PoseStamped()
            pose.pose.position.x = float(pathpoint[0])
            pose.pose.position.y = float(pathpoint[1])
            pose.pose.position.z = float(pathpoint[2])


            waypoint.path.append(pose)

        self.waypointPublisher.publish(waypoint)

    def rotate_in_place(self, yaw, poses, wait_on_last_task=True):
        poses = []
        # Thanks Hami!
        for i, pos in enumerate(poses):
            target_pose = PoseStamped()
            target_pose.pose.position.x = float(pos[0])
            target_pose.pose.position.y = float(pos[1])
            target_pose.pose.position.z = float(-10)
            poses.append(target_pose)
        wp_path = WaypointPath()
        wp_path.path = poses
        wp_path.velocity = 1.0
        wp_path.timeout_sec = 10000.0
        wp_path.drive_train_type = 0
        wp_path.yaw_is_rate = False
        wp_path.yaw = yaw
        wp_path.wait_on_last_task = wait_on_last_task
        wp_path.lookahead = -1.0
        wp_path.adaptive_lookahead = 1.0
        self.waypointPublisher.publish(wp_path)

    # def rotate_in_place(self, yaw):
    #     poses = []
    #     # Thanks Hami!
    #     for i in range(10):
    #         target_pose = PoseStamped()
    #         target_pose.header.frame_id = '/map'
    #         target_pose.header.stamp = self.get_clock().now().to_msg()
    #         new_x = self.current_position[0] + 2.0 if i % 2 == 0 else self.current_position[0] - 2.0
    #         target_pose.pose.position.x = float(new_x)
    #         target_pose.pose.position.y = float(self.current_position[1])
    #         target_pose.pose.position.z = float(-10)
    #         poses.append(target_pose)
    #     wp_path = WaypointPath()
    #     wp_path.path = poses
    #     wp_path.velocity = 1.0
    #     wp_path.timeout_sec = 10000.0
    #     wp_path.drive_train_type = 0
    #     wp_path.yaw_is_rate = False
    #     wp_path.yaw = yaw
    #     wp_path.wait_on_last_task = True
    #     wp_path.lookahead = -1.0
    #     wp_path.adaptive_lookahead = 1.0
    #     self.waypointPublisher.publish(wp_path)


    def waypoint_callback(self):

        if not self.odometry_manager.is_odom_ready():
            return
        
        arrived = False
        if self.current_waypoint is not None and self.wayponint_poses is not None:
            if self.current_waypoint >= len(self.wayponint_poses):
                self.get_logger().info("\n\nAll waypoints reached..")
                self.wayponint_poses = None
                self.current_waypoint = None
                arrived = True
                

        # Update World Map
                
        """
        RUN PLANIFICATION
        """

        position = self.odometry_manager.position

        # initial_pos = np.array([
        #         -440.0,
        #         320.0,
        #         -10.0
        #     ])
        
        if self.wayponint_poses is None or arrived:
            self.wayponint_poses = []
            for i in range(10):
                new_x = position[0] + 2.0 if i % 2 == 0 else position[0] - 2.0     
                self.wayponint_poses.append(np.array([new_x, position[1], position[2]])) 
            self.logger.info(f"\n\nNew Waypoints: {self.wayponint_poses}")
            self.rotate_in_place(180.0, self.wayponint_poses, wait_on_last_task=True)

        # if self.wayponint_poses is None or arrived:

        #     if self.going:
        #         self.wayponint_poses = [
        #             initial_pos,
        #             initial_pos + np.array([0.0, -50.0, 0.0]),
        #         ]

        #         self.waypoint_publish(self.wayponint_poses, wait_on_last_task=True)

        #         if arrived:  
        #             self.get_logger().info("\n\nArrived and Rotating..")
        #             self.wayponint_poses = []
                    

                    

        #     elif not self.going:
        #         self.wayponint_poses = [
        #             initial_pos + np.array([0.0, -50.0, 0.0]),
        #             initial_pos,
        #         ]

        #         self.waypoint_publish(self.wayponint_poses, wait_on_last_task=True)

        #         if np.linalg.norm(position - self.wayponint_poses[-1]) < 5.0:        
        #             self.rotate_in_place(180.0, self.wayponint_poses[-1], wait_on_last_task=True)

                
        # Terminate Mission
        if self.odometry_manager.current_time is not None and self.odometry_manager.current_time > self.mission.maxTime:
            self.get_logger().info("\n\nTime Limit reached..")
            self.terminate_node()

    def terminate_node(self):

        msg = Empty()
        self.terminate_publisher.publish(msg)
        self.get_logger().info('Shutting down in 5 seconds...\n\n')
        time.sleep(5)
        self.get_logger().info('Shutting down...\n\n')
        raise TerminateException()
        
    def terminate_callback(self, msg):
        self.get_logger().info("\n\nReceived terminate message..")
        self.terminate_node()

def main():
    rclpy.init()

    planningNode = ManeuverNode()
    try:
        rclpy.spin(planningNode)
    except TerminateException:
        pass

    planningNode.destroy_node()
    rclpy.shutdown()
    print("Shut down")
