import asyncio
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from typing import List
import logging

from typing import Optional

class RobotService:
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.navigator = BasicNavigator()
        self.navigation_lock = asyncio.Lock()
        self.logger.info("RobotService initialized.")

    async def set_initial_pose_async(self, x: float, y: float, yaw: float) -> dict:
        async with self.navigation_lock:
            q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = q_x
            pose.pose.orientation.y = q_y
            pose.pose.orientation.z = q_z
            pose.pose.orientation.w = q_w

            self.navigator.setInitialPose(pose)
            self.navigator.waitUntilNav2Active()

            self.logger.info(f"Initial pose set to ({x}, {y}, {yaw}).")
            return {"status": "success", "pose": self.pose_to_dict(pose)}

    async def navigate_to_pose_async(self, pose: PoseStamped) -> dict:
        async with self.navigation_lock:
            self.navigator.goToPose(pose)
            self.logger.info(f"Navigating to pose: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.orientation.z}).")
            while not self.navigator.isTaskComplete():
                self.logger.debug("Navigating...")
                await asyncio.sleep(0.1)  # Yield control to the event loop

            self.logger.info(f"Reached pose: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.orientation.z}).")
            return {"status": "success", "pose": self.pose_to_dict(pose)}

    def pose_to_dict(self, pose: PoseStamped) -> dict:
        return {
            "header": {
                "frame_id": pose.header.frame_id,
                "stamp": {
                    "sec": pose.header.stamp.sec,
                    "nanosec": pose.header.stamp.nanosec
                }
            },
            "pose": {
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z,
                },
                "orientation": {
                    "x": pose.pose.orientation.x,
                    "y": pose.pose.orientation.y,
                    "z": pose.pose.orientation.z,
                    "w": pose.pose.orientation.w
                }
            }
        }

    def shutdown(self):
        self.navigator = None
        self.logger.info("RobotService shutdown.")

class RobotServiceSingleton:
    _instance = None

    @staticmethod
    def get_instance():
        from app.utils.rclpyman import RclpyManager  # Import RclpyManager
        rclpy_manager = RclpyManager()
        rclpy_manager.init()  # This will only initialize rclpy if it hasn't been initialized already

        if not RobotServiceSingleton._instance:
            RobotServiceSingleton._instance = RobotService()
        return RobotServiceSingleton._instance
