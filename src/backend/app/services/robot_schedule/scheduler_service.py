import asyncio
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from app.services.robot_schedule.robot_schedule import RobotScheduleService
from app.services.robot.robot import RobotService
from app.services.waypoint_to_room.waypoint_to_room import WaypointToRoomService
from app.services.crud_questions.question import QuestionService
from app.services.crud_answers.answer import AnswerService
from app.services.crud_patient.patient import PatientService
from app.services.llm.llm import OpenAIService
import datetime
from app.core.db import mongo_db
import logging
import os

from typing import Optional

class SchedulerController:
    def __init__(self, robot_service: RobotService):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.running = False
        self.task = None
        self.robot_service = robot_service
        self.schedule_service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
        self.waypoint_to_room_service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
        self.question_service = QuestionService(mongo_db.get_collection("questions"))
        self.answer_service = AnswerService(mongo_db.get_collection("answers"))
        self.patient_service = PatientService(mongo_db.get_collection("patients"))
        self.openai_service = OpenAIService(api_key=os.getenv("OPENAI_API_KEY"))
        self.next_schedule = None
        self.logger.info("SchedulerController initialized.")

    def is_running(self):
        return self.running

    async def start(self):
        if self.running:
            self.logger.warning("Scheduler is already running.")
            return
        self.running = True
        self.task = asyncio.create_task(self._schedule_runner())
        self.logger.info("Scheduler started.")

    async def stop(self):
        if not self.running:
            self.logger.warning("Scheduler is already stopped.")
            return
        self.running = False
        if self.task:
            await self.task  # Wait for the task to finish
            self.logger.info("Scheduler stopped.")

    async def _schedule_runner(self):
        while self.running:
            try:
                # Fetch the next schedule
                self.next_schedule = await self.schedule_service.get_next_schedule()
                if not self.next_schedule:
                    self.logger.info("No schedules found. Retrying in 60 seconds...")
                    await asyncio.sleep(60)
                    continue

                # Ensure scheduled_time is a datetime object
                scheduled_time_raw = self.next_schedule["scheduled_time"]

                if isinstance(scheduled_time_raw, str):
                    scheduled_time = datetime.datetime.fromisoformat(scheduled_time_raw)
                elif isinstance(scheduled_time_raw, datetime.datetime):
                    scheduled_time = scheduled_time_raw
                else:
                    self.logger.error(f"Unexpected type for scheduled_time: {type(scheduled_time_raw)}")
                    continue

                # Compare scheduled_time with current time
                current_time = datetime.datetime.utcnow()
                if current_time < scheduled_time:
                    sleep_time = (scheduled_time - current_time).total_seconds()
                    self.logger.info(f"Schedule not due yet. Sleeping for {sleep_time} seconds...")
                    await asyncio.sleep(sleep_time)
                    continue

                # Proceed with the scheduled task
                patient_id = self.next_schedule["patient_id"]
                patient = await self.patient_service.get_patient(patient_id)
                if not patient:
                    self.logger.warning(f"Patient {patient_id} not found. Skipping...")
                    continue

                id_room = patient.id_room
                waypoint_to_room = await self.waypoint_to_room_service.get_waypoint_by_room_id(id_room)
                if not waypoint_to_room:
                    self.logger.warning(f"No waypoint found for room ID: {id_room}. Skipping...")
                    continue

                # Navigation logic
                self.logger.info(f"Navigating to room '{waypoint_to_room.place_name}' at waypoint {waypoint_to_room.waypoint_id}")

                # Step 1: Set initial pose to (0, 0, 0)
                await self.robot_service.set_initial_pose_async(0.0, 0.0, 0.0)

                print(f"Waypoint to room: {waypoint_to_room}")

                # Step 2: Create a PoseStamped for the waypoint
                waypoint_pose = PoseStamped()
                waypoint_pose.header.frame_id = 'map'
                waypoint_pose.pose.position.x = float(waypoint_to_room.x_pose)
                waypoint_pose.pose.position.y = float(waypoint_to_room.y_pose)
                waypoint_pose.pose.position.z = 0.0
                q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, float(waypoint_to_room.w_pose))
                waypoint_pose.pose.orientation.x = q_x
                waypoint_pose.pose.orientation.y = q_y
                waypoint_pose.pose.orientation.z = q_z
                waypoint_pose.pose.orientation.w = q_w

                # Step 3: Navigate to the waypoint
                await self.robot_service.navigate_to_pose_async(waypoint_pose)

                # Step 4: Run the anamnese process
                self.logger.info(f"Starting anamnese for patient {patient_id}.")
                await self.openai_service.run_anamnese(patient_id)  # Call OpenAIService to handle questions

                # Return to origin (0, 0, 0)
                base_pose = PoseStamped()
                base_pose.header.frame_id = 'map'
                base_pose.pose.position.x = 0.0
                base_pose.pose.position.y = 0.0
                base_pose.pose.position.z = 0.0
                q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
                base_pose.pose.orientation.x = q_x
                base_pose.pose.orientation.y = q_y
                base_pose.pose.orientation.z = q_z
                base_pose.pose.orientation.w = q_w
                self.logger.info("Returning to starting position.")
                await self.robot_service.navigate_to_pose_async(base_pose)

            except Exception as e:
                self.logger.error(f"Scheduler error: {e}")
                await asyncio.sleep(10)

    def shutdown(self):
        self.robot_service.shutdown()

class SchedulerControllerSingleton:
    _instance = None

    @staticmethod
    def get_instance(robot_service: RobotService):
        from app.utils.rclpyman import RclpyManager  # Import RclpyManager
        rclpy_manager = RclpyManager()
        rclpy_manager.init()  # Ensure rclpy is initialized

        if not SchedulerControllerSingleton._instance:
            SchedulerControllerSingleton._instance = SchedulerController(robot_service)
        return SchedulerControllerSingleton._instance
