from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel
from typing import Optional

from app.services.robot_schedule.scheduler_service import SchedulerController, SchedulerControllerSingleton
from app.services.robot.robot import RobotServiceSingleton
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

router = APIRouter(prefix="/scheduler", tags=["Scheduler Organizer"])

class SchedulerStatusResponse(BaseModel):
    status: str
    next_schedule: Optional[dict] = None

def get_scheduler_controller() -> SchedulerController:
    # Get the RobotService instance (Singleton)
    robot_service = RobotServiceSingleton.get_instance()
    
    # Get the SchedulerController instance (Singleton)
    scheduler_controller = SchedulerControllerSingleton.get_instance(robot_service)
    
    return scheduler_controller

@router.post("/scheduler/start")
async def start_scheduler():
    # Get RobotService instance (Singleton)
    robot_service = RobotServiceSingleton.get_instance()

    # Get SchedulerController instance (Singleton)
    scheduler_controller = SchedulerControllerSingleton.get_instance(robot_service)

    # Start the scheduler
    if not scheduler_controller.is_running():
        await scheduler_controller.start()
        return {"message": "Scheduler started"}
    else:
        return {"message": "Scheduler is already running"}

# Stop the scheduler
@router.post("/scheduler/stop")
async def stop_scheduler():
    robot_service = RobotServiceSingleton.get_instance()
    scheduler_controller = SchedulerControllerSingleton.get_instance(robot_service)

    if scheduler_controller.is_running():
        await scheduler_controller.stop()
        return {"message": "Scheduler stopped"}
    else:
        return {"message": "Scheduler is not running"}

@router.get("/status", response_model=SchedulerStatusResponse)
async def get_scheduler_status(
    scheduler_controller: SchedulerController = Depends(get_scheduler_controller)
):
    if scheduler_controller.is_running():
        next_schedule = scheduler_controller.next_schedule  # Get the pre-fetched schedule
        if next_schedule:
            return SchedulerStatusResponse(
                status="running",
                next_schedule={
                    "robot_id": next_schedule["robot_id"],
                    "scheduled_time": next_schedule["scheduled_time"],
                },
            )
        return SchedulerStatusResponse(status="running", next_schedule=None)
    return SchedulerStatusResponse(status="stopped", next_schedule=None)
