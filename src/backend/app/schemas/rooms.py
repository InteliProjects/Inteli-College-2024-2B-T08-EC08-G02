from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import List


class RoomsCreate(BaseModel):
    id: int = Field(..., description="The ID of the room")
    x_pose: float = Field(..., description="Position x of the waypoint")
    y_pose: float = Field(..., description="Position y of the waypoint")
    z_pose: float = Field(..., description="Position z of the waypoint")
    w_pose: float = Field(..., description="Direction of the robot")


class RoomsSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int = Field(..., description="The ID of the task")
    x_pose: float
    y_pose: float
    z_pose: float
    w_pose: float


class TaskRobotResponse(BaseModel):
    tasks: List[TaskRobotSchema]
