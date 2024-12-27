from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

class RobotScheduleCreate(BaseModel):
    robot_id: str = Field(..., description="The ID of the robot assigned to the patient")
    patient_id: str = Field(..., description="The ID of the patient that the robot will attend to")
    scheduled_time: datetime = Field(..., description="The exact datetime when the robot is scheduled to attend the patient")

class RobotScheduleSchema(BaseModel):
    id: str = Field(..., description="The unique database ID for the robot schedule entry")
    robot_id: str = Field(..., description="The ID of the robot assigned to the patient")
    patient_id: str = Field(..., description="The ID of the patient that the robot will attend to")
    scheduled_time: datetime = Field(..., description="The exact datetime when the robot is scheduled to attend the patient")
    created_at: datetime = Field(default_factory=datetime.now, description="The timestamp when the schedule entry was created")
    updated_at: Optional[datetime] = Field(default_factory=datetime.now, description="The timestamp of the last update for this schedule entry")

class RobotScheduleResponse(BaseModel):
    schedules: list[RobotScheduleSchema] = Field(..., description="A list of scheduled entries detailing when a robot will attend patients")
