from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import List


class TaskRobotCreate(BaseModel):
    id_patient: int = Field(..., description="The ID of the patient")
    id_question: int = Field(..., description="The ID of the question")
    status: str = Field(..., description="The status of the task")
    date_hour: datetime = Field(..., description="The hour and date to the robot's task")


class TaskRobotSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int = Field(..., description="The ID of the task")
    id_patient: int
    id_question: int
    status: str
    created_at: datetime = Field(default_factory=datetime.now)
    date_hour: datetime 


class TaskRobotResponse(BaseModel):
    tasks: List[TaskRobotSchema]
