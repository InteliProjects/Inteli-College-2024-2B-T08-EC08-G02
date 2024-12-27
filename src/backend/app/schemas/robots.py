from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class RobotCreate(BaseModel):
    ros_domain_id: int = Field(..., description="The ROS domain ID associated with the robot")
    status: str = Field(..., description="The current status of the robot (e.g., 'idle', 'busy', 'error')")
    current_location: Optional[str] = Field(None, description="The current location of the robot (e.g., 'Lab', 'Warehouse')")
    name: str = Field(..., description="The name of the robot")
    patient_id: Optional[str] = Field(None, description="The ID of the patient associated with the robot")


class RobotSchema(BaseModel):
    id: str = Field(..., description="The unique database ID for the robot")
    ros_domain_id: int = Field(..., description="The ROS domain ID associated with the robot")
    status: str = Field(..., description="The current status of the robot (e.g., 'idle', 'busy', 'error')")
    current_location: Optional[str] = Field(None, description="The current location of the robot (e.g., 'Lab', 'Warehouse')")
    name: str = Field(..., description="The name of the robot")
    created_at: datetime = Field(default_factory=datetime.now, description="The timestamp when the robot was created")
    updated_at: Optional[datetime] = Field(default_factory=datetime.now, description="The timestamp of the last update")
    patient_id: Optional[str] = Field(None, description="The ID of the patient associated with the robot")


class RobotResponse(BaseModel):
    robots: list[RobotSchema] = Field(..., description="A list of robots and their details")
