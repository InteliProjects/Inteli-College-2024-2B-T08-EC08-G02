from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import List


class WaypointToRoomCreate(BaseModel):
    waypoint_id: str = Field(..., description="The ID of the waypoint")
    place_name: str = Field(..., description="The name of the place associated with the waypoint")
    id_room: str = Field(..., description="The ID of the room associated with the waypoint")
    x_pose: float = Field(..., description="Position x of the waypoint")
    y_pose: float = Field(..., description="Position y of the waypoint")
    z_pose: float = Field(..., description="Position z of the waypoint")
    w_pose: float = Field(..., description="Direction of the robot at the waypoint")


class WaypointToRoomSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str = Field(..., description="The ID of the mapping")
    waypoint_id: str
    place_name: str
    id_room: str
    x_pose: float
    y_pose: float
    z_pose: float
    w_pose: float


class WaypointToRoomResponse(BaseModel):
    mappings: List[WaypointToRoomSchema]
