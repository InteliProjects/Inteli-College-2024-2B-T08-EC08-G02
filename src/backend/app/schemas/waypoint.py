from typing import Optional, List
from pydantic import BaseModel, Field, ConfigDict


class PoseRequest(BaseModel):
    x: float
    y: float
    z: float
    yaw: float

class WaypointsRequest(BaseModel):
    waypoints: List[PoseRequest]
