from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import List


class AccessDashboardCreate(BaseModel):
    id_professional: int = Field(..., description="The ID of the professional")
    id_patient: int = Field(..., description="The ID of the patient")


class AccessDashboardSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int = Field(..., description="The ID of the access")
    id_professional: int
    id_patient: int
    date_hour: datetime = Field(default_factory=datetime.now)


class AcessDashboardResponse(BaseModel):
    accesses: List[AccessDashboardSchema]
