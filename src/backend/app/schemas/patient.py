from typing import Optional, List
from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime


class PatientCreate(BaseModel):
    name: str = Field(..., max_length=255, description="The name of the patient")
    date_of_birth: datetime = Field(..., description="The date of birth of the patient")
    sex: str = Field(..., max_length=10, description="Gender of the patient")
    id_room: int
    current_condition: Optional[str] = Field(
        None, description="Current condition of the patient"
    )
    professional_id: Optional[str] = Field(
        None, description="The ID of the professional that is treating the patient"
    )


class PatientSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str = Field(..., description="The ID of the patient")
    name: str
    date_of_birth: datetime
    sex: str
    id_room: int
    current_condition: Optional[str]
    professional_id: Optional[str]


class PatientResponse(BaseModel):
    patients: List[PatientSchema]
