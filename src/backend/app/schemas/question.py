from typing import Optional, List
from pydantic import BaseModel, Field, ConfigDict


class QuestionCreate(BaseModel):
    text: str = Field(..., description="The text of the question")
    patient_id: str = Field(..., description="The ID of the patient")


class QuestionSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str = Field(..., description="The ID of the question")
    text: str
    patient_id: str


class QuestionResponse(BaseModel):
    questions: List[QuestionSchema]
