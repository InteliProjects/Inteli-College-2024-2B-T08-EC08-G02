from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import List


class AnswerCreate(BaseModel):
    id_patient: str = Field(..., description="The ID of the patient")
    id_question: str = Field(..., description="The ID of the question")
    answer: str = Field(..., description="The answer to the question")


class AnswerSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str = Field(..., description="The ID of the answer")
    id_patient: str
    id_question: str
    answer: str
    date_hour: datetime = Field(default_factory=datetime.now)


class AnswerResponse(BaseModel):
    answers: List[AnswerSchema]
