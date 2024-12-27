from typing import Optional, List, Literal
from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime


class ProfessionalCreate(BaseModel):
    name: str = Field(..., max_length=255, description="The name of the professional")
    email: Optional[str] = Field(None, description="The email of the professional")
    password: str = Field(..., description="The password of the professional")
    role: Literal["doctor", "nurse", "admin"] = Field(
        ..., description="The role of the professional"
    )


class ProfessionalSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str = Field(..., description="The ID of the professional")
    name: str
    role: str
    email: Optional[str]
    password: str
    created_at: datetime = Field(default_factory=datetime.now)


class ProfessionalLogin(BaseModel):
    name: str
    password: str


class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"


class ProfessionalResponse(BaseModel):
    professionals: List[ProfessionalSchema]
