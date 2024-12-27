from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException
from fastapi.security import OAuth2PasswordRequestForm
from app.utils.logger import get_logger
from datetime import timedelta

from app.schemas.professional import ProfessionalCreate, ProfessionalLogin, Token
from app.services.login.auth_service import AuthService
from app.services.crud.professional import ProfessionalService

from app.core.config import Settings

from app.core.db import mongo_db

router = APIRouter(prefix="/auth", tags=["Authentication"])
logger = get_logger()
auth_service = AuthService()
professional_service = ProfessionalService(mongo_db.get_collection("professionals"))


@router.post("/register", response_model=Token)
async def register_professional(professional: ProfessionalCreate):
    existing_professional = await professional_service.get_by_name(professional.name)
    if existing_professional:
        raise HTTPException(status_code=400, detail="Professional already registered")

    new_professional = await professional_service.create_professional(professional)

    access_token = auth_service.create_access_token(
        data={"sub": new_professional.name, "role": new_professional.role, "id": str(new_professional.id)},
    )
    return Token(access_token=access_token)


@router.post("/login", response_model=Token)
async def login_professional(professional: ProfessionalLogin):
    db_professional = await professional_service.get_by_name(professional.name)
    if not db_professional or not auth_service.verify_password(
        professional.password, db_professional.password
    ):
        raise HTTPException(status_code=400, detail="Incorrect username or password")

    access_token = auth_service.create_access_token(
        data={"sub": db_professional.name, "role": db_professional.role, "id": str(db_professional.id)},
    )
    return Token(access_token=access_token)


@router.post("/login-test", response_model=Token)
async def login_test(form_data: OAuth2PasswordRequestForm = Depends()):
    db_professional = await professional_service.get_by_name(form_data.username)
    if not db_professional or not auth_service.verify_password(
        form_data.password, db_professional.password
    ):
        raise HTTPException(status_code=401, detail="Incorrect username or password")

    access_token = auth_service.create_access_token(
        data={"sub": db_professional.name, "role": db_professional.role, "id": str(db_professional.id)},
    )
    return Token(access_token=access_token)


@router.post("/protected_route", response_model=dict)
async def protected_route(current_user: dict = Depends(auth_service.get_current_user)):
    return {"message": "This route is protected!", "user": current_user}


@router.get(
    "/doctor-only",
    dependencies=[Depends(auth_service.require_role("doctor"))],
)
async def doctor_only_route():
    return {"message": "This route is only for doctors!"}
