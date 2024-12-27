from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException
from app.utils.logger import get_logger

from app.schemas.professional import (
    ProfessionalCreate,
    ProfessionalSchema,
    ProfessionalResponse,
)

from app.services.crud.professional import ProfessionalService

from app.core.db import mongo_db

router = APIRouter(prefix="/crud", tags=["CRUD"])
logger = get_logger()


@router.post("/", response_model=ProfessionalSchema)
async def create_professional(professional_data: ProfessionalCreate):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    return await service.create_professional(professional_data)


@router.get("/{professional_id}", response_model=ProfessionalSchema)
async def get_professional(professional_id: str):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professional = await service.get_professional(professional_id)
    if not professional:
        raise HTTPException(status_code=404, detail="Professional not found")
    return professional


@router.get("/", response_model=ProfessionalResponse)
async def list_professionals():
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professionals = await service.list_professionals()
    return ProfessionalResponse(professionals=professionals)


@router.put("/{professional_id}", response_model=ProfessionalSchema)
async def update_professional(professional_id: str, update_data: ProfessionalCreate):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professional = await service.update_professional(professional_id, update_data)
    if not professional:
        raise HTTPException(status_code=404, detail="Professional not found")
    return professional


@router.delete("/{professional_id}", response_model=dict)
async def delete_professional(professional_id: str):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    deleted = await service.delete_professional(professional_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Professional not found")
    return {"deleted": deleted}
