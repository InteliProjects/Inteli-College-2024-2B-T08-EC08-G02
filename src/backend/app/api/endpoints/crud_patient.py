from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException

from app.schemas.patient import (
    PatientCreate,
    PatientSchema,
    PatientResponse,
)

from app.services.crud_patient.patient import PatientService

from app.core.db import mongo_db

router = APIRouter(prefix="/patients", tags=["Patients CRUD"])


@router.post("/", response_model=PatientSchema)
async def create_patient(patient_data: PatientCreate):
    service = PatientService(mongo_db.get_collection("patients"))
    return await service.create_patient(patient_data)


@router.get("/{patient_id}", response_model=PatientSchema)
async def get_patient(patient_id: str):
    service = PatientService(mongo_db.get_collection("patients"))
    patient = await service.get_patient(patient_id)
    if not patient:
        raise HTTPException(status_code=404, detail="Patient not found")
    return patient


@router.get("/", response_model=PatientResponse)
async def list_patients():
    service = PatientService(mongo_db.get_collection("patients"))
    patients = await service.list_patients()
    return PatientResponse(patients=patients)


@router.put("/{patient_id}", response_model=PatientSchema)
async def update_patient(patient_id: str, update_data: PatientCreate):
    service = PatientService(mongo_db.get_collection("patients"))
    patient = await service.update_patient(patient_id, update_data)
    if not patient:
        raise HTTPException(status_code=404, detail="Patient not found")
    return patient


@router.delete("/{patient_id}", response_model=dict)
async def delete_patient(patient_id: str):
    service = PatientService(mongo_db.get_collection("patients"))
    deleted = await service.delete_patient(patient_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Patient not found")
    return {"deleted": deleted}


@router.get("/search/", response_model=PatientSchema)
async def get_patient_by_name(name: str):
    service = PatientService(mongo_db.get_collection("patients"))
    patient = await service.get_by_name(name)
    if not patient:
        raise HTTPException(status_code=404, detail="Patient not found")
    return patient


@router.get("/professional/{professional_id}", response_model=PatientResponse)
async def get_patient_by_professional_id(professional_id: str):
    service = PatientService(mongo_db.get_collection("patients"))
    patients = await service.get_patient_by_professional_id(
        professional_id=professional_id
    )
    if not patients:
        raise HTTPException(
            status_code=404, detail="No patients atrelated to this professional"
        )
    return PatientResponse(patients=patients)


@router.put("/{patient_id}/associate/{professional_id}", response_model=PatientSchema)
async def associate_patient_to_professional(patient_id: str, professional_id: str):
    service = PatientService(mongo_db.get_collection("patients"))
    updated_patient = await service.associate_patient_to_professional(
        patient_id=patient_id, professional_id=professional_id
    )

    if not updated_patient:
        raise HTTPException(
            status_code=404,
            detail="Patient not found or failed to associate with professional",
        )

    return updated_patient

@router.put("/{patient_id}/disassociate", response_model=PatientSchema)
async def disassociate_patient_from_professional(patient_id: str):
    service = PatientService(mongo_db.get_collection("patients"))
    updated_patient = await service.dessasociate_patient_from_professional(
        patient_id=patient_id
    )

    if not updated_patient:
        raise HTTPException(
            status_code=404,
            detail="Patient not found or failed to disassociate from professional",
        )

    return updated_patient