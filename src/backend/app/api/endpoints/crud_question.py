from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException

from app.schemas.question import (
    QuestionCreate,
    QuestionSchema,
    QuestionResponse,
)

from app.services.crud_questions.question import QuestionService

from app.core.db import mongo_db

router = APIRouter(prefix="/question", tags=["Questions CRUD"])

@router.post("/", response_model=QuestionSchema)
async def create_question(question_data: QuestionCreate):
    service = QuestionService(mongo_db.get_collection("questions"))
    return await service.create_question(question_data)

@router.get("/{question_id}", response_model=QuestionSchema)
async def get_question(question_id: str):
    service = QuestionService(mongo_db.get_collection("questions"))
    question = await service.get_question(question_id)
    if not question:
        raise HTTPException(status_code=404, detail="Question not found")
    return question

@router.get("/", response_model=QuestionResponse)
async def list_questions():
    service = QuestionService(mongo_db.get_collection("questions"))
    questions = await service.list_questions()
    return QuestionResponse(questions=questions)

@router.put("/{question_id}", response_model=QuestionSchema)
async def update_question(question_id: str, update_data: QuestionCreate):
    service = QuestionService(mongo_db.get_collection("questions"))
    question = await service.update_question(question_id, update_data)
    if not question:
        raise HTTPException(status_code=404, detail="Question not found")
    return question

@router.get("/by-patient/{patient_id}", response_model=QuestionResponse)
async def get_question_by_patient_id(patient_id: str):
    service = QuestionService(mongo_db.get_collection("questions"))
    questions = await service.get_question_by_patient_id(patient_id)
    return QuestionResponse(questions=questions)

@router.delete("/{question_id}", response_model=dict)
async def delete_question(question_id: str):
    service = QuestionService(mongo_db.get_collection("questions"))
    deleted = await service.delete_question(question_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Question not found")
    return {"deleted": deleted}