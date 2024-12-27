from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException

from app.schemas.answer import (
    AnswerCreate,
    AnswerSchema,
    AnswerResponse,
)

from app.services.crud_answers.answer import AnswerService

from app.core.db import mongo_db

router = APIRouter(prefix="/answer", tags=["Answers CRUD"])

@router.post("/", response_model=AnswerSchema)
async def create_answer(answer_data: AnswerCreate):
    service = AnswerService(mongo_db.get_collection("answers"))
    return await service.create_answer(answer_data)

@router.get("/{answer_id}", response_model=AnswerSchema)
async def get_answer(answer_id: str):
    service = AnswerService(mongo_db.get_collection("answers"))
    answer = await service.get_answer(answer_id)
    if not answer:
        raise HTTPException(status_code=404, detail="Answer not found")
    return answer

@router.get("/", response_model=AnswerResponse)
async def list_answers():
    service = AnswerService(mongo_db.get_collection("answers"))
    answers = await service.list_answers()
    return AnswerResponse(answers=answers)

@router.get("/question/{question_id}", response_model=AnswerResponse)
async def get_answer_from_question_id(question_id: str):
    service = AnswerService(mongo_db.get_collection("answers"))
    answers = await service.get_answer_from_question_id(question_id)
    return AnswerResponse(answers=answers)

@router.get("/patient/{patient_id}", response_model=AnswerResponse)
async def get_answer_by_patient_id(patient_id: str):
    service = AnswerService(mongo_db.get_collection("answers"))
    answers = await service.get_answer_by_patient_id(patient_id)
    return AnswerResponse(answers=answers)

@router.delete("/{answer_id}", response_model=dict)
async def delete_answer(answer_id: str):
    service = AnswerService(mongo_db.get_collection("answers"))
    deleted = await service.delete_answer(answer_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Answer not found")
    return {"deleted": deleted}