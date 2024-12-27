from app.schemas.answer import AnswerCreate, AnswerSchema
from app.services.crud_answers.base import AnswerBase
from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional

class AnswerService(AnswerBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def create_answer(self, answer_data: AnswerCreate) -> AnswerSchema:
        data = answer_data.dict()
        result = await self.collection.insert_one(data)
        created_answer = await self.collection.find_one({"_id": result.inserted_id})
        created_answer["id"] = str(created_answer.pop("_id"))
        return AnswerSchema(**created_answer)
    
    async def get_answer(self, answer_id: str) -> Optional[AnswerSchema]:
        answer = await self.collection.find_one({"_id": ObjectId(answer_id)})
        if not answer:
            return None
        answer["id"] = str(answer.pop("_id"))
        return AnswerSchema(**answer)
    
    async def list_answers(self) -> List[AnswerSchema]:
        cursor = self.collection.find()
        answers = await cursor.to_list(length=100)
        return [AnswerSchema(**{**ans, "id": str(ans["_id"])}) for ans in answers]
    
    async def get_answer_from_question_id(
        self, question_id: str
    ) -> List[AnswerSchema]:
        cursor = self.collection.find({"id_question": question_id})
        answers = await cursor.to_list(length=100)
        return [AnswerSchema(**{**ans, "id": str(ans["_id"])}) for ans in answers]
    
    async def delete_answer(self, answer_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(answer_id)})
        return result.deleted_count == 1
    
    async def get_answer_by_patient_id(
        self, patient_id: str
    ) -> List[AnswerSchema]:
        cursor = self.collection.find({"id_patient": patient_id})
        answers = await cursor.to_list(length=100)
        return [AnswerSchema(**{**ans, "id": str(ans["_id"])}) for ans in answers]