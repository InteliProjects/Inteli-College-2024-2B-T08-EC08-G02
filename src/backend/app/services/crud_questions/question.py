from app.schemas.question import QuestionCreate, QuestionSchema
from app.services.crud_questions.base import QuestionBase
from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional


class QuestionService(QuestionBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def create_question(self, question_data: QuestionCreate) -> QuestionSchema:
        data = question_data.dict()
        result = await self.collection.insert_one(data)
        created_question = await self.collection.find_one({"_id": result.inserted_id})
        if not created_question:
            raise ValueError("Failed to create question")
        created_question["id"] = str(created_question.pop("_id"))
        return QuestionSchema(**created_question)

    async def get_question(self, question_id: str) -> Optional[QuestionSchema]:
        try:
            question = await self.collection.find_one({"_id": ObjectId(question_id)})
        except Exception as e:
            raise ValueError(f"Invalid question ID format: {e}")
        if not question:
            return None
        question["id"] = str(question.pop("_id"))
        return QuestionSchema(**question)

    async def list_questions(self) -> List[QuestionSchema]:
        cursor = self.collection.find()
        questions = await cursor.to_list(length=100)
        return [
            QuestionSchema(
                **{
                    **ques,
                    "id": str(ques["_id"]),
                    "patient_id": ques.get("patient_id", "unknown"),  # Default value
                }
            )
            for ques in questions
        ]


    async def update_question(
        self, question_id: str, update_data: QuestionCreate
    ) -> Optional[QuestionSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}
        try:
            result = await self.collection.update_one(
                {"_id": ObjectId(question_id)}, {"$set": data}
            )
        except Exception as e:
            raise ValueError(f"Invalid question ID format: {e}")
        
        if result.modified_count == 0:
            return None
        
        updated_question = await self.collection.find_one({"_id": ObjectId(question_id)})
        if not updated_question:
            return None
        updated_question["id"] = str(updated_question.pop("_id"))
        return QuestionSchema(**updated_question)

    async def delete_question(self, question_id: str) -> bool:
        try:
            result = await self.collection.delete_one({"_id": ObjectId(question_id)})
        except Exception as e:
            raise ValueError(f"Invalid question ID format: {e}")
        return result.deleted_count == 1

    async def get_question_by_patient_id(
        self, patient_id: str
    ) -> List[QuestionSchema]:
        cursor = self.collection.find({"patient_id": patient_id})
        questions = await cursor.to_list(length=100)
        return [
            QuestionSchema(**{**ques, "id": str(ques["_id"])})
            for ques in questions if ques
        ]
