from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.question import QuestionCreate, QuestionSchema

class QuestionBase(ABC):
    @abstractmethod
    async def create_question(self, question_data: QuestionCreate) -> QuestionSchema:
        pass
    
    @abstractmethod
    async def get_question(self, question_id: str) -> Optional[QuestionSchema]:
        pass

    @abstractmethod
    async def update_question(
        self, question_id: str, update_data: QuestionCreate
    ) -> Optional[QuestionSchema]:
        pass

    @abstractmethod
    async def list_questions(self) -> List[QuestionSchema]:
        pass

    @abstractmethod
    async def delete_question(self, question_id: str) -> bool:
        pass

    @abstractmethod
    async def get_question_by_patient_id(
        self, patient_id: str
    ) -> List[QuestionSchema]:
        pass