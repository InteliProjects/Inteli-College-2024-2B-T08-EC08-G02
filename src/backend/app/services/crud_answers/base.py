from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.answer import AnswerCreate, AnswerSchema

class AnswerBase(ABC):
    @abstractmethod
    async def create_answer(self, answer_data: AnswerCreate) -> AnswerSchema:
        pass
    
    @abstractmethod
    async def get_answer(self, answer_id: str) -> Optional[AnswerSchema]:
        pass

    @abstractmethod
    async def list_answers(self) -> List[AnswerSchema]:
        pass

    @abstractmethod
    async def get_answer_from_question_id(
        self, question_id: str
    ) -> List[AnswerSchema]:
        pass

    @abstractmethod
    async def delete_answer(self, answer_id: str) -> bool:
        pass

    @abstractmethod
    async def get_answer_by_patient_id(
        self, patient_id: str
    ) -> List[AnswerSchema]:
        pass