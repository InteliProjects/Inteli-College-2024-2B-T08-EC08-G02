from abc import ABC, abstractmethod
from typing import Optional, List
from app.schemas.question import QuestionSchema
from app.schemas.answer import AnswerCreate

class AssistantBase(ABC):
    @abstractmethod
    async def transcribe_audio(self, file_path: str) -> Optional[str]:
        pass

    @abstractmethod
    async def send_message_to_assistant(self, text: str) -> Optional[str]:
        pass

    @abstractmethod
    async def run_anamnese(self, questions: List[QuestionSchema], patient_id: str):
        pass

    @abstractmethod
    async def save_question(self, question: QuestionSchema) -> None:
        pass

    @abstractmethod
    async def save_answer(self, answer: AnswerCreate) -> None:
        pass

    @abstractmethod
    async def fetch_next_question(self, context: str) -> QuestionSchema:
        pass
