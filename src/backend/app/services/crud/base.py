from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.professional import ProfessionalCreate, ProfessionalSchema


class ProfessionalBase(ABC):
    @abstractmethod
    async def create_professional(
        self, professional_data: ProfessionalCreate
    ) -> ProfessionalSchema:
        pass

    @abstractmethod
    async def get_professional(
        self, professional_id: str
    ) -> Optional[ProfessionalSchema]:
        pass

    @abstractmethod
    async def update_professional(
        self, professional_id: str, update_data: ProfessionalCreate
    ) -> Optional[ProfessionalSchema]:
        pass

    @abstractmethod
    async def delete_professional(self, professional_id: int) -> bool:
        pass

    @abstractmethod
    async def list_professionals(self) -> List[ProfessionalSchema]:
        pass
