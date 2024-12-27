from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.patient import PatientCreate, PatientSchema


class PatientBase(ABC):
    @abstractmethod
    async def create_patient(self, patient_data: PatientCreate) -> PatientSchema:
        pass

    @abstractmethod
    async def get_patient(self, patient_id: str) -> Optional[PatientSchema]:
        pass

    @abstractmethod
    async def update_patient(
        self, patient_id: str, update_data: PatientCreate
    ) -> Optional[PatientSchema]:
        pass

    @abstractmethod
    async def delete_patient(self, patient_id: str) -> bool:
        pass

    @abstractmethod
    async def list_patients(self) -> List[PatientSchema]:
        pass

    @abstractmethod
    async def get_by_name(self, name: str) -> Optional[PatientSchema]:
        pass
