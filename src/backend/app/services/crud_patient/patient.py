from app.services.crud_patient.base import PatientBase
from app.schemas.patient import PatientCreate, PatientSchema
from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional


class PatientService(PatientBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def create_patient(self, patient_data: PatientCreate) -> PatientSchema:
        data = patient_data.dict()
        result = await self.collection.insert_one(data)
        created_patient = await self.collection.find_one({"_id": result.inserted_id})
        created_patient["id"] = str(created_patient.pop("_id"))
        return PatientSchema(**created_patient)

    async def get_patient(self, patient_id: str) -> Optional[PatientSchema]:
        patient = await self.collection.find_one({"_id": ObjectId(patient_id)})
        if not patient:
            return None
        patient["id"] = str(patient.pop("_id"))
        return PatientSchema(**patient)

    async def list_patients(self) -> List[PatientSchema]:
        cursor = self.collection.find()
        patients = await cursor.to_list(length=100)
        return [PatientSchema(**{**pat, "id": str(pat["_id"])}) for pat in patients]

    async def update_patient(
        self, patient_id: str, update_data: PatientCreate
    ) -> Optional[PatientSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}

        result = await self.collection.update_one(
            {"_id": ObjectId(patient_id)}, {"$set": data}
        )

        if result.modified_count == 0:
            return None

        updated_patient = await self.collection.find_one({"_id": ObjectId(patient_id)})
        updated_patient["id"] = str(updated_patient.pop("_id"))
        return PatientSchema(**updated_patient)

    async def get_by_name(self, name: str) -> Optional[PatientSchema]:
        patient = await self.collection.find_one({"name": name})
        if not patient:
            return None
        patient["id"] = str(patient.pop("_id"))
        return PatientSchema(**patient)

    async def delete_patient(self, patient_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(patient_id)})
        return result.deleted_count == 1

    async def get_patient_by_professional_id(
        self, professional_id: str
    ) -> List[PatientSchema]:
        cursor = self.collection.find({"professional_id": professional_id})
        patients = await cursor.to_list(length=100)
        return [PatientSchema(**{**pat, "id": str(pat["_id"])}) for pat in patients]

    async def associate_patient_to_professional(
        self, patient_id: str, professional_id: str
    ) -> Optional[PatientSchema]:
        update_data = {"professional_id": professional_id}

        result = await self.collection.update_one(
            {"_id": ObjectId(patient_id)}, {"$set": update_data}
        )

        if result.modified_count == 0:
            return None

        updated_patient = await self.collection.find_one({"_id": ObjectId(patient_id)})
        updated_patient["id"] = str(updated_patient.pop("_id"))
        return PatientSchema(**updated_patient)

    async def dessasociate_patient_from_professional(
            self, patient_id: str
    ) -> Optional[PatientSchema]:
        update_data = {"professional_id": None}

        result = await self.collection.update_one(
            {"_id": ObjectId(patient_id)}, {"$set": update_data}
        )

        if result.modified_count == 0:
            return None

        updated_patient = await self.collection.find_one({"_id": ObjectId(patient_id)})
        updated_patient["id"] = str(updated_patient.pop("_id"))
        return PatientSchema(**updated_patient)
    