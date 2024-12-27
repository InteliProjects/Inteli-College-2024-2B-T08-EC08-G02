from app.services.crud.base import ProfessionalBase
from app.schemas.professional import ProfessionalCreate, ProfessionalSchema
from app.services.login.auth_service import AuthService
from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional


class ProfessionalService(ProfessionalBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection
        self.auth_service = AuthService()

    async def create_professional(
        self, professional_data: ProfessionalCreate
    ) -> ProfessionalSchema:
        hashed_password = self.auth_service.hash_password(professional_data.password)

        data = professional_data.dict()
        data["password"] = hashed_password
        result = await self.collection.insert_one(data)
        created_professional = await self.collection.find_one(
            {"_id": result.inserted_id}
        )
        created_professional["id"] = str(created_professional.pop("_id"))
        return ProfessionalSchema(**created_professional)

    async def get_professional(
        self, professional_id: str
    ) -> Optional[ProfessionalSchema]:
        professional = await self.collection.find_one(
            {"_id": ObjectId(professional_id)}
        )
        if not professional:
            return None
        professional["id"] = str(professional.pop("_id"))
        return ProfessionalSchema(**professional)

    async def list_professionals(self) -> List[ProfessionalSchema]:
        cursor = self.collection.find()
        professionals = await cursor.to_list(length=100)
        return [
            ProfessionalSchema(**{**prof, "id": str(prof["_id"])})
            for prof in professionals
        ]

    async def update_professional(
        self, professional_id: str, update_data: ProfessionalCreate
    ) -> Optional[ProfessionalSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}

        if "password" in data:
            data["password"] = self.auth_service.hash_password(data["password"])

        result = await self.collection.update_one(
            {"_id": ObjectId(professional_id)}, {"$set": data}
        )

        if result.modified_count == 0:
            return None

        updated_professional = await self.collection.find_one(
            {"_id": ObjectId(professional_id)}
        )
        updated_professional["id"] = str(updated_professional.pop("_id"))
        return ProfessionalSchema(**updated_professional)

    async def get_by_name(self, name: str) -> Optional[ProfessionalSchema]:
        professional = await self.collection.find_one({"name": name})
        if not professional:
            return None
        professional["id"] = str(professional.pop("_id"))
        return ProfessionalSchema(**professional)

    async def delete_professional(self, professional_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(professional_id)})
        return result.deleted_count == 1
