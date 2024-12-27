from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional
from app.schemas.waypointmap import WaypointToRoomCreate, WaypointToRoomSchema
from app.services.map_crud.base import WaypointBase

class WaypointToRoomService(WaypointBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def create_mapping(self, mapping_data: WaypointToRoomCreate) -> WaypointToRoomSchema:
        data = mapping_data.dict()
        result = await self.collection.insert_one(data)
        created_mapping = await self.collection.find_one({"_id": result.inserted_id})
        created_mapping["id"] = str(created_mapping.pop("_id"))
        return WaypointToRoomSchema(**created_mapping)
    
    async def get_mapping(self, mapping_id: str) -> Optional[WaypointToRoomSchema]:
        mapping = await self.collection.find_one({"_id": ObjectId(mapping_id)})
        if not mapping:
            return None
        mapping["id"] = str(mapping.pop("_id"))
        return WaypointToRoomSchema(**mapping)
    
    async def list_mappings(self) -> List[WaypointToRoomSchema]:
        cursor = self.collection.find()
        mappings = await cursor.to_list(length=100)
        return [WaypointToRoomSchema(**{**map, "id": str(map["_id"])}) for map in mappings]
    
    async def update_mapping(
        self, mapping_id: str, update_data: WaypointToRoomCreate
    ) -> Optional[WaypointToRoomSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}
        
        result = await self.collection.update_one(
            {"_id": ObjectId(mapping_id)}, {"$set": data}
        )
        
        if result.modified_count == 0:
            return None
        
        updated_mapping = await self.collection.find_one({"_id": ObjectId(mapping_id)})
        updated_mapping["id"] = str(updated_mapping.pop("_id"))
        return WaypointToRoomSchema(**updated_mapping)
    
    async def delete_mapping(self, mapping_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(mapping_id)})
        return result.deleted_count == 1
    
    async def update_mapping_by_waypoint_id(
        self, waypoint_id: int, update_data: WaypointToRoomCreate
    ) -> Optional[WaypointToRoomSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}
        
        result = await self.collection.update_one(
            {"waypoint_id": waypoint_id}, {"$set": data}
        )
        
        if result.modified_count == 0:
            return None
        
        updated_mapping = await self.collection.find_one({"waypoint_id": waypoint_id})
        updated_mapping["id"] = str(updated_mapping.pop("_id"))
        return WaypointToRoomSchema(**updated_mapping)