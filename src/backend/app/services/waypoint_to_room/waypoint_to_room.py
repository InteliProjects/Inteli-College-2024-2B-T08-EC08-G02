from motor.motor_asyncio import AsyncIOMotorCollection
from typing import Optional
from app.schemas.waypointmap import WaypointToRoomSchema

class WaypointToRoomService:
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def get_waypoint_by_room_id(self, id_room: int) -> Optional[WaypointToRoomSchema]:
        mapping = await self.collection.find_one({"id_room": str(id_room)})  # Ensure id_room is a string in your database
        if not mapping:
            return None
        mapping["id"] = str(mapping.pop("_id"))
        return WaypointToRoomSchema(**mapping)