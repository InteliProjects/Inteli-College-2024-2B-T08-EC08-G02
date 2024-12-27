from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.waypointmap import WaypointToRoomCreate, WaypointToRoomSchema


class WaypointBase(ABC):
    @abstractmethod
    async def create_mapping(self, mapping_data: WaypointToRoomCreate) -> WaypointToRoomSchema:
        pass

    @abstractmethod
    async def get_mapping(self, mapping_id: str) -> Optional[WaypointToRoomSchema]:
        pass

    @abstractmethod
    async def list_mappings(self) -> List[WaypointToRoomSchema]:
        pass

    @abstractmethod
    async def update_mapping(
        self, mapping_id: str, update_data: WaypointToRoomCreate
    ) -> Optional[WaypointToRoomSchema]:
        pass

    @abstractmethod
    async def delete_mapping(self, mapping_id: str) -> bool:
        pass

    @abstractmethod
    async def update_mapping_by_waypoint_id(
        self, waypoint_id: int, update_data: WaypointToRoomCreate
    ) -> Optional[WaypointToRoomSchema]:
        pass