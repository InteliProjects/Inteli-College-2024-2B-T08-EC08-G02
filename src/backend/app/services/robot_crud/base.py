from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.robots import RobotCreate, RobotSchema


class RobotBase(ABC):
    @abstractmethod
    async def create_robot(self, robot_data: RobotCreate) -> RobotSchema:
        pass

    @abstractmethod
    async def get_robot(self, robot_id: str) -> Optional[RobotSchema]:
        pass

    @abstractmethod
    async def list_robots(self) -> List[RobotSchema]:
        pass

    @abstractmethod
    async def update_robot(
        self, robot_id: str, update_data: RobotCreate
    ) -> Optional[RobotSchema]:
        pass

    @abstractmethod
    async def delete_robot(self, robot_id: str) -> bool:
        pass

    @abstractmethod
    async def get_robot_by_status(self, status: str) -> List[RobotSchema]:
        pass

    @abstractmethod
    async def get_robot_by_location(self, location: str) -> List[RobotSchema]:
        pass
