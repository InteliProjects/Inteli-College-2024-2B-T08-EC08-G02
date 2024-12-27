# app/services/robot_schedules/base.py
from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.robotschedule import RobotScheduleCreate, RobotScheduleSchema


class RobotScheduleBase(ABC):
    @abstractmethod
    async def create_schedule(self, schedule_data: RobotScheduleCreate) -> RobotScheduleSchema:
        pass

    @abstractmethod
    async def get_schedule(self, schedule_id: str) -> Optional[RobotScheduleSchema]:
        pass

    @abstractmethod
    async def list_schedules(self) -> List[RobotScheduleSchema]:
        pass

    @abstractmethod
    async def delete_schedule(self, schedule_id: str) -> bool:
        pass

    @abstractmethod
    async def get_schedules_by_robot(self, robot_id: str) -> List[RobotScheduleSchema]:
        pass

    @abstractmethod
    async def get_schedules_by_patient(self, patient_id: str) -> List[RobotScheduleSchema]:
        pass
    
