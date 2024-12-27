# app/services/robot_schedules/service.py
from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional
from datetime import datetime
from app.schemas.robotschedule import RobotScheduleCreate, RobotScheduleSchema

class RobotScheduleService:
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def create_schedule(self, schedule_data: RobotScheduleCreate) -> RobotScheduleSchema:
        data = schedule_data.dict()
        data["created_at"] = datetime.utcnow()
        data["updated_at"] = datetime.utcnow()
        result = await self.collection.insert_one(data)
        created_schedule = await self.collection.find_one({"_id": result.inserted_id})
        created_schedule["id"] = str(created_schedule.pop("_id"))
        return RobotScheduleSchema(**created_schedule)

    async def get_schedule(self, schedule_id: str) -> Optional[RobotScheduleSchema]:
        schedule = await self.collection.find_one({"_id": ObjectId(schedule_id)})
        if not schedule:
            return None
        schedule["id"] = str(schedule.pop("_id"))
        return RobotScheduleSchema(**schedule)

    async def list_schedules(self) -> List[RobotScheduleSchema]:
        cursor = self.collection.find()
        schedules = await cursor.to_list(length=1000)
        return [
            RobotScheduleSchema(
                **{**s, "id": str(s["_id"])}
            ) for s in schedules
        ]

    async def delete_schedule(self, schedule_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(schedule_id)})
        return result.deleted_count == 1

    async def get_schedules_by_robot(self, robot_id: str) -> List[RobotScheduleSchema]:
        cursor = self.collection.find({"robot_id": robot_id})
        schedules = await cursor.to_list(length=1000)
        return [
            RobotScheduleSchema(
                **{**s, "id": str(s["_id"])}
            ) for s in schedules
        ]

    async def get_schedules_by_patient(self, patient_id: str) -> List[RobotScheduleSchema]:
        cursor = self.collection.find({"patient_id": patient_id})
        schedules = await cursor.to_list(length=1000)
        return [
            RobotScheduleSchema(
                **{**s, "id": str(s["_id"])}
            ) for s in schedules
        ]

    async def get_next_schedule(self):
        now = datetime.now()
        next_schedule = await self.collection.find_one(
            {"scheduled_time": {"$gte": now}},
            sort=[("scheduled_time", 1)]
        )
        return next_schedule