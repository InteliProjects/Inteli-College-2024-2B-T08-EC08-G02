from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional
from app.schemas.robots import RobotCreate, RobotSchema
from app.services.robot_crud.base import RobotBase

class RobotService(RobotBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection

    async def create_robot(self, robot_data: RobotCreate) -> RobotSchema:
        data = robot_data.dict()
        result = await self.collection.insert_one(data)
        created_robot = await self.collection.find_one({"_id": result.inserted_id})
        created_robot["id"] = str(created_robot.pop("_id"))
        return RobotSchema(**created_robot)

    async def get_robot(self, robot_id: str) -> Optional[RobotSchema]:
        robot = await self.collection.find_one({"_id": ObjectId(robot_id)})
        if not robot:
            return None
        robot["id"] = str(robot.pop("_id"))
        return RobotSchema(**robot)

    async def list_robots(self) -> List[RobotSchema]:
        cursor = self.collection.find()
        robots = await cursor.to_list(length=100)
        return [RobotSchema(**{**robot, "id": str(robot["_id"])}) for robot in robots]

    async def update_robot(
        self, robot_id: str, update_data: RobotCreate
    ) -> Optional[RobotSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}

        result = await self.collection.update_one(
            {"_id": ObjectId(robot_id)}, {"$set": data}
        )

        if result.modified_count == 0:
            return None

        updated_robot = await self.collection.find_one({"_id": ObjectId(robot_id)})
        updated_robot["id"] = str(updated_robot.pop("_id"))
        return RobotSchema(**updated_robot)

    async def delete_robot(self, robot_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(robot_id)})
        return result.deleted_count == 1

    async def get_robot_by_status(self, status: str) -> List[RobotSchema]:
        cursor = self.collection.find({"status": status})
        robots = await cursor.to_list(length=100)
        return [RobotSchema(**{**robot, "id": str(robot["_id"])}) for robot in robots]

    async def get_robot_by_location(self, location: str) -> List[RobotSchema]:
        cursor = self.collection.find({"current_location": location})
        robots = await cursor.to_list(length=100)
        return [RobotSchema(**{**robot, "id": str(robot["_id"])}) for robot in robots]
