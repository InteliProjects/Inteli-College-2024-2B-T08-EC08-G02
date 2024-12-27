from typing import List
from fastapi import APIRouter, HTTPException
from app.schemas.robots import (
    RobotCreate,
    RobotSchema,
    RobotResponse
)
from app.services.robot_crud.robot_crud import RobotService
from app.core.db import mongo_db

router = APIRouter(prefix="/robots", tags=["Robots CRUD"])


@router.post("/", response_model=RobotSchema)
async def create_robot(robot_data: RobotCreate):
    service = RobotService(mongo_db.get_collection("robots"))
    return await service.create_robot(robot_data)


@router.get("/{robot_id}", response_model=RobotSchema)
async def get_robot(robot_id: str):
    service = RobotService(mongo_db.get_collection("robots"))
    robot = await service.get_robot(robot_id)
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    return robot


@router.get("/", response_model=RobotResponse)
async def list_robots():
    service = RobotService(mongo_db.get_collection("robots"))
    robots = await service.list_robots()
    return RobotResponse(robots=robots)


@router.put("/{robot_id}", response_model=RobotSchema)
async def update_robot(robot_id: str, update_data: RobotCreate):
    service = RobotService(mongo_db.get_collection("robots"))
    robot = await service.update_robot(robot_id, update_data)
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    return robot


@router.delete("/{robot_id}", response_model=dict)
async def delete_robot(robot_id: str):
    service = RobotService(mongo_db.get_collection("robots"))
    deleted = await service.delete_robot(robot_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Robot not found")
    return {"deleted": deleted}


@router.get("/status/{status}", response_model=RobotResponse)
async def get_robots_by_status(status: str):
    service = RobotService(mongo_db.get_collection("robots"))
    robots = await service.get_robot_by_status(status)
    if not robots:
        raise HTTPException(status_code=404, detail="No robots with the specified status found")
    return RobotResponse(robots=robots)


@router.get("/location/{location}", response_model=RobotResponse)
async def get_robots_by_location(location: str):
    service = RobotService(mongo_db.get_collection("robots"))
    robots = await service.get_robot_by_location(location)
    if not robots:
        raise HTTPException(status_code=404, detail="No robots found at the specified location")
    return RobotResponse(robots=robots)
