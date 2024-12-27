# app/endpoints/robot_schedules.py
from typing import List
from fastapi import APIRouter, HTTPException
from app.schemas.robotschedule import (
    RobotScheduleCreate,
    RobotScheduleSchema,
    RobotScheduleResponse
)
from app.services.robot_schedule.robot_schedule import RobotScheduleService
from app.core.db import mongo_db

router = APIRouter(prefix="/robot_schedules", tags=["Robot Schedules"])

@router.post("/", response_model=RobotScheduleSchema)
async def create_schedule(schedule_data: RobotScheduleCreate):
    service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
    return await service.create_schedule(schedule_data)

@router.get("/{schedule_id}", response_model=RobotScheduleSchema)
async def get_schedule(schedule_id: str):
    service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
    schedule = await service.get_schedule(schedule_id)
    if not schedule:
        raise HTTPException(status_code=404, detail="Schedule not found")
    return schedule

@router.get("/", response_model=RobotScheduleResponse)
async def list_schedules():
    service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
    schedules = await service.list_schedules()
    return RobotScheduleResponse(schedules=schedules)

@router.delete("/{schedule_id}", response_model=dict)
async def delete_schedule(schedule_id: str):
    service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
    deleted = await service.delete_schedule(schedule_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Schedule not found")
    return {"deleted": deleted}

@router.get("/robot/{robot_id}", response_model=RobotScheduleResponse)
async def get_schedules_by_robot(robot_id: str):
    service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
    schedules = await service.get_schedules_by_robot(robot_id)
    if not schedules:
        raise HTTPException(status_code=404, detail="No schedules found for this robot")
    return RobotScheduleResponse(schedules=schedules)

@router.get("/patient/{patient_id}", response_model=RobotScheduleResponse)
async def get_schedules_by_patient(patient_id: str):
    service = RobotScheduleService(mongo_db.get_collection("robot_schedules"))
    schedules = await service.get_schedules_by_patient(patient_id)
    if not schedules:
        raise HTTPException(status_code=404, detail="No schedules found for this patient")
    return RobotScheduleResponse(schedules=schedules)
