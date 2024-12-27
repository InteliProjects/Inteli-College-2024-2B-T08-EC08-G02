from typing import List
from fastapi import APIRouter, Depends, HTTPException

from app.schemas.waypointmap import (
    WaypointToRoomCreate,
    WaypointToRoomSchema,
    WaypointToRoomResponse,
)

from app.services.map_crud.map_crud import WaypointToRoomService
from app.core.db import mongo_db

router = APIRouter(prefix="/waypoint-to-room", tags=["Waypoint to Room CRUD"])

@router.post("/", response_model=WaypointToRoomSchema)
async def create_mapping(mapping_data: WaypointToRoomCreate):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    return await service.create_mapping(mapping_data)

@router.get("/{mapping_id}", response_model=WaypointToRoomSchema)
async def get_mapping(mapping_id: str):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    mapping = await service.get_mapping(mapping_id)
    if not mapping:
        raise HTTPException(status_code=404, detail="Mapping not found")
    return mapping

@router.get("/", response_model=WaypointToRoomResponse)
async def list_mappings():
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    mappings = await service.list_mappings()
    return WaypointToRoomResponse(mappings=mappings)

@router.put("/{mapping_id}", response_model=WaypointToRoomSchema)
async def update_mapping(mapping_id: str, update_data: WaypointToRoomCreate):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    mapping = await service.update_mapping(mapping_id, update_data)
    if not mapping:
        raise HTTPException(status_code=404, detail="Mapping not found")
    return mapping

@router.delete("/{mapping_id}", response_model=dict)
async def delete_mapping(mapping_id: str):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    deleted = await service.delete_mapping(mapping_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Mapping not found")
    return {"deleted": deleted}