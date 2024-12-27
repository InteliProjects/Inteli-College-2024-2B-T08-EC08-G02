# app/main.py

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from app.api.endpoints import (
    crud, auth, crud_patient, crud_question, crud_answer,
    map_crud, robot_crud, robot_schedule, scheduler_organizer, llm
)
from app.core.db import mongo_db
from app.utils.logger import get_logger
from app.utils.rclpyman import RclpyManager


logger = get_logger()

@asynccontextmanager
async def app_lifespan(app: FastAPI):
    logger.info("Application startup")

    # Initialize rclpy using RclpyManager Singleton
    rclpy_manager = RclpyManager()
    rclpy_manager.init()

    yield

    # Shutdown rclpy
    rclpy_manager.shutdown()

    # Close the database connection
    await mongo_db.close()
    logger.info("Application shutdown")

app = FastAPI(lifespan=app_lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust as needed for security
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include all routers
app.include_router(crud.router, prefix="/api", tags=["CRUD"])
app.include_router(auth.router, prefix="/api", tags=["Authentication"])
app.include_router(crud_patient.router, prefix="/api", tags=["Patients CRUD"])
app.include_router(crud_question.router, prefix="/api", tags=["Questions CRUD"])
app.include_router(crud_answer.router, prefix="/api", tags=["Answers CRUD"])
app.include_router(llm.router, prefix="/api", tags=["LLM"])
app.include_router(map_crud.router, prefix="/api", tags=["Waypoint to Room CRUD"])
app.include_router(robot_crud.router, prefix="/api", tags=["Robots CRUD"])
app.include_router(robot_schedule.router, prefix="/api", tags=["Robot Schedules"])
app.include_router(scheduler_organizer.router, prefix="/api", tags=["Scheduler Organizer"])

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.error(
        f"Unhandled error occurred: {exc}"
        f" [Path: {request.url.path}, Method: {request.method}]",
        exc_info=True
    )
    return JSONResponse(
        status_code=500,
        content={"message": "Internal Server Error"}
    )

@app.get("/")
async def root():
    return {"message": "Welcome to the CORA API"}
