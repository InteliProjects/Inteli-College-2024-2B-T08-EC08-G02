from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    SECRET_KEY: str = "oii"  # Field(..., env="SECRET_KEY")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    class Config:
        env_file = ".env"


settings = Settings()
