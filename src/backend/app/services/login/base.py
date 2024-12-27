from abc import ABC, abstractmethod
from datetime import timedelta
from typing import Optional


class AuthBase(ABC):
    @abstractmethod
    def hash_password(self, password: str) -> str:
        pass

    @abstractmethod
    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        pass

    @abstractmethod
    def create_access_token(
        self, data: dict, expires_delta: Optional[timedelta] = None
    ) -> str:
        pass

    @abstractmethod
    def decode_access_token(self, token: str) -> Optional[dict]:
        pass

    @abstractmethod
    async def get_current_user_role(
        self, required_role: str, token: str
    ) -> Optional[dict]:
        pass
