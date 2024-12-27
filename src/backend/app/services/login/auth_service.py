from datetime import datetime, timedelta
from fastapi import HTTPException, Depends
from fastapi.security import OAuth2PasswordBearer
from jose import JWTError, jwt
from passlib.context import CryptContext
from app.core.config import settings
from app.services.login.base import AuthBase
from typing import Optional

# TODO: Change this to api/auth/login, was used login-test for testing in Swagger
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/auth/login-test")


class AuthService(AuthBase):
    pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

    def hash_password(self, password: str) -> str:
        return self.pwd_context.hash(password)

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        return self.pwd_context.verify(plain_password, hashed_password)

    def create_access_token(
        self, data: dict, expires_delta: Optional[timedelta] = None
    ) -> str:
        to_encode = data.copy()
        expire = (
            datetime.utcnow() + expires_delta
            if expires_delta
            else datetime.utcnow()
            + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
        )
        to_encode.update({"exp": expire})
        return jwt.encode(to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM)

    async def get_current_user(self, token: str = Depends(oauth2_scheme)):
        """Decodes the JWT token and retrieves the current user payload."""
        payload = self.decode_access_token(token)
        if payload is None:
            raise HTTPException(
                status_code=401,
                detail="Could not validate the credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return payload

    async def get_current_user_role(
        self, required_role: str, token: str = Depends(oauth2_scheme)
    ):
        """Validates if the current user has the required role."""
        payload = await self.get_current_user(token)
        user_role = payload.get("role")

        if user_role != required_role:
            raise HTTPException(
                status_code=403,
                detail="You do not have permission to access this resource",
            )

        return payload

    def decode_access_token(self, token: str) -> Optional[dict]:
        """Decodes the JWT token and checks for the presence of 'sub'."""
        try:
            payload = jwt.decode(
                token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM]
            )
            return payload if "sub" in payload else None
        except JWTError:
            return None

    def require_role(self, required_role: str):
        """Creates a dependency to verify the user role in routes."""

        async def role_dependency(token: str = Depends(oauth2_scheme)):
            return await self.get_current_user_role(required_role, token)

        return role_dependency
