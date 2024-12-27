---
title: Estrutura do Backend
sidebar_position: 1
---

## **1.0** Introdução

Este documento apresenta uma visão abrangente sobre a estrutura do backend do projeto, detalhando sua concepção e funcionamento. Desenvolvido com foco em **alta escalabilidade** e **modularidade**, o backend foi projetado para permitir a integração de novas funcionalidades de forma ágil e sem comprometer a estabilidade do sistema existente.

A arquitetura é composta por diversos módulos que interagem de maneira coordenada para realizar as operações necessárias ao pleno funcionamento do sistema.

Ao longo deste documento, serão explorados os principais módulos e componentes do backend, bem como a estrutura geral da arquitetura, destacando os princípios que orientaram seu desenvolvimento.

## **2.0** Estrutura de pastas do backend

Abaixo será apresentada a estrutura de pastas do backend do projeto, nos próximos tópicos cada pasta será detalhada mais a fundo.

```bash
.
├── app
│   ├── __pycache__
│   ├── api
│   │   └── endpoints
│   │       ├── __init__.py
│   │       ├── __pycache__
│   │       ├── auth.py
│   │       ├── crud.py
│   │       ├── llm.py
│   │       ├── robot.py
│   │       └── robot_tasks.py
│   ├── app.log
│   ├── core
│   │   ├── __init__.py
│   │   ├── __pycache__
│   │   ├── config.py
│   │   └── db.py
│   ├── main.py
│   ├── schemas
│   │   ├── __pycache__
│   │   ├── access.py
│   │   ├── answer.py
│   │   ├── patient.py
│   │   ├── professional.py
│   │   ├── question.py
│   │   ├── robottask.py
│   │   └── rooms.py
│   ├── services
│   │   ├── __init__.py
│   │   ├── __pycache__
│   │   ├── crud
│   │   │   ├── __pycache__
│   │   │   ├── base.py
│   │   │   └── professional.py
│   │   ├── llm
│   │   │   ├── base.py
│   │   │   └── llm.py
│   │   ├── login
│   │   │   ├── __pycache__
│   │   │   ├── auth_service.py
│   │   │   └── base.py
│   │   └── robot
│   │       └── robot.py
│   └── utils
│       ├── __init__.py
│       ├── __pycache__
│       └── logger.py
└── requirements.txt
```

## **3.0** Principais pastas e módulos

O backend do projeto é organizado em diversas pastas, cada uma responsável por um conjunto específico de funcionalidades. A seguir, serão apresentadas as principais pastas e módulos do backend, destacando suas responsabilidades e interações.

### **3.1** `api/endpoints/`

Dentro desta pasta estão localizados os arquivos .py que contém as rotas relacionadas a cada parte, como por exemplo, a rota relacionada a autenticação, usuários, etc.

Segue abaixo o código de exemplo do arquivo de rota de autenticação:

```python
from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException
from fastapi.security import OAuth2PasswordRequestForm
from app.utils.logger import get_logger
from datetime import timedelta

from app.schemas.professional import ProfessionalCreate, ProfessionalLogin, Token
from app.services.login.auth_service import AuthService
from app.services.crud.professional import ProfessionalService

from app.core.config import Settings

from app.core.db import mongo_db

router = APIRouter(prefix="/auth", tags=["Authentication"])
logger = get_logger()
auth_service = AuthService()
professional_service = ProfessionalService(mongo_db.get_collection("professionals"))


@router.post("/register", response_model=Token)
async def register_professional(professional: ProfessionalCreate):
    existing_professional = await professional_service.get_by_name(professional.name)
    if existing_professional:
        raise HTTPException(status_code=400, detail="Professional already registered")

    new_professional = await professional_service.create_professional(professional)

    access_token = auth_service.create_access_token(
        data={"sub": new_professional.name, "role": new_professional.role}
    )
    return Token(access_token=access_token)
```

:::warning
O mesmo tem apenas um rota afim de exemplificar, porém, existem diversas outras rotas dentro do projeto. Para saber mais a respeito da parte de autenticação, cheque a documentação [**aqui**](/category/backend).
:::

No código acima, as principais partes são o import do logger, schemas, services e configurações, além da criação da rota de registro de profissionais.

Mais abaixo serão descritos o que são cada um desses imports e por que os mesmos estão sendo utilizados.

:::tip
Foi escolhido está estrutura de pastas para facilitar a organização e manutenção do código, além de permitir a escalabilidade do projeto.
:::

### **3.2** `core/`

Dentro desta pasta estão localizados os arquivos que contém as configurações principais do projeto, como por exemplo, as configurações do banco de dados, configurações de segurança, etc.

Segue abaixo as configurações do banco de dados dentro de `core/db.py`:

```python
from motor.motor_asyncio import AsyncIOMotorClient, AsyncIOMotorCollection

MONGO_URI = "mongodb://root:password@localhost:27017"
DB_NAME = "cora"


class MongoDB:
    def __init__(self, uri: str, db_name: str):
        self.client = AsyncIOMotorClient(uri)
        self.db = self.client[db_name]

    def get_collection(self, collection_name: str) -> AsyncIOMotorCollection:
        return self.db[collection_name]

    async def close(self):
        self.client.close()

mongo_db = MongoDB(MONGO_URI, DB_NAME)
```

Nela é possível ver a configuração do banco de dados, onde é definido o `MONGO_URI` e o `DB_NAME`, além da classe `MongoDB` que é responsável por criar a conexão com o banco de dados e que é exportada mais para frente.

Há também as configurações gerais do projeto, como por exemplo, as configurações de segurança, que são definidas em `core/config.py`:

```python
from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    SECRET_KEY: str = Field(..., env="SECRET_KEY")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    class Config:
        env_file = ".env"


settings = Settings()
```

Dentro dela é possível ver a definição da classe `Settings` que é responsável por definir as configurações do projeto, neste caso até então estão sendo definidas apenas as configurações relacionadas ao token JWT, será falado sobre o mesmo mais para frente.

### **3.3** `schemas/`

Dentro desta pasta estão localizados os arquivos que contém os schemas do projeto, que são responsáveis por definir a estrutura dos dados que serão utilizados no projeto.

O mesmo é essencial para definir a estrutura da base de dados, levando em consideração que estamos utilizando o MongoDB. Também serve para deixar o código altamente tipado, facilitando a manutenção e entendimento do mesmo.

Segue abaixo um exemplo de schema de profissional:

```python
from typing import Optional, List, Literal
from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime


class ProfessionalCreate(BaseModel):
    name: str = Field(..., max_length=255, description="The name of the professional")
    email: Optional[str] = Field(None, description="The email of the professional")
    password: str = Field(..., description="The password of the professional")
    role: Literal["doctor", "nurse", "admin"] = Field(
        ..., description="The role of the professional"
    )


class ProfessionalSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str = Field(..., description="The ID of the professional")
    name: str
    role: str
    email: Optional[str]
    password: str
    created_at: datetime = Field(default_factory=datetime.now)


class ProfessionalLogin(BaseModel):
    name: str
    password: str


class Token(BaseModel):
    access_token: str
    token_type: str = "bearer"


class ProfessionalResponse(BaseModel):
    professionals: List[ProfessionalSchema]
```

Há três `Schemas` principais, o `ProfessionalCreate`, que é responsável por definir a estrutura de criação de um profissional, o `ProfessionalSchema`, que é responsável por definir a estrutura de um profissional, e o `ProfessionalResponse`, que é responsável por definir a estrutura de resposta de uma lista de profissionais.

Todos os outros Schemas seguirão a estrutura deste, removendo apenas as classes `ProfessionalLogin` e `Token`, que são específicas para a autenticação de um usuário.

### **3.4** `services/`

Dentro desta pasta estão localizados os arquivos que contém os serviços do projeto, que são responsáveis por realizar as operações necessárias para o funcionamento do sistema, os mesmos são responsáveis por realizar a ponte entre os `endpoints` e as ações que devem ser realizadas.

Cada serviço deve possuir tanto o arquivo com a lógica, quanto um arquivo chamado `base.py` que servirá como as classes abstratas que serão herdadas pelos serviços.

Segue abaixo um exemplo de serviço de CRUD:

Primeiramente segue abaixo o código do arquivo onde a lógica dos serviços estará presente:

```python
from app.services.crud.base import ProfessionalBase
from app.schemas.professional import ProfessionalCreate, ProfessionalSchema
from app.services.login.auth_service import AuthService
from motor.motor_asyncio import AsyncIOMotorCollection
from bson.objectid import ObjectId
from typing import List, Optional


class ProfessionalService(ProfessionalBase):
    def __init__(self, collection: AsyncIOMotorCollection):
        self.collection = collection
        self.auth_service = AuthService()

    async def create_professional(
        self, professional_data: ProfessionalCreate
    ) -> ProfessionalSchema:
        hashed_password = self.auth_service.hash_password(professional_data.password)

        data = professional_data.dict()
        data["password"] = hashed_password
        result = await self.collection.insert_one(data)
        created_professional = await self.collection.find_one(
            {"_id": result.inserted_id}
        )
        created_professional["id"] = str(created_professional.pop("_id"))
        return ProfessionalSchema(**created_professional)

    async def get_professional(
        self, professional_id: str
    ) -> Optional[ProfessionalSchema]:
        professional = await self.collection.find_one(
            {"_id": ObjectId(professional_id)}
        )
        if not professional:
            return None
        professional["id"] = str(professional.pop("_id"))
        return ProfessionalSchema(**professional)

    async def list_professionals(self) -> List[ProfessionalSchema]:
        cursor = self.collection.find()
        professionals = await cursor.to_list(length=100)
        return [
            ProfessionalSchema(**{**prof, "id": str(prof["_id"])})
            for prof in professionals
        ]

    async def update_professional(
        self, professional_id: str, update_data: ProfessionalCreate
    ) -> Optional[ProfessionalSchema]:
        data = {k: v for k, v in update_data.dict(exclude_unset=True).items()}

        if "password" in data:
            data["password"] = self.auth_service.hash_password(data["password"])

        result = await self.collection.update_one(
            {"_id": ObjectId(professional_id)}, {"$set": data}
        )

        if result.modified_count == 0:
            return None

        updated_professional = await self.collection.find_one(
            {"_id": ObjectId(professional_id)}
        )
        updated_professional["id"] = str(updated_professional.pop("_id"))
        return ProfessionalSchema(**updated_professional)

    async def get_by_name(self, name: str) -> Optional[ProfessionalSchema]:
        professional = await self.collection.find_one({"name": name})
        if not professional:
            return None
        professional["id"] = str(professional.pop("_id"))
        return ProfessionalSchema(**professional)

    async def delete_professional(self, professional_id: str) -> bool:
        result = await self.collection.delete_one({"_id": ObjectId(professional_id)})
        return result.deleted_count == 1
```

Neste arquivo é possível ver a lógica de criação, listagem, atualização, busca por nome e remoção de um profissional, além da importação da classe `ProfessionalBase` que é responsável por definir as classes abstratas que serão herdadas pelos serviços.

Agora segue abaixo o código do arquivo `base.py`:

```python
from abc import ABC, abstractmethod
from typing import List, Optional
from app.schemas.professional import ProfessionalCreate, ProfessionalSchema


class ProfessionalBase(ABC):
    @abstractmethod
    async def create_professional(
        self, professional_data: ProfessionalCreate
    ) -> ProfessionalSchema:
        pass

    @abstractmethod
    async def get_professional(
        self, professional_id: str
    ) -> Optional[ProfessionalSchema]:
        pass

    @abstractmethod
    async def update_professional(
        self, professional_id: str, update_data: ProfessionalCreate
    ) -> Optional[ProfessionalSchema]:
        pass

    @abstractmethod
    async def delete_professional(self, professional_id: int) -> bool:
        pass

    @abstractmethod
    async def list_professionals(self) -> List[ProfessionalSchema]:
        pass
```

Neste arquivo é possível ver a definição da classe abstrata `ProfessionalBase` que é responsável por definir as classes abstratas que serão herdadas pelos serviços.

Estas classes abstratas são essenciais para garantir que todos os serviços sigam a mesma estrutura, facilitando a manutenção e entendimento do código.

### **3.5** `utils/`

Dentro desta pasta estão localizados os arquivos que contém as funções utilitárias do projeto, que são responsáveis por realizar operações que são comuns a diversos módulos do projeto.

Até o momento da terceira Sprint do projeto, as funções utilitárias são responsáveis por realizar a criação de logs, porém, futuramente novas funções utilitárias poderão ser adicionadas caso seja necessário.

Segue abaixo o exemplo da função de logs:

```python
import logging
from logging.handlers import RotatingFileHandler
import os

log_directory = "app"
if not os.path.exists(log_directory):
    os.makedirs(log_directory)


log_file = os.path.join(log_directory, "app.log")
handler = RotatingFileHandler(log_file, maxBytes=10 * 1024 * 1024, backupCount=5)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)

logger = logging.getLogger("app_logger")
logger.setLevel(logging.INFO)
logger.addHandler(handler)


def get_logger():
    return logger
```

Neste arquivo é possível ver a definição da função `get_logger` que é responsável por criar um logger para o projeto, além da definição do arquivo de logs e do formato do log.

### **3.6** `main.py`

Por fim, o arquivo `main.py` é responsável por iniciar o servidor do projeto, nele é possível ver a importação dos arquivos de rotas, configurações, etc.

O mesmo é o orquestrador do projeto, sendo responsável por iniciar todos os módulos e garantir o correto funcionamento do sistema.

Segue abaixo o `main.py`:

```python
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from app.api.endpoints import crud
from app.api.endpoints import auth
from app.core.db import mongo_db  # Importa mongo_db


@asynccontextmanager
async def app_lifespan(app: FastAPI):
    print("Application startup")
    yield
    await mongo_db.close()
    print("Application shutdown")


app = FastAPI(lifespan=app_lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(crud.router, prefix="/api", tags=["crud"])
app.include_router(auth.router, prefix="/api", tags=["auth"])


@app.get("/")
async def root():
    return {"message": "Welcome to the CORA API"}
```

As principais operações que são realizadas neste arquivo são a importação dos arquivos de rotas, a definição do contexto de vida do aplicativo e a definição das rotas do aplicativo.

## **4.0** Conclusão

A estrutura do backend do projeto foi desenvolvida com foco em alta escalabilidade e modularidade, permitindo a integração de novas funcionalidades de forma ágil e sem comprometer a estabilidade do sistema existente.

A arquitetura é composta por diversos módulos que interagem de maneira coordenada para realizar as operações necessárias ao pleno funcionamento do sistema. A organização em pastas e módulos facilita a manutenção e evolução do código, garantindo a qualidade e robustez do sistema.

Ao longo deste documento, foram explorados os principais módulos e componentes do backend, destacando suas responsabilidades e interações. A estrutura do backend foi projetada para atender às necessidades do projeto, garantindo a eficiência e a confiabilidade do sistema.

:::info
Para saber mais a respeito de cada serviço, endpoints entre outros, cheque a documentação do Backend relacionda a Sprint 3 [**aqui**](/category/backend).
:::