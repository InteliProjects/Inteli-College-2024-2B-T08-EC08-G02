---
title: Lógica do CRUD
sidebar_position: 1
---

## **1.0** Introdução

Este documento serve para entender como a lógica do CRUD para profissionais funciona no backend de nosso projeto, CORA. O CRUD é um acrônimo para Create, Read, Update e Delete, que são as quatro operações básicas utilizadas em bancos de dados relacionais. Neste documento, serão apresentados os principais conceitos e técnicas de implementação do CRUD no backend do projeto CORA, bem como a estrutura de dados utilizada e as operações disponíveis.

## **2.0** Estrutura de arquivos

A estrutura de arquivos do backend do projeto CORA é organizada de forma a facilitar a implementação e manutenção do CRUD. Os arquivos estão divididos em módulos, cada um responsável por uma parte específica do sistema. A estrutura de arquivos é a seguinte:

### **2.1** `api/endpoints/crud.py`

Este arquivo contém as rotas e controladores para as operações CRUD. Ele é responsável por receber as requisições HTTP, chamar as funções correspondentes e retornar as respostas adequadas.

Segue abaixo o mesmo:

```python
from typing import List
from fastapi import APIRouter, Depends
from fastapi.exceptions import HTTPException
from app.utils.logger import get_logger

from app.schemas.professional import (
    ProfessionalCreate,
    ProfessionalSchema,
    ProfessionalResponse,
)

from app.services.crud.professional import ProfessionalService

from app.core.db import mongo_db

router = APIRouter(prefix="/crud", tags=["CRUD"])
logger = get_logger()


@router.post("/", response_model=ProfessionalSchema)
async def create_professional(professional_data: ProfessionalCreate):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    return await service.create_professional(professional_data)


@router.get("/{professional_id}", response_model=ProfessionalSchema)
async def get_professional(professional_id: str):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professional = await service.get_professional(professional_id)
    if not professional:
        raise HTTPException(status_code=404, detail="Professional not found")
    return professional


@router.get("/", response_model=ProfessionalResponse)
async def list_professionals():
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professionals = await service.list_professionals()
    return ProfessionalResponse(professionals=professionals)


@router.put("/{professional_id}", response_model=ProfessionalSchema)
async def update_professional(professional_id: str, update_data: ProfessionalCreate):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professional = await service.update_professional(professional_id, update_data)
    if not professional:
        raise HTTPException(status_code=404, detail="Professional not found")
    return professional


@router.delete("/{professional_id}", response_model=dict)
async def delete_professional(professional_id: str):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    deleted = await service.delete_professional(professional_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Professional not found")
    return {"deleted": deleted}
```

Nele são definidas as rotas para as operações CRUD, bem como os controladores correspondentes. As rotas são definidas utilizando o decorador `@router.method`, que recebe como argumento o método HTTP correspondente (POST, GET, PUT, DELETE) e a URL da rota. Os controladores são funções assíncronas que recebem os dados da requisição, chamam os métodos correspondentes do serviço e retornam as respostas adequadas.

### **2.2** `schemas/professional.py`

Este arquivo contém os esquemas de dados utilizados para as operações CRUD. Ele define as estruturas de dados para os profissionais, incluindo os campos necessários para criar, ler, atualizar e deletar um profissional.

:::info
Nele também é definido a estrutura da coleção **Professionals** no banco de dados MongoDB. Para saber mais a respeito, clique [**aqui**](/Sprint%203/Backend/estrutura)
:::

Segue abaixo o mesmo:

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

Os esquemas de dados são definidos utilizando a classe `BaseModel` do Pydantic. Cada esquema define os campos necessários para criar, ler, atualizar e deletar um profissional, bem como os tipos de dados e validações necessárias. Os esquemas são utilizados para validar os dados recebidos nas requisições e para serializar os dados retornados nas respostas.

Nele, as classes mais importantes são, `ProfessionalCreate`, `ProfessionalSchema` e `ProfessionalResponse`. Pois são elas que definem a estrutura dos dados para as operações CRUD. Além da resposta de listagem de profissionais.

### **2.3** `services/crud/professional.py`

Este arquivo contém os serviços responsáveis por executar as operações CRUD. Ele é responsável por interagir com o banco de dados, executar as operações de criação, leitura, atualização e deleção de profissionais e retornar os resultados para os controladores.

Segue abaixo o mesmo:

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

Os serviços são definidos utilizando a classe `ProfessionalBase`, que contém os métodos base para as operações CRUD. Cada serviço é responsável por executar uma operação específica, como criar, ler, atualizar ou deletar um profissional, e interagir com o banco de dados para realizar a operação desejada.

Os métodos mais importantes são `create_professional`, `get_professional`, `list_professionals`, `update_professional` e `delete_professional`. Pois são eles que executam as operações CRUD no banco de dados e retornam os resultados para os controladores.

Este arquivo, também acompanha o `base.py` que contém a classe `ProfessionalBase` que é responsável por definir os métodos base para as operações CRUD.

Segue abaixo o mesmo:

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

Nele são definidos os métodos base para as operações CRUD, que são implementados na classe `ProfessionalService`. A classe `ProfessionalBase` é uma classe abstrata que define os métodos que devem ser implementados pelos serviços concretos.

## **3.0** Conclusão

A lógica do CRUD para profissionais no backend do projeto CORA é essencial para a criação, leitura, atualização e deleção de dados no banco de dados. A estrutura de arquivos organizada e os serviços bem definidos facilitam a implementação e manutenção das operações CRUD, garantindo a consistência e integridade dos dados.

O CRUD em nosso projeto, também serve para definidir as permissões de login e prioridades de acesso para os profissionais, garantindo a segurança e confiabilidade do sistema. Com a implementação do CRUD, é possível gerenciar os dados dos profissionais de forma eficiente e segura, otimizando o desempenho e a usabilidade do sistema.