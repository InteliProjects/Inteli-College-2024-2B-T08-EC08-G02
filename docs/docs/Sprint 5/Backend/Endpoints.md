---
title: Endpoints
sidebar_position: 1
---


# Documentação dos Endpoints

Este documento descreve os principais endpoints da nossa API construída com FastAPI, **MongoDB** para armazenar os dados de robôs e tarefas e **APScheduler** para agendar tarefas automatizadas.
A API contém funcionalidades de autenticação, operações CRUD (Create, Read, Update, Delete) para profissionais, pacientes e respostas, e algumas rotas protegidas por níveis de acesso.

## 1.0. **Autenticação (`auth.py`)**

É por meio desses endpoints que é possível fazer a diferenciação de features de acordo com as credenciais do usuário (médico ou enfermeiro). Também é dessa forma que a segurança é implementada no sistema, permitindo apenas os que possuem um usuário cadastrado acessem a plataforma e por conseguinte as informações dos pacientes. 

### 1.1. `POST /auth/register`

Registra um novo profissional na base de dados.

#### Exemplo de Requisição:
```python
{
    "name": "Dr. John",
    "password": "senha123",
    "role": "doctor"
}
```

#### Exemplo de Resposta:
```json
{
    "access_token": "eyJhbGciOiJIUzI1NiIsInR5c..."
}
```

#### Código:
```python
@router.post("/register", response_model=Token)
async def register_professional(professional: ProfessionalCreate):
    existing_professional = await professional_service.get_by_name(professional.name)
    if existing_professional:
        raise HTTPException(status_code=400, detail="Professional already registered")
    new_professional = await professional_service.create_professional(professional)
    access_token = auth_service.create_access_token(
        data={"sub": new_professional.name, "role": new_professional.role, "id": str(new_professional.id)},
    )
    return Token(access_token=access_token)
```

### 1.2. `POST /auth/login`

Faz login de um profissional e retorna um token JWT.

#### Exemplo de Requisição:
```python
{
    "name": "Dr. John",
    "password": "senha123"
}
```

#### Exemplo de Resposta:
```json
{
    "access_token": "eyJhbGciOiJIUzI1NiIsInR5c..."
}
```

#### Código:
```python
@router.post("/login", response_model=Token)
async def login_professional(professional: ProfessionalLogin):
    db_professional = await professional_service.get_by_name(professional.name)
    if not db_professional or not auth_service.verify_password(professional.password, db_professional.password):
        raise HTTPException(status_code=400, detail="Incorrect username or password")
    access_token = auth_service.create_access_token(
        data={"sub": db_professional.name, "role": db_professional.role, "id": str(db_professional.id)},
    )
    return Token(access_token=access_token)
```

### 1.3. `POST /auth/protected_route`

Uma rota protegida que requer autenticação. Retorna uma mensagem de sucesso e informações do usuário.

#### Exemplo de Resposta:
```json
{
    "message": "This route is protected!",
    "user": {
        "sub": "Dr. John",
        "role": "doctor",
        "id": "123"
    }
}
```

#### Código:
```python
@router.post("/protected_route", response_model=dict)
async def protected_route(current_user: dict = Depends(auth_service.get_current_user)):
    return {"message": "This route is protected!", "user": current_user}
```

## 2.0. **Operações CRUD  (`crud.py`)**

Para todos os agentes da nossa solução(Médicos, pacientes, robôs, perguntas), foram construídos os "CRUDs" (Create, Read, Update e Delete), permitindo modificar as informações guardadas no nosso banco de dados.

Por exemplo, podemos adicionar um profissional:

### 2.1. `POST /crud/`

Cria um novo profissional.

#### Exemplo de Requisição:
```python
{
    "name": "Dr. John",
    "password": "senha123",
    "role": "doctor"
}
```

#### Exemplo de Resposta:
```json
{
    "id": "123",
    "name": "Dr. John",
    "role": "doctor"
}
```

#### Código:
```python
@router.post("/", response_model=ProfessionalSchema)
async def create_professional(professional_data: ProfessionalCreate):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    return await service.create_professional(professional_data)
```

Podemos também verificar os profissionais registrados no sistema:

### 2.2. `GET /crud/{professional_id}`

Obtém um profissional pelo seu ID.

#### Exemplo de Resposta:
```json
{
    "id": "123",
    "name": "Dr. John",
    "role": "doctor"
}
```

#### Código:
```python
@router.get("/{professional_id}", response_model=ProfessionalSchema)
async def get_professional(professional_id: str):
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professional = await service.get_professional(professional_id)
    if not professional:
        raise HTTPException(status_code=404, detail="Professional not found")
    return professional
```

### 2.3. `GET /crud/`

Lista todos os profissionais.

#### Exemplo de Resposta:
```json
{
    "professionals": [
        {"id": "123", "name": "Dr. John", "role": "doctor"},
        {"id": "124", "name": "Dr. Jane", "role": "nurse"}
    ]
}
```

#### Código:
```python
@router.get("/", response_model=ProfessionalResponse)
async def list_professionals():
    service = ProfessionalService(mongo_db.get_collection("professionals"))
    professionals = await service.list_professionals()
    return ProfessionalResponse(professionals=professionals)
```

De maneira semelhante, para as perguntas de checagem, registro e acompanhamento de robôs, médicos, enfermeiros e pacientes, todas essas operações estão cobertas pelas rotas da nossa API em nosso backend.


## 3.0. **Endpoints para `llm.py`**

Aqui serão listados os endponits que estão relacionados ao nosso modelo de LLM que irá interagir com o paciente.

### 3.1. **Adicionar Pergunta**
Adiciona uma nova pergunta ao banco de perguntas.

**Endpoint:**
```python
@router.post("/questions", response_model=QuestionSchema)
async def add_question(question_data: QuestionCreate):
    """
    Adiciona uma nova pergunta ao banco de perguntas.
    """
    question = await watson_service.add_question(question_data)
    return question
```

**Método:** `POST`

**Request Body:**
- `question_data`: Dados da nova pergunta.

**Response:** Retorna os detalhes da pergunta adicionada.

### 3.2. **Listar Perguntas**
Lista todas as perguntas cadastradas no banco de dados.

**Endpoint:**
```python
@router.get("/questions", response_model=QuestionResponse)
async def list_questions():
    """
    Lista todas as perguntas cadastradas.
    """
    questions = await watson_service.get_questions()
    return QuestionResponse(questions=questions)
```

**Método:** `GET`

**Response:** Retorna uma lista de perguntas cadastradas.

### 3.3. **Salvar Resposta**
Salva a resposta de uma pergunta específica.

**Endpoint:**
```python
@router.post("/answers", response_model=AnswerSchema)
async def save_answer(answer_data: AnswerCreate):
    """
    Salva a resposta de uma pergunta específica.
    """
    answer = await watson_service.save_answer(answer_data)
    return answer
```

**Método:** `POST`

**Request Body:** 
- `answer_data`: Dados da resposta a ser salva.

**Response:** Retorna a resposta salva.

### 3.4. **Listar Respostas**
Lista todas as respostas cadastradas.

**Endpoint:**
```python
@router.get("/answers", response_model=AnswerResponse)
async def list_answers():
    """
    Lista todas as respostas cadastradas.
    """
    answers = await watson_service.get_answers()
    return AnswerResponse(answers=answers)
```

**Método:** `GET`

**Response:** Retorna uma lista de respostas cadastradas.

### 3.5. **Transcrever Áudio**
Transcreve um arquivo de áudio para texto usando o serviço Watson Speech to Text.

**Endpoint:**
```python
@router.post("/transcribe", response_model=dict)
async def transcribe_audio(file: UploadFile = File(...)):
    """
    Transcreve um arquivo de áudio para texto usando o Watson Speech to Text.
    """
    try:
        file_path = f"/tmp/{file.filename}"
        with open(file_path, "wb") as buffer:
            buffer.write(await file.read())

        transcript = await watson_service.transcribe_audio(file_path)
        if not transcript:
            raise HTTPException(status_code=500, detail="Erro ao transcrever o áudio")
        return {"transcript": transcript}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

**Método:** `POST`

**Request Body:** Arquivo de áudio no formato `multipart/form-data`.

**Response:** Retorna a transcrição do áudio.


### 3.6. **Interagir com o Watson Assistant**
Envia uma mensagem para o Watson Assistant e retorna a resposta.

**Endpoint:**
```python
@router.post("/assistant", response_model=dict)
async def interact_with_assistant(user_input: str):
    """
    Interage com o Watson Assistant enviando uma mensagem e retornando a resposta.
    """
    try:
        logging.info(f"Passou por /assistant com a mensagem: {user_input}")
        response = await watson_service.send_message_to_assistant(user_input)
        if not response:
            raise HTTPException(
                status_code=500, detail="Erro: Resposta do Watson Assistant está vazia ou inválida."
            )
        return {"response": response}
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Erro inesperado: {str(e)}"
        )
```

**Método:** `POST`

**Request Body:** 
- `user_input`: Mensagem enviada ao Watson Assistant.

**Response:** Retorna a resposta do assistente.


### 3.7. **Executar Anamnese**
Executa a anamnese interativa com perguntas e salva as respostas.

**Endpoint:**
```python
@router.post("/run-anamnese", response_model=dict)
async def run_anamnese():
    """
    Executa a anamnese interativa com perguntas e salva as respostas.
    """
    try:
        await watson_service.run_anamnese()
        answers = await watson_service.get_answers()
        return {"message": "Anamnese concluída", "answers": answers}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

**Método:** `POST`

**Response:** Retorna uma mensagem de sucesso e as respostas da anamnese.


## 4.0. **Endpoints de mapeamento**

Aqui serão listados todos os endpoints que estão gerenciando o mapeamento feito pelo robô.


### 4.1. **Criar Mapeamento de Waypoint para Sala:  `map_crud.py`**

Cria um novo mapeamento entre um waypoint e uma sala.

**Endpoint:**
```python
@router.post("/", response_model=WaypointToRoomSchema)
async def create_mapping(mapping_data: WaypointToRoomCreate):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    return await service.create_mapping(mapping_data)
```

**Método:** `POST`

**Request Body:** 
- `mapping_data`: Dados do mapeamento a ser criado.

**Response:** Retorna os detalhes do mapeamento criado.


### 4.2. **Listar Mapeamentos**
Lista todos os mapeamentos cadastrados.

**Endpoint:**
```python
@router.get("/", response_model=WaypointToRoomResponse)
async def list_mappings():
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    mappings = await service.list_mappings()
    return WaypointToRoomResponse(mappings=mappings)
```

**Método:** `GET`

**Response:** Retorna uma lista de mapeamentos cadastrados.


### 4.3. **Buscar Mapeamento por ID**
Busca um mapeamento específico por ID.

**Endpoint:**
```python
@router.get("/{mapping_id}", response_model=WaypointToRoomSchema)
async def get_mapping(mapping_id: str):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    mapping = await service.get_mapping(mapping_id)
    if not mapping:
        raise HTTPException(status_code=404, detail="Mapping not found")
    return mapping
```

**Método:** `GET`

**Response:** Retorna o mapeamento com o ID especificado.


### 4.4. **Atualizar Mapeamento**
Atualiza os dados de um mapeamento existente.

**Endpoint:**
```python
@router.put("/{mapping_id}", response_model=WaypointToRoomSchema)
async def update_mapping(mapping_id: str, update_data: WaypointToRoomCreate):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    mapping = await service.update_mapping(mapping_id, update_data)
    if not mapping:
        raise HTTPException(status_code=404, detail="Mapping not found")
    return mapping
```

**Método:** `PUT`

**Request Body:** 
- `update_data`: Novos dados para o mapeamento.

**Response:** Retorna os dados atualizados do mapeamento.

### 4.5. **Deletar Mapeamento**
Deleta um mapeamento existente pelo ID.

**Endpoint:**
```python
@router.delete("/{mapping_id}", response_model=dict)
async def delete_mapping(mapping_id: str):
    service = WaypointToRoomService(mongo_db.get_collection("waypoint_to_room"))
    deleted = await service.delete_mapping(mapping_id)
    if not deleted:
        raise HTTPException(status_code=404, detail="Mapping not found")
    return {"deleted": deleted}
```

**Método:** `DELETE`

**Response:** Retorna uma mensagem de sucesso com a confirmação da exclusão.

## 5.0. **Endpoints de Controle do Robô**

### 5.1. `POST /robot/set_initial_pose`

**Descrição**: Define a pose inicial do robô no ambiente, especificando a posição (x, y) e o ângulo de orientação (`yaw`).

**Request Body**:
- `x` (float): Coordenada X da posição inicial.
- `y` (float): Coordenada Y da posição inicial.
- `yaw` (float): Ângulo de orientação.

**Exemplo de Requisição**:

```json
{
  "x": 1.0,
  "y": 2.0,
  "yaw": 0.5
}
```

**Exemplo de Código**:

```python
@router.post("/set_initial_pose")
async def set_initial_pose(request: PoseRequest):
    try:
        result = robot_service.set_initial_pose(request.x, request.y, request.yaw)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error setting initial pose: {str(e)}")
```

### 5.2. `POST /robot/go_to_origin`

**Descrição**: Comanda o robô para retornar à posição de origem no ambiente.

**Exemplo de Código**:

```python
@router.post("/go_to_origin")
async def go_to_origin():
    try:
        result = robot_service.go_to_origin()
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error going to origin: {str(e)}")
```

### 5.3. `POST /robot/go_to_waypoints`

**Descrição**: Envia uma lista de waypoints (coordenadas) para que o robô siga um caminho pré-definido.

**Request Body**:
- `waypoints` (list): Lista de pontos de referência, contendo as coordenadas X, Y, Z e o ângulo de orientação (`yaw`).

**Exemplo de Requisição**:

```json
{
  "waypoints": [
    {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 1.57},
    {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}
  ]
}
```

**Exemplo de Código**:

```python
@router.post("/go_to_waypoints")
async def go_to_waypoints(request: WaypointsRequest):
    try:
        waypoints = []
        for wp in request.waypoints:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = wp.x
            waypoint.pose.position.y = wp.y
            waypoint.pose.position.z = wp.z
            q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, wp.yaw)
            waypoint.pose.orientation.x = q_x
            waypoint.pose.orientation.y = q_y
            waypoint.pose.orientation.z = q_z
            waypoint.pose.orientation.w = q_w
            waypoints.append(waypoint)
        
        result = robot_service.go_to_waypoints(waypoints)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error going to waypoints: {str(e)}")
```


## 6.0. **Tarefas Automatizadas para Robôs (`robot_tasks.py`)**

Este módulo contém a lógica para agendar tarefas automatizadas, como enviar waypoints para os robôs em momentos específicos.

### **6.1. Função `schedule_robot_tasks`**

**Descrição**: Agenda tarefas futuras que ocorrerão nos próximos 30 minutos. As tarefas são recuperadas do banco de dados e os waypoints relacionados ao quarto do paciente são enviados ao robô.

**Exemplo de Código**:

```python
async def schedule_robot_tasks():
    now = datetime.now()
    next_half_hour = now + timedelta(minutes=30)
    
    task_collection = mongo_db.get_collection("tasks")
    patient_collection = mongo_db.get_collection("patients")
    room_collection = mongo_db.get_collection("rooms")

    tasks = await task_collection.find({
        "date_hour": {
            "$gte": now,
            "$lt": next_half_hour
        },
        "status": "pending"
    }).to_list(length=None)

    for task in tasks:
        patient = await patient_collection.find_one({"id": task["id_patient"]})
        if not patient:
            continue
        room = await room_collection.find_one({"id": patient["id_room"]})
        if not room:
            continue

        waypoints = [{
            "x": room["x_pose"],
            "y": room["y_pose"],
            "z": room["z_pose"],
            "yaw": room["w_pose"]
        }]
        
        task_time = task["date_hour"]
        scheduler.add_job(send_waypoints_to_robot, 'date', run_date=task_time, args=[waypoints])
```

## **Conclusão**

A documentação apresentada abrange uma série de módulos, endpoints e serviços essenciais para um sistema de controle de robôs e integração com assistentes baseados em inteligência artificial. Ao combinar funcionalidades de controle direto dos robôs, gerenciamento de dados e automatização de tarefas com a interação com modelos de linguagem de grande porte (LLMs), como o Watson da IBM, o sistema oferece uma infraestrutura robusta e flexível para diversas aplicações.

O **módulo `robot.py`** trata do controle direto dos robôs, permitindo comandos como definição da pose inicial, envio para waypoints ou para a origem. Esses endpoints utilizam serviços internos para processar as ações de controle, oferecendo uma interface eficiente para gerenciar os robôs em tempo real.

O **módulo `robot_crud.py`** fornece uma interface CRUD (Create, Read, Update, Delete) para a gestão dos robôs. Com integração ao banco de dados MongoDB, é possível filtrar robôs por status ou localização, garantindo uma administração eficaz dos recursos operacionais.

No **módulo `robot_tasks.py`**, destaca-se a automação de tarefas, permitindo que os robôs executem ações predefinidas em horários específicos. Isso é feito com base em dados de pacientes e salas armazenados em um banco de dados, otimizando o uso dos robôs em ambientes críticos como hospitais.

Além disso, a documentação inclui endpoints para interação com um **assistente LLM**, como o Watson da IBM, por meio do **endpoint `/llm/assistant`**, permitindo que a API receba perguntas e comandos de texto e retorne respostas geradas pelo assistente. Esse serviço é encapsulado no **WatsonService**, que gerencia a comunicação com o modelo, mantendo uma arquitetura modular e escalável.

No geral, esta documentação detalha uma estrutura modular e eficiente, com grande potencial de expansão e integração. Os módulos apresentados são organizados de forma clara e coesa, facilitando a manutenção, a adição de novas funcionalidades e a escalabilidade do sistema. A combinação de controle robótico e IA oferece versatilidade para aplicações avançadas em diversas áreas, como automação, assistência hospitalar e suporte ao cliente.
