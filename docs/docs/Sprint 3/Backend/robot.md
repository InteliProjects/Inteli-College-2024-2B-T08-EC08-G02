---
title: Estrutura do serviço ao robô no backend
sidebar_position: 3
---

## **1.0** Introdução

Este documento tem como objetivo apresentar a estrutura do serviço ao robô no backend, detalhando como ele foi projetado para atender às necessidades do sistema. A arquitetura abordada visa garantir a eficiência na comunicação com o robô, além de oferecer uma base sólida para o desenvolvimento e manutenção de novas funcionalidades.

Além disso, o documento explora a organização do serviço, suas principais funções e a forma como ele interage com outros componentes do sistema. A intenção é proporcionar uma visão clara e abrangente para desenvolvedores e profissionais interessados em entender a lógica por trás da implementação e seus benefícios.

## **2.0** Estrutura de arquivos

A estrutura de arquivos do serviço ao robô no backend é organizada de forma a facilitar a manutenção e o desenvolvimento de novas funcionalidades. Os principais diretórios e arquivos são:

:::warning
Para ter uma visão geral de todas as pastas e arquivos do projeto, acesse a [documentação da estrutura do backend](/Sprint%203/Backend/estrutura).
:::

### **2.1** `/api/endpoints/robot.py`

Este arquivo contém as definições das rotas relacionadas ao serviço ao robô. Ele é responsável por receber as requisições do frontend, processá-las e enviar as respostas adequadas. Além disso, ele interage com o módulo serviço do robô para executar as operações necessárias.

Segue abaixo o mesmo:

```python
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from app.services.robot.robot import RobotService

router = APIRouter(prefix="/robot", tags=["Robot Control"])

class PoseRequest(BaseModel):
    x: float
    y: float
    z: float
    yaw: float

class WaypointsRequest(BaseModel):
    waypoints: List[PoseRequest]

robot_service = RobotService()

@router.post("/set_initial_pose")
async def set_initial_pose(request: PoseRequest):
    try:
        result = robot_service.set_initial_pose(request.x, request.y, request.yaw)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error setting initial pose: {str(e)}")

@router.post("/go_to_origin")
async def go_to_origin():
    try:
        result = robot_service.go_to_origin()
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error going to origin: {str(e)}")

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

No mesmo, são definidas as rotas `/set_initial_pose`, `/go_to_origin` e `/go_to_waypoints`, que são responsáveis por receber as requisições do frontend e chamar os métodos correspondentes do módulo serviço do robô.

Além disso, são definidos os modelos `PoseRequest` e `WaypointsRequest`, que representam os dados enviados pelo frontend nas requisições. Eles são utilizados para validar os dados recebidos e garantir que estão corretos antes de serem processados.

:::info
Na próxima sprint os modelos `PoseRequest` e `WaypointsRequest` serão realocados para os Schemas, afim de manter a organização do código.
:::

### **2.2** `services/robot/robot.py`

Este arquivo contém a implementação do módulo serviço do robô. Ele é responsável por realizar as operações necessárias para controlar o robô, como definir a posição inicial, ir para a origem e seguir waypoints.

Segue abaixo o mesmo:

```python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from typing import List

class RobotService:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()

    def set_initial_pose(self, x: float, y: float, yaw: float) -> dict:
        """Set the initial pose for the robot"""
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        self.navigator.setInitialPose(pose)
        self.navigator.waitUntilNav2Active()

        return {"status": "success", "pose": self.pose_to_dict(pose)}

    def go_to_origin(self) -> dict:
        """Move the robot to the origin (0, 0)"""
        origin_pose = PoseStamped()
        origin_pose.header.frame_id = 'map'
        origin_pose.pose.position.x = 0.0
        origin_pose.pose.position.y = 0.0
        origin_pose.pose.position.z = 0.0
        origin_pose.pose.orientation.x = 0.0
        origin_pose.pose.orientation.y = 0.0
        origin_pose.pose.orientation.z = 0.0
        origin_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(origin_pose)
        while not self.navigator.isTaskComplete():
            pass

        return {"status": "success", "pose": self.pose_to_dict(origin_pose)}

    def go_to_waypoints(self, waypoints: List[PoseStamped]) -> dict:
        """Move the robot through a list of waypoints"""
        self.navigator.followWaypoints(waypoints)
        while not self.navigator.isTaskComplete():
            pass

        waypoints_dict = [self.pose_to_dict(wp) for wp in waypoints]
        return {"status": "success", "waypoints": waypoints_dict}

    def pose_to_dict(self, pose: PoseStamped) -> dict:
        """Convert PoseStamped message to dictionary for easier response formatting"""
        return {
            "header": {
                "frame_id": pose.header.frame_id,
                "stamp": {
                    "sec": pose.header.stamp.sec,
                    "nanosec": pose.header.stamp.nanosec
                }
            },
            "pose": {
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z,
                },
                "orientation": {
                    "x": pose.pose.orientation.x,
                    "y": pose.pose.orientation.y,
                    "z": pose.pose.orientation.z,
                    "w": pose.pose.orientation.w
                }
            }
        }

    def shutdown(self):
        """Shutdown the ROS 2 system"""
        rclpy.shutdown()
```

No mesmo, são definidos os métodos `set_initial_pose`, `go_to_origin` e `go_to_waypoints`, que são responsáveis por controlar o robô de acordo com as requisições recebidas. Eles utilizam a biblioteca `nav2_simple_commander` para enviar comandos de navegação ao robô e aguardar a conclusão das tarefas.

Cada método tem uma implementação específica para realizar a operação desejada, como definir a posição inicial, ir para a origem ou seguir uma lista de waypoints. Além disso, eles retornam um dicionário com o status da operação e os dados relevantes para o frontend.

## **3.0** Robot Tasks

Foi implementado um sistema que permite o robô definir uma `schedule` de tarefas a serem executadas. O sistema é composto por um conjunto de tarefas, onde cada tarefa é composta por um conjunto de ações. As ações são executadas sequencialmente, de acordo com a ordem definida na tarefa.

Segue abaixo o mesmo:

```python
from datetime import datetime, timedelta
from apscheduler.schedulers.asyncio import AsyncIOScheduler
from robot_tasks import send_waypoints_to_robot  # Função para enviar os waypoints
from app.db import mongo_db  # Conexão com o MongoDB
from motor.motor_asyncio import AsyncIOMotorClient
import asyncio

# Inicializando o agendador
scheduler = AsyncIOScheduler()

# Função para buscar as próximas tarefas do robô e enviá-las no horário correto
async def schedule_robot_tasks():
    # Obter o horário atual e o horário daqui a 30 minutos
    now = datetime.now()
    next_half_hour = now + timedelta(minutes=30)
    
    # Conectar às coleções de tarefas
    task_collection = mongo_db.get_collection("tasks")
    patient_collection = mongo_db.get_collection("patients")
    room_collection = mongo_db.get_collection("rooms")

    # Buscar as tarefas que estão agendadas para os próximos 30 minutos
    tasks = await task_collection.find({
        "date_hour": {
            "$gte": now,
            "$lt": next_half_hour
        },
        "status": "pending"
    }).to_list(length=None)

    for task in tasks:
        # Obter o paciente relacionado à tarefa
        patient = await patient_collection.find_one({"id": task["id_patient"]})
        if not patient:
            print(f"Paciente não encontrado para a tarefa {task['_id']}")
            continue

        # Obter o quarto relacionado ao paciente
        room = await room_collection.find_one({"id": patient["id_room"]})
        if not room:
            print(f"Quarto não encontrado para o paciente {patient['id']}")
            continue

        # Preparar os waypoints (coordenadas) com base no quarto do paciente
        waypoints = [{
            "x": room["x_pose"],
            "y": room["y_pose"],
            "z": room["z_pose"],
            "yaw": room["w_pose"]
        }]

        # Agendar o envio dos waypoints para o horário da tarefa
        task_time = task["date_hour"]
        delay = (task_time - now).total_seconds()

        scheduler.add_job(send_waypoints_to_robot, 'date', run_date=task_time, args=[waypoints])
        print(f"Tarefa {task['_id']} agendada para {task_time}")

# Função para iniciar o agendador
async def start_scheduler():
    scheduler.start()

    # Verifica as tarefas a cada 30 minutos
    scheduler.add_job(schedule_robot_tasks, 'interval', minutes=30)

# Função principal para iniciar o loop de eventos
async def main():
    await start_scheduler()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
```

No mesmo, é definida a função `schedule_robot_tasks`, que busca as tarefas agendadas para os próximos 30 minutos e prepara os waypoints com base nas informações do paciente e do quarto. Em seguida, ela agenda o envio dos waypoints para o horário da tarefa, utilizando o agendador `apscheduler`.

Além disso, é definida a função `start_scheduler`, que inicia o agendador e adiciona uma tarefa para verificar as tarefas a cada 30 minutos. Por fim, a função `main` é responsável por iniciar o loop de eventos e executar o agendador.

:::info
Na próxima sprint, este arquivo sera remanejado em outros afim de manter a organização
:::

## **4.0** Conclusão

A estrutura do serviço ao robô no backend foi projetada para oferecer uma base sólida e eficiente para o controle do robô. A organização dos arquivos e a implementação das funcionalidades garantem a integração adequada com o frontend e a comunicação eficaz com o robô.

Além disso, o sistema de tarefas permite agendar o envio de waypoints para o robô de acordo com as necessidades do sistema. Isso proporciona uma maior flexibilidade e automação no controle do robô, facilitando a execução de tarefas complexas e a otimização do tempo de operação.

Em resumo, a estrutura do serviço ao robô no backend foi desenvolvida com foco na eficiência, escalabilidade e manutenção do sistema. Ela oferece uma base sólida para o desenvolvimento de novas funcionalidades e aprimoramentos futuros, garantindo a qualidade e a confiabilidade do sistema como um todo.