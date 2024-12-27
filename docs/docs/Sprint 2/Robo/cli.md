---
title: Interface de Linha de Comando (CLI)
sidebar_position: 3
---

## **1.0** Introdução

Este documento serve para explicar como funciona o código e como foi feita a implementação da Interface de Linha de Comando (CLI) do projeto CORA. A CLI é uma ferramenta essencial para interagir com o sistema robótico, permitindo que o usuário envie comandos e receba informações do robô de forma simples e intuitiva.

Serão apresentados os principais conceitos e técnicas de programação utilizados na implementação da CLI, bem como os comandos disponíveis e suas funcionalidades. Serão abordados os seguintes tópicos:

- **Estrutura do código**: como o código da CLI está organizado e quais são os principais arquivos e pastas.

- **Comandos disponíveis**: quais são os comandos disponíveis na CLI e como utilizá-los para interagir com o robô.

- **Funcionalidades**: quais são as principais funcionalidades da CLI e como elas foram implementadas para facilitar a interação com o sistema robótico.

## **2.0** Estrutura do código

A CLI foi implementada dentro de um Workspace do ROS2, o mesmo é composto por vários pacotes que se comunicam entre si para realizar as operações desejadas. A estrutura do código da CLI é organizada da seguinte forma:

- **Pacote da CLI (CORA)**: contém os arquivos e pastas necessários para a implementação da CLI, incluindo os comandos disponíveis e as funcionalidades implementadas.

:::info
Criamos tudo dentro do `workspace` do **ROS2**, para que a comunicação entre a interface de linha de comando e o robô seja eficiente e segura. Para saber mais como funcionam estes `workspaces` clique [**aqui**](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
:::

Todo o código referente a CLI pode ser encontrado no diretório `src/cora_ws/cora/cora/` do workspace do ROS2. Dentro deste diretório, estão os seguintes arquivos e pastas:

- **`__init__.py`**: Arquivo de inicialização do pacote, que define as funções e classes disponíveis para importação.

- **`main.py`**: Arquivo principal da CLI, que contém a lógica principal da interface de linha de comando e os comandos disponíveis.

### **2.1** Bibliotecas utilizadas

A implementação da CLI do projeto CORA utiliza várias bibliotecas do Python para facilitar a interação com o robô e executar operações específicas. Algumas das bibliotecas mais utilizadas são:

- **`typer`**: Biblioteca para criar interfaces de linha de comando de forma simples e intuitiva.

- **`rclpy`**: Biblioteca para interagir com o ROS 2 em Python, permitindo a comunicação com o robô.

- **`paramiko`**: Biblioteca para estabelecer conexões SSH com o robô e iniciar o processo de bringup.

- **`inquirer`**: Biblioteca para exibir menus de opções na interface de linha de comando e permitir ao usuário escolher uma operação.

- **`threading`**: Biblioteca para criar threads e executar operações em paralelo na interface de linha de comando.

- **`os`**: Biblioteca para interagir com o sistema operacional e executar comandos no terminal.

Segue abaixo o import de todas as bibliotecas utilizadas:

```python
import typer
import threading
import inquirer
import os
from dotenv import load_dotenv
import paramiko
from typing import Optional
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi
```



## **3.0** Comandos disponíveis

A CLI do projeto CORA possui vários comandos disponíveis para interagir com o robô e executar operações específicas. Alguns dos comandos mais utilizados são:

- **`Ir a origem`**: Comando que permite ao robô se deslocar até a origem do ambiente.

- **`Seguir Waypoints`**: Comando que permite ao robô se deslocar entre os waypoints pré definidos anteriormente.

- **`Definir Posição Inicial`**: Comando que permite ao robô definir a posição inicial para a navegação autônoma.

- **`Conectar Robô`**: Comando que inicializa o bringup dentro do robô para que ele possa se comunicar com a interface de linha de comando.

- **`Finalizar Bringup`**: Comando que finaliza o bringup do robô, encerrando a comunicação com a interface de linha de comando.

- **`Sair`**: Comando que encerra a execução da CLI e finaliza a interação com o robô.


## **4.0** Código da CLI

A seguir, será apresentado alguns trechos do código da CLI para exemplificar como os comandos são implementados e como a interação com o robô é realizada.

### **4.1** Comando `Ir a origem`

```python
    def goToOrigin(self):
        rclpy.init()
        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        origin_pose = PoseHelper.createPoseStamped(nav, 0.0, 0.0, 0.0)
        nav.goToPose(origin_pose)
        while not nav.isTaskComplete():
            print(nav.getFeedback())
        rclpy.shutdown()
```

Este comando é responsável por enviar o robô de volta ao ponto de origem. Ele inicializa o nó de navegação, cria uma mensagem de pose com a posição inicial do robô e envia essa mensagem ao controlador de navegação. O robô seguirá a trajetória de volta ao ponto de origem.

### **4.2** Comando `Seguir Waypoints`

```python
    def followWp(self):
        rclpy.init()
        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        waypoints = [
            # Abaixo há os valores já pré-definidos. Para saber como adquirí-los, consulte a parte da documentação relacionada as instruções.
            PoseHelper.createPoseStamped(nav, 0.807, 0.128, 0.00247),
            PoseHelper.createPoseStamped(nav, 1.37, -0.435, 0.00247),
            PoseHelper.createPoseStamped(nav, 0.0438, -1.02, 0.00247),
        ]
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            print(nav.getFeedback())
        rclpy.shutdown()
```

Este comando é responsável por enviar o robô para uma sequência de waypoints declarados no código. Ele inicializa o nó de navegação, cria uma lista de mensagens de pose com os waypoints, e envia essa lista ao controlador de navegação. O robô seguirá a sequência de waypoints até chegar ao destino final.

### **4.3** Comando `Definir Posição Inicial`

```python
    def setInitialPose(self):
        rclpy.init()
        nav = BasicNavigator()
        initial_pose = PoseHelper.createPoseStamped(nav, 0.0, 0.0, 0.0)
        nav.setInitialPose(initial_pose)
        nav.waitUntilNav2Active()
        rclpy.shutdown()
```

Este comando é responsável por declarar a posição inicial do robô no ambiente. Ele inicializa o nó de navegação, cria uma mensagem de pose com a posição inicial do robô e envia essa mensagem ao controlador de navegação.

### **4.4** Comando `Conectar Robô`

```python
def setupSSH(self):
        host = os.getenv("SSH_HOST")
        user = os.getenv("SSH_USER")
        password = os.getenv("SSH_PASSWORD")

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            client.connect(
                host,
                username=user,
                password=password,
                banner_timeout=30,
            )

            stdin, stdout, stderr = client.exec_command(
                "source /opt/ros/humble/setup.bash && "
                "export TURTLEBOT3_MODEL=burger && "
                "source ~/turtlebot3_ws/install/setup.bash && "
                "export LDS_MODEL=LDS-02 && "
                "export OPENCR_PORT=/dev/ttyACM0 && "
                "export OPENCR_MODEL=burger && "
                "export ROS_DOMAIN_ID=117 && "
                "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                "ros2 launch turtlebot3_bringup robot.launch.py",
                get_pty=True,
            )

            self.bringup_active = True
            self.stdin = stdin
            self.stdout = stdout
            self.stderr = stderr

            self.bringup_thread = threading.Thread(target=self._monitorBringupOutput)
            self.bringup_thread.start()
            typer.echo(
                typer.style(
                    "Bringup initiated successfully.", fg=typer.colors.GREEN, bold=True
                )
            )

        except Exception as e:
            typer.echo(
                typer.style(
                    "Error in bringup connection to the robot.",
                    fg=typer.colors.RED,
                    bold=True,
                )
            )

def _monitorBringupOutput(self):
        try:
            while self.bringup_active and not self.stdout.channel.exit_status_ready():
                if self.stdout.channel.recv_ready():
                    typer.echo(self.stdout.channel.recv(1024).decode("utf-8"), nl=False)
        except Exception as e:
            typer.echo(
                typer.style(
                    "Error in monitoring bringup output.",
                    fg=typer.colors.RED,
                    bold=True,
                )
            )
```

Este comando é responsável por estabelecer uma conexão SSH com o robô e iniciar o processo de bringup para que o robô possa se comunicar com a interface de linha de comando. Ele utiliza a biblioteca `paramiko` para estabelecer a conexão SSH e executa os comandos necessários para iniciar o bringup do robô.

### **4.5** Comando `Finalizar Bringup`

```python
def endBringup(self):
        if self.bringup_active:
            self.stdin.write("\x03")
            self.stdin.flush()
            self.bringup_active = False
            self.bringup_thread.join()
            typer.echo(
                typer.style(
                    "Bringup ended successfully.", fg=typer.colors.YELLOW, bold=True
                )
            )
```

### **4.6** Função principal

```python

@app.command()
def main():
    typer.echo(typer.style(banner, fg=typer.colors.CYAN, bold=True))
    typer.echo(typer.style("Navegação robô via CLI!", fg=typer.colors.GREEN, bold=True))
    manager = RobotNavigator()

    try:
        while True:
            options = [
                inquirer.List(
                    "operation",
                    message="What operation would you like to perform?",
                    choices=[
                        "Ir a origem",
                        "Seguir Waypoints",
                        "Definir Posição Inicial",
                        "Conectar Robo",
                        "Finalizar Bringup",
                        "Sair",
                    ],
                )
            ]

            choice: Optional[dict] = inquirer.prompt(options)
            if not choice:
                continue

            match choice["operation"]:
                case "Ir a origem":
                    manager.goToOrigin()
                case "Seguir Waypoints":
                    manager.followWp()
                case "Definir Posição Inicial":
                    manager.setInitialPose()
                case "Conectar Robo":
                    manager.setupSSH()
                case "Finalizar Bringup":
                    manager.endBringup()
                case "Sair":
                    break

    finally:
        manager.shutdown()
```

Este é o arquivo principal da CLI, que contém a função principal da interface de linha de comando. Ele exibe um menu com as operações disponíveis e permite ao usuário escolher qual operação deseja realizar. O código utiliza a biblioteca `typer` para criar a interface de linha de comando e a biblioteca `inquirer` para exibir o menu de opções.

## **5.0** Conclusão

A implementação da CLI do projeto CORA é uma ferramentjsona essencial para interagir com o sistema robótico de forma simples e intuitiva. A CLI permite ao usuário enviar comandos e receber informações do robô de forma eficiente, facilitando a operação e o controle do robô.

Ao entender os conceitos e técnicas de programação utilizados na implementação da CLI, é possível otimizar a interação com o robô e explorar todo o potencial da navegação autônoma. Com uma CLI bem implementada, o usuário pode controlar o robô de forma remota e executar tarefas complexas de forma simples e eficiente.

A aderência rigorosa aos procedimentos descritos neste documento é vital para garantir o funcionamento eficiente da CLI e do sistema robótico associado. Ao seguir as instruções detalhadas, espera-se que o usuário possa operar o robô com eficácia e precisão, explorando todo o potencial do sistema.

:::info

Para saber **como** executar a **cli**, cheque as instruções de inicialização [**aqui**](/Sprint%202/instructions).
:::