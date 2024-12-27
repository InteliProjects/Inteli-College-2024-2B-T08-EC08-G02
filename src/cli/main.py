import typer
import threading
import os
from dotenv import load_dotenv
import paramiko
import inquirer
from typing import Optional

app = typer.Typer()

load_dotenv()

banner = """
 ________  ________  ________  ________          ________  ___       ___     
|\   ____\|\   __  \|\   __  \|\   __  \        |\   ____\|\  \     |\  \    
\ \  \___|\ \  \|\  \ \  \|\  \ \  \|\  \       \ \  \___|\ \  \    \ \  \   
 \ \  \    \ \  \\\  \ \   _  _\ \   __  \       \ \  \    \ \  \    \ \  \  
  \ \  \____\ \  \\\  \ \  \\  \\ \  \ \  \       \ \  \____\ \  \____\ \  \ 
   \ \_______\ \_______\ \__\\ _\\ \__\ \__\       \ \_______\ \_______\ \__\
    \|_______|\|_______|\|__|\|__|\|__|\|__|        \|_______|\|_______|\|__|
"""


class PoseHelper:
    @staticmethod
    def create_pose_stamped(pos_x, pos_y, rot_z):
        pose = {
            "position": {"x": pos_x, "y": pos_y, "z": rot_z},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": rot_z},
        }
        typer.echo(
            typer.style(
                f"Created PoseStamped: {pose}", fg=typer.colors.GREEN, bold=True
            )
        )
        return pose


class RobotNavigator:
    def __init__(self):
        self.client = None
        self.bringup_thread = None
        self.bringup_active = None

    def setup_ssh(self):
        host = os.getenv("SSH_HOST")
        user = os.getenv("SSH_USER")
        password = os.getenv("SSH_PASSWORD")

        client = paramiko.client.SSHClient()
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

            self.bringup_thread = threading.Thread(target=self._monitor_bringup_output)
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

    def _monitor_bringup_output(self):
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

    def end_bringup(self):
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

    def shutdown(self):
        if self.bringup_active:
            self.end_bringup()
        if self.client:
            self.client.close()
        typer.echo(
            typer.style(
                "Navigator shutdown complete.", fg=typer.colors.GREEN, bold=True
            )
        )

    def go_to_origin(self):
        origin_pose = PoseHelper.create_pose_stamped(0.0, 0.0, 0.0)
        typer.echo(
            typer.style(
                f"Going to origin with pose: {origin_pose}",
                fg=typer.colors.YELLOW,
                bold=True,
            )
        )
        self.wait_until_task_completed()

    def go_to_waypoints(self):
        waypoints = [
            PoseHelper.create_pose_stamped(0.807, 0.128, 0.00247),
            PoseHelper.create_pose_stamped(1.37, -0.435, 0.00247),
            PoseHelper.create_pose_stamped(0.0438, -1.02, 0.00247),
        ]
        typer.echo(
            typer.style("Following waypoints:", fg=typer.colors.YELLOW, bold=True)
        )
        for wp in waypoints:
            typer.echo(typer.style(f" - {wp}", fg=typer.colors.YELLOW, bold=True))
        self.wait_until_task_completed()

    def set_initial_pose(self):
        initial_pose = PoseHelper.create_pose_stamped(0.0, 0.0, 0.0)
        typer.echo(
            typer.style(
                f"Setting initial pose: {initial_pose}",
                fg=typer.colors.YELLOW,
                bold=True,
            )
        )
        typer.echo(
            typer.style(
                "Waiting for navigation to activate...",
                fg=typer.colors.YELLOW,
                bold=True,
            )
        )

    def wait_until_task_completed(self):
        typer.echo(
            typer.style(
                "Waiting for task to complete...", fg=typer.colors.YELLOW, bold=True
            )
        )
        typer.echo(typer.style("Task completed.", fg=typer.colors.GREEN, bold=True))


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
                    manager.go_to_origin()
                case "Seguir Waypoints":
                    manager.go_to_waypoints()
                case "Definir Posição Inicial":
                    manager.set_initial_pose()
                case "Conectar Robo":
                    manager.setup_ssh()
                case "Finalizar Bringup":
                    manager.end_bringup()
                case "Sair":
                    break

    finally:
        manager.shutdown()


if __name__ == "__main__":
    main()
