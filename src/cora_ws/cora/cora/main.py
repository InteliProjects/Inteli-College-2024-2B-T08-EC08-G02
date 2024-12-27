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
    def createPoseStamped(navigator, pos_x, pos_y, rot_z):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pos_x
        pose.pose.position.y = pos_y
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose


class RobotNavigator:
    def __init__(self):
        self.client = None
        self.bringup_thread = None
        self.bringup_active = None

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

    def goToOrigin(self):
        rclpy.init()
        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        origin_pose = PoseHelper.createPoseStamped(nav, 0.0, 0.0, 0.0)
        nav.goToPose(origin_pose)
        while not nav.isTaskComplete():
            print(nav.getFeedback())
        rclpy.shutdown()

    def followWp(self):
        rclpy.init()
        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        waypoints = [
            PoseHelper.createPoseStamped(nav, 0.807, 0.128, 0.00247),
            PoseHelper.createPoseStamped(nav, 1.37, -0.435, 0.00247),
            PoseHelper.createPoseStamped(nav, 0.0438, -1.02, 0.00247),
        ]
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            print(nav.getFeedback())
        rclpy.shutdown()

    def setInitialPose(self):
        rclpy.init()
        nav = BasicNavigator()
        initial_pose = PoseHelper.createPoseStamped(nav, 0.0, 0.0, 0.0)
        nav.setInitialPose(initial_pose)
        nav.waitUntilNav2Active()
        rclpy.shutdown()


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


if __name__ == "__main__":
    app()
