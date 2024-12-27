---
title: Navegação
sidebar_position: 2
---

## **1.0** Introdução

A navegação autônoma é uma das funcionalidades mais importantes para um robô móvel. Ela permite que o robô se desloque de um ponto a outro de forma autônoma, evitando obstáculos e respeitando as restrições do ambiente. Neste documento, serão apresentados os principais conceitos e técnicas de navegação autônoma, bem como a implementação de um sistema de navegação baseado no ROS 2.

No contexto deste projeto, e do robô Turtlebot3, utilizamos o pacote de navegação do ROS 2 (Navigation 2), que é uma implementação do algoritmo de navegação baseado em Dijkstra e A* para planejamento de trajetórias. O pacote de navegação é composto por vários nós que se comunicam entre si para realizar o planejamento da trajetória, controle do robô e detecção de obstáculos.

## **2.0** Como funciona a navegação autônoma

A navegação autônoma é um processo complexo que envolve várias etapas, desde a captura de dados do ambiente até a execução do plano de trajetória. O robô precisa ser capaz de perceber o ambiente ao seu redor, planejar uma trajetória segura até o destino e executar o plano de forma eficiente.

O processo de navegação autônoma pode ser dividido em três etapas principais:

### **2.1** Percepção do ambiente

O robô utiliza sensores, como o lidar para capturar informações sobre o ambiente ao seu redor. Essas informações são utilizadas para construir um mapa do ambiente e identificar obstáculos.

### **2.2** Planejamento da trajetória

Com base no mapa do ambiente e na posição do robô, é feito o planejamento da trajetória até o destino. algoritmo de planejamento de trajetória calcula a rota mais curta e segura, levando em consideração os obstáculos e as restrições do ambiente.

### **2.3** Controle do robô

Uma vez que a trajetória foi planejada, o robô precisa executar o plano de forma eficiente. O controle do robô é responsável por enviar comandos para os motores e garantir que o robô siga a trajetória planejada.

## **3.0** Implementação da navegação autônoma no ROS 2

A implementação da navegação autônoma no ROS 2 envolve a configuração de vários componentes, como o pacote de navegação, o mapa do ambiente e os parâmetros de navegação. Neste projeto, utilizamos o pacote de navegação do ROS 2 (Navigation 2) para realizar a navegação autônoma do robô Turtlebot3.

Para que está implementação funcionasse, segue abaixo alguns trechos de código que foram utilizados:

### **3.1** Classe PoseHelper

```python
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
```

A classe acima é responsável por criar uma mensagem de pose (posição e orientação) para ser enviada ao controlador de navegação. Ela recebe como parâmetros a posição x, posição y e rotação z do robô, e retorna uma mensagem PoseStamped.

### **3.2** Classe de Navegação

Segue abaixo os três métodos principais utilizados para que o robô sejá capaz de ir aos respectivos locais:

#### **3.2.1** Método para declarar a posição inicial do robô

```python
def setInitialPose(self):
        rclpy.init()
        nav = BasicNavigator()
        initial_pose = PoseHelper.createPoseStamped(nav, 0.0, 0.0, 0.0)
        nav.setInitialPose(initial_pose)
        nav.waitUntilNav2Active()
        rclpy.shutdown()
```

Este método é responsável por declarar a posição inicial do robô no ambiente. Ele inicializa o nó de navegação, cria uma mensagem de pose com a posição inicial do robô e envia essa mensagem ao controlador de navegação.

#### **3.2.2** Método para enviar o robô para os waypoints declarados no código

```python
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
```

Este método é responsável por enviar o robô para uma sequência de waypoints declarados no código. Ele inicializa o nó de navegação, cria uma lista de mensagens de pose com os waypoints, e envia essa lista ao controlador de navegação. O robô seguirá a sequência de waypoints até chegar ao destino final.

#### **3.2.3** Método para enviar o robô de volta ao ponto de origem

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

Este método é responsável por enviar o robô de volta ao ponto de origem. Ele inicializa o nó de navegação, cria uma mensagem de pose com a posição inicial do robô e envia essa mensagem ao controlador de navegação. O robô seguirá a trajetória de volta ao ponto de origem.

:::tip
Cheque a documentação relacionada a cartografia do mapa, para saber como adquirir o mesmo. Lembrando, é **necessário** ter um mapa do ambiente já criado antes de utilizar a navegação para saber como fazer o mesmo, cheque a parte da documentação referente clicando [**aqui**](/Sprint%202/Robo/cartography).
:::

## **4.0** Conclusão

A navegação autônoma é uma funcionalidade essencial para robôs móveis, permitindo que eles se desloquem de forma autônoma em ambientes desconhecidos. Neste documento, foram apresentados os principais conceitos e técnicas de navegação autônoma, bem como a implementação de um sistema de navegação baseado no ROS 2.

A implementação da navegação autônoma no ROS 2 envolve a configuração de vários componentes, como o pacote de navegação, o mapa do ambiente e os parâmetros de navegação. Com a utilização do pacote de navegação do ROS 2 (Navigation 2), é possível realizar a navegação autônoma do robô Turtlebot3 de forma eficiente e segura.

A navegação autônoma é uma área de pesquisa em constante evolução, com novas técnicas e algoritmos sendo desenvolvidos para melhorar o desempenho e a eficiência dos robôs móveis. Com a implementação da navegação autônoma no ROS 2, é possível explorar todo o potencial do robô Turtlebot3 e realizar tarefas complexas de forma autônoma.