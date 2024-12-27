---
title: Instruções de Inicialização
sidebar_position: 1
---

## **1.0** Introdução 

Este documento é crucial para qualquer usuário que deseje inicializar e operar eficientemente a Interface de Linha de Comando (CLI) e o sistema robótico associado. Contendo instruções detalhadas e meticulosas, ele serve como um guia fundamental para configurar corretamente ambos os componentes tecnológicos. Ao seguir este manual, espera-se que os sistemas operem com plena funcionalidade, otimizando sua performance operacional.

   A aderência rigorosa aos procedimentos descritos é vital. Isso não apenas facilita uma configuração bem-sucedida, mas também previne possíveis falhas operacionais. A execução correta dessas etapas assegura que eventuais problemas sejam minimizados, permitindo que o usuário explore todo o potencial do sistema.

   Portanto, é imprescindível seguir cuidadosamente cada passo apresentado neste documento. Fazer isso garantirá o melhor desempenho possível do equipamento, maximizando a eficiência e eficácia na utilização da CLI e do sistema robótico.

## **2.0** Versões das bibliotecas necessárias

:::danger 
Para evitar problemas de compatibilidade e garantir a integridade do ambiente de execução, é imperativo utilizar as versões das bibliotecas conforme listado abaixo
:::

- ROS2: `humble`
- Python: `3.11`
- Typer: `0.9.0`
- Inquirer: `3.2.4`

## **3.0** Preparação do ambiente de execução

:::warning 
Para garantir uma execução eficiente do software, é essencial preparar adequadamente o ambiente de execução. Isso evitará problemas e erros durante a operação do robô.
:::

### **3.1** Configuração do sistema operacional

O software foi desenvolvido para ser executado especificamente no Ubuntu 22.04. Utilizar uma versão ou sistema operacional diferente pode resultar em falhas de execução.

:::tip
Para checar sua versão atual do Ubuntu, digite o seguinte comando no terminal:
:::

```bash
lsb_release -a
```

E veja se seguido na parte Description está Ubuntu 22.04.04 LTS, se não estiver, será necessário reinstalar o sistema operacional em sua correta versão. Se necessário, segue o link com a versão correta do [Ubuntu](https://releases.ubuntu.com/jammy/)

### **3.2** Verificação e instalação das dependências

Antes de prosseguir, é essencial verificar se as dependências principais estão corretamente instaladas em seu sistema. Utilize os comandos abaixo para confirmar as versões de Python e ROS2:

```bash
python3 --version

printenv ROS_DISTRO
```

#### **3.2.1** Instalando Python

Se o Python não estiver presente em seu sistema, instale-o usando o comando:

```bash
sudo apt install python3
```

:::info 
Sempre que utilizar o comando sudo em seu terminal, será necessário digitar a senha de seu usuário na máquina.
 :::

#### **3.2.2** Instalando ROS2

Para a comunicação entre a interface de linha de comando (CLI) e o robô, será empregado o ROS2, uma escolha tecnológica estratégica que permite a transmissão eficiente de informações através de uma rede robusta. Este sistema é projetado para facilitar a interação entre dispositivos computacionais e máquinas autônomas, proporcionando uma plataforma confiável para o envio de comandos.


:::tip

Para instalar o ROS2, caso ainda não esteja configurado, siga as instruções detalhadas disponíveis neste [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
:::

### **3.3** Estabelecendo a Conexão de Rede

É crucial que seu computador esteja conectado à mesma rede Wi-Fi que o robô para permitir a comunicação.

:::tip 
Para conectar o robo na mesma rede em que você está conectado em sua máquina será necessário executar os seguintes passos:

Conectar a Raspberry Pi do robô em um **monitor**, **teclado** e **mouse**.

Após, entrar no Ubuntu deverá se conectar a rede desejada.
:::

### **3.4** Inicialização da CLI

A inicialização correta da Interface de Linha de Comando (CLI) é fundamental para o controle efetivo do robô e para a execução de comandos específicos do sistema. Para começar, siga estes passos detalhadamente para garantir que a CLI seja iniciada sem problemas:

#### **3.4.1** Configuração para o bash

1. **Clonar o repositório do projeto**: Abra um terminal e digite o seguinte comando para clonar o repositório do projeto para sua máquina local

```bash
git clone https://github.com/Inteli-College/2024-2B-T08-EC08-G02.git
```

2. **Acessar o diretório do projeto**: Navegue até o diretório do projeto utilizando o comando

```bash
cd 2024-2B-T08-EC08-G02
```

3. **Instale o venv na sua máquina**: Instale a extensão de ambiente virtual do python 3.11

```bash
sudo apt install python3.11-venv
```

4. **Execute o script de inicialização**: Execute o seguinte código para o robô criar os tópicos para navegação.

```bash
# Inicialização do Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:={caminho_do_mapa}
```

:::danger
Para ver como adquirir o mapa, cheque a parte **Extra** da documentação [**aqui**](#41-extra)
:::

Após, abra um novo terminal para executar o código da CLI:

```bash
./exec_bash.sh
```

:::tip
Para usuários que usam o Z-Shell, em vez de `./exec_bash.sh`, deve utilizar o seguinte:

```zsh
./exec_zsh.sh
```
:::

Uma vez concluídos esses procedimentos, a Interface de Linha de Comando (CLI) estará totalmente operacional e configurada para facilitar a interação eficaz com o robô.

## **4.0** Utilização da CLI

Afim de utilizar a CLI, é necessário ter entendimento dos seguintes passos, afim de evitar problemas de execução e garantir o correto funcionamento do sistema.

![CLI](/img/cli/cli.png)

A CLI oferece as seguintes opções:

1. **Conectar ao robô:** Estabelece uma conexão SSH com o robô, iniciando o processo de bringup a partir da CLI, para controlar e monitorar remotamente.

2. **Definir posição inicial:** Permite ao usuário definir a posição inicial do robô no mapa, alinhando sua localização no ambiente com a posição atual.

3. **Seguir waypoints:** Faz com que o robô siga os waypoints previamente configurados no código, utilizando o **Nav2** para navegação autônoma em tempo real e evitando obstáculos.

:::info
`Nav2` é o sistema de navegação utilizado para o controle de trajetórias de robôs móveis no `ROS 2`. Ele permite ao robô se mover autonomamente em um ambiente, definindo rotas com segurança e contornando obstáculos. Muito utilizado em aplicações com o TurtleBot3, o `Nav2` facilita a navegação através do mapeamento do ambiente e planejamento de trajetória.
:::

4. **Ir à origem:** Envia o robô de volta à posição inicial definida no mapa.
   
5. **Finalizar bringup:** Encerra a conexão com o robô e finaliza o bringup, interrompendo qualquer processo ativo de controle.

6. **Sair:** Encerra a execução da CLI e, caso esteja rodando, finaliza também o bringup se ele não foi previamente finalizado.

A ordem de execução das opções é crucial para garantir o correto funcionamento do sistema. Portanto, é fundamental seguir a sequência correta para evitar problemas de execução e garantir a eficácia da CLI.

## **4.1** Extra

Afim de utilizar a CLI, é necessário já ter um mapa previamente cartografado utilizando o próprio turtlebot3. Para fazer executar tal mapeamento, siga os passos abaixos:

1. **Conectar ao robô:** Estabelece uma conexão SSH com o robô, iniciando o processo de bringup a partir da CLI, para controlar e monitorar remotamente.

```bash
# Conectar SSH
ssh cora@cora.local
```

```bash
# Iniciar o bringup
ros2 launch turtlebot3_bringup robot.launch.py
```

:::info
Todos os próximos comandos são na sua máquina **local**, **não** no terminal `SSH`. (Supondo que os passos anteriores de configuração da CLI foram seguidos, caso não funcionem, execute os passos anteriores novamente [aqui](#34-inicialização-da-cli))
:::

2. **Iniciar a teleoperação:** Inicie a teleoperação para controlar o robô e mapear o ambiente:

```bash
# Iniciar a teleoperação
ros2 run turtlebot3_teleop teleop_keyboard
```

3. **Mapear o ambiente:** Mapeie o ambiente utilizando o TurtleBot3, movendo-o pelo espaço e coletando informações sobre o ambiente.

```bash
# Iniciar o cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

Segue abaixo uma imagem da ferramente de cartografia:

![Cartographer](/img/cli/cart.png)

4. **Salvar o mapa:** Salve o mapa gerado para utilização futura na navegação autônoma.

```bash
# Salvar o mapa
ros2 run nav2_map_server map_saver_cli -f ~/{caminho_do_mapa}
```

5. **Finalizar o mapeamento:** Encerre o processo de mapeamento e teleoperação.

Digite CTRL + C para encerrar a teleoperação e o cartógrafo (em ambos os terminais).

## **5.0** Conclusão

A execução correta das etapas descritas neste documento é crucial para garantir o funcionamento eficiente da Interface de Linha de Comando (CLI) e do sistema robótico associado. Ao seguir as instruções detalhadas, espera-se que o usuário possa operar o robô com eficácia e precisão, explorando todo o potencial do sistema.

A aderência rigorosa aos procedimentos descritos é vital para minimizar possíveis falhas operacionais e garantir a máxima eficiência do equipamento. Portanto, é essencial seguir cuidadosamente cada passo apresentado neste manual para otimizar o desempenho operacional do sistema.

Ao finalizar a configuração e inicialização da CLI, o usuário estará pronto para interagir com o robô e executar comandos específicos, permitindo a navegação autônoma e o controle remoto do equipamento. Com a execução correta das etapas, o usuário poderá explorar todo o potencial do sistema, maximizando sua eficiência e eficácia operacional.