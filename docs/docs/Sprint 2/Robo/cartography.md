---
title: Cartografia do ambiente
sidebar_position: 1
---

## **1.0** Introdução

Este documento é crucial para entender como funciona a cartografia do ambiente utilizando o sistema robótico. A cartografia é um processo essencial para a navegação autônoma de robôs móveis, pois permite que o robô construa um mapa do ambiente em que está operando. Com um mapa preciso, o robô pode planejar rotas eficientes, evitar obstáculos e executar tarefas de forma autônoma.

Neste documento, serão apresentados os principais conceitos e técnicas de cartografia do ambiente, bem como a implementação deste sistema no nosso projeto, CORA baseado no ROS 2. Serão abordados os seguintes tópicos:

- **Mapeamento do ambiente**: como o robô constrói um mapa do ambiente utilizando sensores e algoritmos de mapeamento.

- **Implementação do mapeamento**: como implementamos o sistema de mapeamento no projeto CORA, incluindo os pacotes e nós utilizados.

- **Visualização do mapa**: como visualizar o mapa do ambiente gerado pelo robô e como interpretar as informações contidas no mapa.

## **2.0** Mapeamento do ambiente

O mapeamento do ambiente é um processo fundamental para a navegação autônoma de robôs móveis. Ele permite que o robô construa um mapa do ambiente em que está operando, identificando obstáculos, paredes, portas e outros elementos do ambiente. Com um mapa preciso, o robô pode planejar rotas eficientes, evitar obstáculos e executar tarefas de forma autônoma.

Existem várias técnicas e algoritmos de mapeamento que podem ser utilizados para construir um mapa do ambiente. Alguns dos mais comuns incluem:

- **SLAM (Simultaneous Localization and Mapping)**: um método que permite que um robô construa um mapa do ambiente enquanto se localiza dentro dele. O SLAM é frequentemente utilizado em ambientes desconhecidos ou em constante mudança.

- **Grid mapping**: um método que divide o ambiente em uma grade de células e atribui a cada célula um valor que representa a probabilidade de ocupação. Este método é comumente utilizado em ambientes estáticos e bem estruturados.

- **Feature-based mapping**: um método que identifica características distintas do ambiente, como cantos, bordas e linhas, e utiliza essas características para construir um mapa. Este método é útil em ambientes com características distintas e facilmente identificáveis.

:::info
No caso do nosso projeto, utilzamos o próprio pacote do **Turtlebot3** para `cartografia`, ja previamente feito.
:::

## **3.0** Implementação do mapeamento

A implementação do sistema de mapeamento no projeto CORA é baseada no pacote de cartografia do Turtlebot3, que utiliza o algoritmo **SLAM** para construir um mapa do ambiente. O pacote de cartografia é composto por vários nós que se comunicam entre si para realizar a cartografia do ambiente, identificar obstáculos e construir um mapa preciso.

Para que o sistema de mapeamento funcione corretamente, é necessário configurar corretamente os parâmetros do pacote de cartografia e iniciar os nós necessários para a cartografia do ambiente. Além disso, é importante calibrar os sensores do robô e garantir que o ambiente esteja adequado para a cartografia.

## **4.0** Visualização do mapa

Após a cartografia do ambiente ser concluída, é possível visualizar o mapa gerado pelo robô e interpretar as informações contidas no mapa. O mapa pode ser visualizado em ferramentas de visualização 2D e 3D, como o **RViz**, que permite visualizar o mapa em tempo real e interagir com ele.

No mapa, é possível identificar obstáculos, paredes, portas e outros elementos do ambiente. Com base nessas informações, o robô pode planejar rotas eficientes, evitar obstáculos e executar tarefas de forma autônoma. A visualização do mapa é uma ferramenta poderosa para entender o ambiente em que o robô está operando e otimizar suas operações.

:::tip
Para saber como executar todo o processo de cartografia e visualização do mapa, cheque as instruções de inicialização [**aqui**](/Sprint%202/instructions).
:::

Segue abaixo também uma imagem exemplo de como é a interface do **RViz** com o mapa aberto:

![RViz](/img/cli/cart.png)

## **5.0** Conclusão

A cartografia do ambiente é um processo essencial para a navegação autônoma de robôs móveis. Com um mapa preciso do ambiente, o robô pode planejar rotas eficientes, evitar obstáculos e executar tarefas de forma autônoma. A implementação do sistema de mapeamento no projeto CORA baseado no ROS 2 permite que o robô construa um mapa do ambiente e utilize esse mapa para navegar de forma autônoma.

Ao entender os conceitos e técnicas de cartografia do ambiente, é possível otimizar o desempenho do robô e explorar todo o potencial da navegação autônoma. Com um sistema de mapeamento preciso e confiável, o robô pode operar de forma eficiente em ambientes complexos e dinâmicos, garantindo o sucesso de suas operações.