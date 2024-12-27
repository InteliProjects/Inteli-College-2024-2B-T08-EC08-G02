---

title: Documentação geral
sidebar_position: 1

---

# Resumo Consolidado

Este projeto tem como objetivo principal integrar robótica, inteligência artificial (IA) e sistemas computacionais avançados para oferecer soluções inovadoras no monitoramento e cuidado de pacientes em ambientes hospitalares e domiciliares. A seguir, um detalhamento abrangente de algumas das várias etapas envolvidas:

---

## **1. Análise de Impacto Ético**

### **Objetivo**: Identificar e mitigar os impactos éticos do uso de tecnologias em saúde, promovendo uma solução responsável e inclusiva.

- **Privacidade e Proteção de Dados**: 
  - Aplicação da LGPD e GDPR para proteger dados sensíveis coletados de pacientes.
  - Implementação de criptografia de ponta a ponta e controle rigoroso de acesso.
  - Minimização da coleta de dados para reduzir riscos e garantir a segurança das informações.

- **Equidade e Justiça**:
  - Adaptação do robô para atender diferentes grupos, respeitando variabilidades linguísticas, culturais e socioeconômicas.
  - Testes com usuários diversos para garantir interfaces inclusivas e acessíveis.

- **Transparência e Consentimento Informado**:
  - Solicitação de consentimento explícito antes de coletar dados.
  - Explicações claras sobre o uso das informações, permitindo que os pacientes revoguem o consentimento a qualquer momento.

- **Responsabilidade Social**:
  - Complementação, e não substituição, do trabalho dos profissionais de saúde.
  - Contribuição para os Objetivos de Desenvolvimento Sustentável, com foco na redução de desigualdades e no bem-estar.

- **Viés e Discriminação**:
  - Monitoramento contínuo de dados e algoritmos para evitar vieses.
  - Desenvolvimento de sistemas justos que atendam a todos os pacientes, independentemente de gênero, idade, raça ou condições regionais.

---

## **2. Proposta da Solução Inicial**

### **Objetivo**: Utilizar um robô autônomo para automatizar o monitoramento de pacientes, aumentando a eficiência e personalização do atendimento.

- **Funcionamento**:
  - O robô TurtleBot realiza perguntas de saúde diretamente aos pacientes, como dores, sintomas e avaliações gerais.
  - As informações coletadas são enviadas para um sistema central e exibidas em um dashboard para médicos e enfermeiros.

- **Integrações Tecnológicas**:
  - IBM Watson para análise de dados e geração de insights a partir das respostas dos pacientes.
  - Computação em nuvem para armazenar dados com segurança e permitir acesso remoto.

- **Benefícios**:
  - Redução da carga de trabalho de médicos e enfermeiros em tarefas repetitivas.
  - Preparação mais eficiente dos profissionais para visitas aos pacientes, com informações pré-analisadas.

---

## **3. Análise de Mercado**

### **Objetivo**: Estimar o potencial de mercado e planejar estratégias de penetração.

- **TAM (Total Addressable Market)**: Estimativa do mercado total para a solução, chegando a **R$ 392,87 milhões** por ano.
- **SAM (Service Addressable Market)**: Mercado endereçável focado em hospitais privados das capitais brasileiras, estimado em **R$ 117,86 milhões** por ano.
- **SOM (Service Obtainable Market)**: Previsão de participação no mercado no primeiro ano, alcançando **R$ 11,79 milhões**.

A análise demonstra o grande potencial da solução no setor de saúde, com estratégia inicial voltada para hospitais privados de maior porte.

---

## **4. Navegação Autônoma**

### **Objetivo**: Desenvolver um sistema de navegação robusto e confiável para o robô operar autonomamente em ambientes hospitalares.

- **Tecnologia**:
  - Utilização do pacote Navigation 2 do ROS 2, baseado em algoritmos como Dijkstra e A* para planejamento de trajetórias.
  - Integração de sensores LIDAR para percepção ambiental e detecção de obstáculos.

- **Componentes Implementados**:
  - **Planejamento de Trajetória**: Determinação da rota mais eficiente e segura entre os pontos de origem e destino.
  - **Controle do Robô**: Comandos para os motores garantirem que o robô siga o plano de forma precisa.
  - **Retorno ao Ponto de Origem**: Implementação de rotinas que permitem ao robô retornar à base automaticamente.

- **Benefícios**:
  - Redução de erros humanos na locomoção do robô.
  - Operação confiável em ambientes dinâmicos e potencialmente imprevisíveis.

---

## **5. Sistema de Anamnese**

### **Objetivo**: Automatizar a coleta inicial de dados clínicos, utilizando um assistente virtual para realizar anamnese com os pacientes.

- **Tecnologia Utilizada**:
  - IBM Watsonx Assistant para interpretar intenções e entidades na fala dos pacientes.
  - Planejamento para integrar uma LLM futuramente, tornando o sistema mais flexível e dinâmico.

- **Principais Funcionalidades**:
  - Coleta de informações sobre dor, sintomas respiratórios, estado mental e histórico médico.
  - Adaptação dinâmica das perguntas com base nas respostas do paciente.

- **Impacto**:
  - Otimização do tempo de médicos e enfermeiros ao fornecer dados clínicos estruturados.
  - Experiência personalizada para os pacientes, promovendo interações naturais e eficazes.

---

## **6. Estrutura do Backend**

### **Objetivo**: Criar um backend modular e escalável para integrar todas as funcionalidades do sistema.

- **Principais Tecnologias**:
  - FastAPI para construção de APIs.
  - MongoDB para armazenamento seguro e escalável de dados.
  - Autenticação baseada em JWT para garantir a segurança das informações.

- **Componentes**:
  - **APIs**: Rotas para autenticação, gestão de profissionais, controle de robôs e análise de dados.
  - **Serviços**: Módulos específicos para manipulação de dados e execução de operações complexas.
  - **Logs e Utilitários**: Monitoramento de atividades e funções auxiliares para suporte ao sistema.

- **Benefícios**:
  - Integração eficiente entre diferentes partes do sistema.
  - Facilidade para adicionar novas funcionalidades, mantendo a estabilidade do sistema existente.

---

## **7. Desenvolvimento do Frontend**

### **Objetivo**: Criar interfaces intuitivas e funcionais para médicos, enfermeiros e administradores.

- **Principais Telas Desenvolvidas**:
  - **Login**: Interface inicial de autenticação.
  - **Dashboard do Médico**: Resumo dos pacientes em tratamento e acesso às principais funcionalidades.
  - **Detalhes do Paciente**: Exibição de informações detalhadas, incluindo histórico médico e respostas do robô.
  - **Controle de Robôs**: Monitoramento do status e atribuição de tarefas.

- **Design Centrado no Usuário**:
  - Experiência visual agradável e alinhada às necessidades dos profissionais de saúde.
  - Layout responsivo para acessibilidade em diferentes dispositivos.

- **Impacto**:
  - Facilitação do trabalho dos profissionais, com acesso rápido e eficiente às informações.
  - Interface clara para integração com as funcionalidades do backend e controle dos robôs.

---

## **Conclusão Geral**

O projeto é um exemplo de como a tecnologia pode ser utilizada para enfrentar desafios complexos no setor de saúde. Por meio de soluções éticas e tecnológicas avançadas, foi possível integrar robótica, inteligência artificial e sistemas computacionais para melhorar a eficiência, acessibilidade e qualidade do atendimento. A estrutura modular e escalável garante a evolução contínua do sistema, permitindo adaptações e melhorias ao longo do tempo.

Os impactos esperados incluem:
- **Apoio aos profissionais de saúde**: Redução da carga de trabalho e maior eficiência no atendimento.
- **Melhoria na experiência dos pacientes**: Atendimento mais humanizado e personalizado.
- **Sustentabilidade e ética**: Respeito aos princípios éticos e contribuições para os Objetivos de Desenvolvimento Sustentável.
