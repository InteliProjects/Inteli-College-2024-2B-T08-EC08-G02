# Matriz de Risco

A Matriz de Avaliação de Riscos, também conhecida como matriz de Probabilidade e Impacto, é uma ferramenta visual que representa os riscos potenciais que podem afetar um negócio ou projeto. A matriz de risco é baseada em dois fatores que se cruzam: a probabilidade de o evento de risco ocorrer e o impacto potencial que esse evento pode causar. Em outras palavras, é uma ferramenta que ajuda a visualizar a probabilidade em relação à gravidade de um risco potencial. [1]

Este documento apresenta a matriz de risco e oportunidades para o projeto de robô autônomo de acompanhamento de pacientes. Foram listados 10 riscos e 6 oportunidades, que foram distribuídos na matriz de risco considerando os seguintes fatores:

- **Probabilidade**: A chance de o evento ocorrer, classificada em 10%, 30%, 50%, 70%, 90%.
- **Impacto**: O efeito que o evento causaria no projeto, classificado de Muito Baixo a Muito Alto.

A matriz é dividida em dois principais quadrantes:
- **Riscos**: Identificação de fatores que podem afetar negativamente o sucesso do projeto.
- **Oportunidades**: Identificação de fatores que podem agregar valor e impulsionar o projeto.

![image](/img/negocios/matriz_risco.png)


---

# Riscos 

## 1. Falha crítica no sistema de sensores de saúde
- **Probabilidade**: 50%
- **Impacto**: Muito Alto
- **Justificativa**: Se os sensores responsáveis por monitorar sinais vitais (como batimentos cardíacos e temperatura) falharem devido a desgaste, interferência eletrônica ou falta de calibração, o robô pode gerar dados incorretos ou deixar de enviar alertas críticos. Isso pode atrasar a intervenção médica, colocando a vida do paciente em risco.
- **Mitigação**: Implementar um cronograma de manutenção preventiva para os sensores e desenvolver um sistema de redundância de sensores.

## 2. Erro de software
- **Probabilidade**: 50%
- **Impacto**: Alto
- **Justificativa**: Erros no código de monitoramento ou nas atualizações automáticas do sistema podem causar a perda de informações cruciais de saúde do paciente. Por exemplo, um bug pode fazer com que o robô deixe de identificar sinais críticos de alerta, como a não responsividade do paciente. Esses erros podem ser introduzidos após atualizações do software, o que exige testes rigorosos antes de sua implementação no ambiente hospitalar.
- **Mitigação**: Adotar um processo de controle de qualidade rigoroso antes de liberar atualizações, incluindo testes de regressão, bem como programas piloto em ambientes controlados antes do lançamento geral.

## 3. Conectividade instável
- **Probabilidade**: 50%
- **Impacto**: Moderado
- **Justificativa**: Problemas na infraestrutura de rede hospitalar ou falhas nos módulos de comunicação do robô podem interromper a troca de dados em tempo real entre o robô e os servidores médicos. Isso pode levar à perda de relatórios ou atrasos na emissão de alertas críticos. Esse risco pode ser mais grave em hospitais localizados em áreas com infraestrutura de rede instável.
- **Mitigação**: Incorporar funcionalidades de fallback que permitem o armazenamento temporário de dados localmente no robô e realizar a sincronização assim que a conectividade for restabelecida.

## 4. Dependência excessiva de conectividade de rede para funções críticas
- **Probabilidade**: 30%
- **Impacto**: Alto
- **Justificativa**: Em ambientes hospitalares com redes instáveis ou em áreas com infraestrutura de internet precária, a dependência do robô em conexões contínuas de rede pode resultar em perda de comunicação ou de dados importantes. A desconexão pode impedir o envio de alertas em tempo real, o que poderia comprometer a saúde dos pacientes.
- **Mitigação**: Incorporar funcionalidades offline no robô, de modo que ele armazene os dados localmente e sincronize quando a conexão for restabelecida.

## 5. Ataque cibernético e exposição de dados de pacientes
- **Probabilidade**: 10%
- **Impacto**: Muito Alto
- **Justificativa**: Hackers podem explorar vulnerabilidades na infraestrutura de rede ou no software do robô para acessar informações confidenciais de pacientes. Isso pode resultar em grandes multas por violação de regulamentos de privacidade de dados – como LGPD –, além de danos irreparáveis à reputação da empresa e do hospital.
- **Mitigação**: Fortalecer a segurança do sistema com criptografia de ponta a ponta, testes regulares de penetração, programas de resposta a incidentes de cibersegurança, e implementar firewalls robustos e protocolos de monitoramento contínuo.

## 6. Sobrecarga de dados e dificuldade de armazenamento e processamento
- **Probabilidade**: 30%
- **Impacto**: Moderado
- **Justificativa**: Se o robô gerar grandes 'volumes de dados de monitoramento de pacientes sem que haja uma infraestrutura robusta para armazenar e processar esses dados, o hospital pode enfrentar dificuldades para gerenciar essas informações, levando a perdas ou atrasos no acesso aos dados mais recentes.
- **Mitigação**: Utilizar soluções de armazenamento em nuvem com escalabilidade automática e implementar algoritmos de compressão e processamento em tempo real para otimizar o uso de dados.

## 7. Aceitação limitada do robô por parte dos pacientes idosos
- **Probabilidade**: 50%
- **Impacto**: Moderado
- **Justificativa**: Pacientes mais velhos, que estão menos familiarizados com tecnologia, podem se sentir desconfortáveis ao interagir com o robô. Isso pode levar a uma diminuição na eficácia do monitoramento, já que o robô depende da colaboração do paciente para realizar algumas tarefas, como responder a perguntas de avaliação de saúde.
- **Mitigação**: Implementar uma fase de adaptação, onde os pacientes recebam orientações sobre como interagir com o robô de forma amigável e intuitiva, além de ajustes na interface para torná-la mais simples e humanizada.

## 8. Incompatibilidade entre o sistema do robô e o prontuário eletrônico do hospital
- **Probabilidade**: 70%
- **Impacto**: Alto
- **Justificativa**: Se o robô não puder se integrar perfeitamente ao sistema de prontuário eletrônico do hospital, os dados coletados pelo robô podem não ser registrados automaticamente no sistema, resultando em trabalho manual adicional para os médicos e aumentando a chance de erros humanos na transferência de informações.
- **Mitigação**: Garantir que o software do robô seja compatível com padrões universais de interoperabilidade de sistemas de saúde, como HL7, e realizar testes de integração antes da implantação.

## 9. Incapacidade de escalar a operação para outros hospitais
- **Probabilidade**: 30%
- **Impacto**: Muito Alto
- **Justificativa**: Pode ser difícil ou caro adaptar o robô para outros hospitais com diferentes infraestruturas, regulamentos ou processos. Isso pode limitar a escalabilidade e a adoção em massa.
- **Mitigação**: Desenvolver o robô com uma arquitetura modular e flexível, capaz de se adaptar a diferentes configurações hospitalares e integrá-lo facilmente a novos sistemas.

## 10. Falta de treinamento e capacitação adequada da equipe médica
- **Probabilidade**: 10%
- **Impacto**: Alto
- **Justificativa**: Se os médicos, enfermeiros e outros profissionais de saúde não forem adequadamente treinados para utilizar o robô, eles podem cometer erros operacionais, comprometer a eficácia do monitoramento dos pacientes e rejeitar o uso da tecnologia. A falta de conhecimento sobre as funcionalidades do robô também pode gerar resistência à sua adoção.
- **Mitigação**: Estabelecer programas contínuos de treinamento prático, workshops regulares e suporte técnico para garantir que os profissionais de saúde estejam familiarizados com todas as funcionalidades do robô e saibam utilizá-lo corretamente em diferentes situações clínicas.

---

# Oportunidades

## 1. Melhoria na Gestão de Tempo da Equipe
- **Probabilidade**: 90%
- **Impacto**: Muito Alto
- **Justificativa**: O robô de serviço permitirá que as equipes médicas e de enfermagem tenham um maior nível de informação acerca dos pacientes, mesmo que a distância. Por consequência, essas equipes poderão tomar decisões mais certeiras quanto ao atendimento, sem a necessidade de acompanhamento presencial de todos os pacientes, uma vez que aqueles em situações mais controladas poderão ser acompanhados somente pelos robôs de serviço.

## 2. Maior Conforto para Pacientes Solitários
- **Probabilidade**: 30%
- **Impacto**: Médio
- **Justificativa**: Para aqueles pacientes que se encontrem em momentos solitários e não estão incapacitados de se comunicar, além de terem interesse no contato com a tecnologia, o robô de serviço poderá manter uma conversa, visando diminuir o desconforto do paciente nesse momento. Apesar de ter um ganho de valor médio, essa oportunidade depende tanto do interesse do paciente quanto da sua capacidade de comunicação.

## 3. Maior Acesso a Informações Internas
- **Probabilidade**: 70%
- **Impacto**: Alto
- **Justificativa**: O sistema promove uma maior disseminação da informação sobre os pacientes entre as equipes médicas e de enfermagem, permitindo que a equipe responsável esteja na mesma página quanto ao quadro do paciente. Nesse sentido, seria promovida uma melhor coordenação entre a equipe e seria reduzida a possibilidade de erros por falhas de comunicação.

## 4. Redução de Custos Operacionais
- **Probabilidade**: 70%
- **Impacto**: Alto
- **Justificativa**: A adoção do robô pode reduzir a necessidade de pessoal dedicado a monitorar constantemente os pacientes, otimizando a alocação de recursos humanos. Isso diminui os custos com profissionais para tarefas rotineiras e permite que o foco seja em atividades mais críticas e de maior valor.

## 5. Integração com Outros Sistemas de Saúde
- **Probabilidade**: 50%
- **Impacto**: Muito Alto
- **Justificativa**: O robô pode ser integrado com prontuários eletrônicos e outros sistemas hospitalares, facilitando o compartilhamento de informações entre diferentes profissionais de saúde. Isso pode melhorar a coordenação do atendimento e reduzir a fragmentação das informações do paciente.


## Referências
[1] AuditBoard. (2023). What is a Risk Assessment Matrix? AuditBoard. Disponível em: https://www.auditboard.com/blog/what-is-a-risk-assessment-matrix/
