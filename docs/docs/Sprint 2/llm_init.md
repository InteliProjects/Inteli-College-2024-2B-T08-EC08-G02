---
sidebar_position: 1
title: Sistema de Anamnese 
---

# Sistema de Anamnese

Para automatizar a parte de anamnese de médicos, iniciamos a construção de uma LLM, utilizando o **IBM Watsonx Assistant** para interagir com pacientes hospitalizados, conduzindo uma avaliação clínica inicial (anamnese). O assistente faz perguntas relacionadas aos sintomas, histórico médico e estado mental do paciente, coletando informações relevantes que serão posteriormente enviadas para o médico responsável. 

:::warning
Atualmente, estamos utilizando o IBM Watsonx Assistant para estruturar diálogos com base em intenções e entidades, o que permite um controle mais preciso na coleta de informações. Futuramente, planeja-se integrar uma LLM para tornar o sistema mais flexível e dinâmico, mas é essencial começar com o Watsonx Assistant para garantir precisão nos primeiros estágios da anamnese.
:::

O **Watsonx Assistant** interpreta as intenções e entidades mencionadas pelos pacientes e ajusta as perguntas e diálogos conforme necessário, gerando uma conversa fluida e personalizada para cada paciente.

![alt text](../../static/gif.gif)
---

## Funcionalidades

O assistente virtual é capaz de:

1. **Entender Intenções**: Identificar o propósito da interação do paciente, como relatar dor, sintomas respiratórios, ou estado mental.
2. **Identificar Entidades**: Extrair dados específicos da conversa, como a intensidade da dor, localização do sintoma ou a duração de um problema.
3. **Adaptar o Diálogo**: Com base nas respostas, o assistente continua a anamnese, perguntando de maneira relevante e contextualizada.

---

## Estrutura do Sistema

### 1. Intenções

As **intenções** representam o propósito por trás da fala do paciente. A seguir, listamos algumas das intenções utilizadas no sistema:

| Intenção | Exemplos de Frases |
|----------|--------------------|
| **Relatar_dor** | "Eu estou com dor", "Minha cabeça está doendo", "Estou com dor forte" |
| **Condição_respiratoria** | "Estou com falta de ar", "Estou respirando normalmente", "Sinto dificuldade para respirar" |
| **Duração_sintomas** | "Esses sintomas aparecem há dias", "A dor começou ontem à noite", "Faz três dias que estou sentindo isso" |
| **Estado_mental** | "Estou feliz", "Estou triste", "Estou me sentindo ansioso", "Estou deprimido" |
| **Febre** | "Estou com febre", "Minha temperatura está normal", "Tive febre durante a noite" |
| **Intensidade_dor** | "A dor é leve", "A dor é insuportável", "A dor vai e vem" |
| **Localização_dor** | "Sinto dor no peito", "A dor é na cabeça", "Minha dor é nas costas" |

### 2. Entidades

As **entidades** são informações específicas dentro das respostas dos pacientes, como localização do sintoma, duração e intensidade da dor. Abaixo, listamos as principais entidades e seus valores:

| Entidade | Valores Possíveis |
|----------|-------------------|
| **afirmacao** | "Entendido", "Sim", "Ok", "Certo", "Afirmativo" |
| **condicoes_respiratorias** | "Respiração normal", "Falta de ar", "Tosse seca", "Dificuldade para respirar" |
| **duracao** | "Dias", "Semanas", "Desde ontem", "Hoje de manhã" |
| **estado_mental** | "Feliz", "Depressão", "Ansioso", "Triste", "Estressado" |
| **febre** | "Febre baixa", "Febre alta", "Temperatura normal", "Sem febre" |
| **intensidade** | "Leve", "Moderada", "Forte", "Insuportável" |
| **local** | "Cabeça", "Peito", "Costas", "Perna", "Braço", "Estômago" |
| **problemas_digestivos** | "Azia", "Diarreia", "Náusea", "Inchaço", "Vômito" |
| **tipo_dor** | "Aguda", "Pontada", "Queimação", "Latejante", "Pressão" |

---

## Exemplo de Diálogo

Aqui está um exemplo de como o diálogo entre o paciente e o assistente pode ocorrer:

1. **Assistente**: "Você está sentindo dor?"
2. **Paciente**: "Sim, estou com dor."
   - **Intenção**: `Relatar_dor`
   - **Entidade**: `afirmacao: Sim`
   
3. **Assistente**: "Onde você está sentindo dor?"
4. **Paciente**: "Sinto dor no lado esquerdo do peito."
   - **Intenção**: `Localização_dor`
   - **Entidade**: `local: Peito esquerdo`

5. **Assistente**: "Qual a intensidade da dor?"
6. **Paciente**: "É uma dor forte."
   - **Intenção**: `Intensidade_dor`
   - **Entidade**: `intensidade: Forte`

7. **Assistente**: "Há quanto tempo você está sentindo essa dor?"
8. **Paciente**: "A dor começou ontem à noite."
   - **Intenção**: `Duração_sintomas`
   - **Entidade**: `duracao: Desde ontem`

---

## Integração e Configuração

### Passo 1: Criar Intenções no Watsonx Assistant
- Acesse o Watsonx Assistant e crie intenções baseadas nas necessidades da anamnese.
- Exemplo: Para a intenção de **Relatar_dor**, inclua exemplos de frases como "Estou com dor", "Minha cabeça está doendo", "Tenho uma dor nas costas."

### Passo 2: Criar Entidades
- Crie entidades para capturar informações específicas. Por exemplo:
  - Entidade `local` para capturar o local da dor (e.g., cabeça, peito, costas).
  - Entidade `intensidade` para capturar a intensidade da dor (e.g., leve, moderada, forte).

### Passo 3: Configuração dos Diálogos
- No Watsonx Assistant, crie diálogos que façam perguntas baseadas nas intenções e entidades detectadas.
- Exemplo: Quando a intenção de **Relatar_dor** for detectada, pergunte "Onde está sentindo dor?" e, dependendo da resposta, continue com perguntas sobre intensidade, duração, etc.

---

## Conclusão

Este sistema de anamnese automatizada, utilizando o **IBM Watsonx Assistant**, permite realizar uma avaliação clínica inicial de maneira eficiente e personalizada, coletando informações para o diagnóstico médico. O uso de intenções e entidades garante que as interações sejam naturais e adaptáveis, facilitando o processo de coleta de dados dos pacientes. Lembrando, está sendo implementado dessa forma inicial apenas para posteriormente ser incrementado com LLM.

