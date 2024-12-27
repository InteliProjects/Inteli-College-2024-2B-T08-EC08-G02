---
title: Estrutura da LLM
sidebar_position: 1
---


# Documentação do LLM

## Introdução

A Sprint 5 teve como objetivo integrar funcionalidades avançadas relacionadas a **Modelos de Linguagem Natural (LLM)**, com foco em processamento de voz, análise de texto e interação personalizada. O módulo **LLM** foi projetado para enriquecer as interações do chatbot, oferecendo uma experiência mais responsiva e contextualizada. Isso foi alcançado através da integração de tecnologias de ponta, como **spaCy**, **IBM Watson** e **Ollama**, em um fluxo modular que facilita futuras expansões e melhorias.

---

## Funcionalidades Implementadas

### 1. **LLM (Large Language Model) - Módulo Principal**
- **Descrição**: Integra o modelo avançado **Llama 3.2**, permitindo:
  - **Compreensão contextual** da conversa, utilizando históricos de interação.
  - **Geração de perguntas dinâmicas**, adaptadas às respostas fornecidas pelo usuário.
- **Explicação Técnica**:
  - **Por que Llama 3.2?**:
    - O Llama 3.2 foi escolhido por sua capacidade de gerar respostas coerentes e alinhadas com o contexto, essencial para um chatbot humanizado.
  - **Como funciona?**:
    - O modelo é chamado com prompts que incluem o histórico da conversa e perguntas pré-definidas. Ele processa o input textual e retorna uma resposta formatada.
- **Principais Funções**:
  - **`generate_question()`**:
    - Recebe o contexto atual da conversa e utiliza o Llama 3.2 para sugerir perguntas relevantes.
    - Inclui fallback para perguntas padrão caso a geração dinâmica falhe.
  - **`analyze_text()`**:
    - Usa **spaCy** para identificar entidades nomeadas (como nomes, locais e datas) e intenções no texto do usuário.
    - Integra as entidades extraídas ao contexto da conversa para melhorar a precisão das interações.
  - **`chatbot_anamnese()`**:
    - Coordena o fluxo de interação com o usuário, alternando entre transcrição de voz, análise textual e síntese de fala.
    - Garante fluidez na conversa ao lidar com respostas imprecisas ou silêncio.

---

### 2. **Reconhecimento de Voz (Speech Recognition)**
- **Descrição**: Este módulo converte a fala do usuário em texto, utilizando a biblioteca **speech_recognition**, com suporte para o idioma português.
- **Explicação Técnica**:
  - **Configuração do Idioma**:
    - A biblioteca foi configurada para processar áudio em **pt-BR**, utilizando a API do Google Speech-to-Text para maior precisão.
  - **Gestão de Erros**:
    - Implementa tratamento de exceções para cenários como:
      - Falta de áudio (tempo limite).
      - Áudio incompreensível (erro de reconhecimento).
      - Problemas de conectividade com a API.
- **Funcionalidades**:
  - Realiza transcrição com timeout configurado para evitar travamentos.
  - Integra-se com o módulo de síntese de voz para fornecer feedback ao usuário em caso de erro.

---

### 3. **Síntese de Voz (Text-to-Speech)**
- **Descrição**: Utiliza o serviço **IBM Watson Text-to-Speech** para transformar texto em áudio, oferecendo feedback claro e compreensível ao usuário.
- **Explicação Técnica**:
  - **Por que IBM Watson?**:
    - A escolha se baseia na capacidade do Watson de oferecer vozes naturais e suporte a personalização, como ajuste de tom e estilo de fala.
  - **Processo de Geração**:
    - O texto é enviado ao serviço Watson com especificações como idioma e formato do áudio.
    - O serviço retorna um arquivo de áudio em formato MP3, que é reproduzido pelo módulo de áudio.
  - **Fallback Offline**:
    - Em caso de falha no serviço Watson, o sistema usa **pyttsx3** para gerar áudio localmente.
- **Principais Funções**:
  - **`synthesize_speech()`**:
    - Converte texto em áudio no formato MP3.
    - Suporta vozes customizadas para diferentes estilos de interação.

---

### 4. **Assistente Virtual (Watson Assistant)**
- **Descrição**: O **Watson Assistant** atua como um gerenciador de sessões, fornecendo respostas contextuais personalizadas e garantindo fluidez na interação.
- **Explicação Técnica**:
  - **Gestão de Sessões**:
    - Cada interação do usuário cria uma sessão dedicada no Watson Assistant, garantindo isolamento e rastreabilidade.
  - **Processamento de Mensagens**:
    - O assistente processa entradas complexas e fornece respostas baseadas em fluxos configurados previamente, ajustados com base no feedback do LLM.
  - **Integração com LLM**:
    - Utiliza o Llama 3.2 para complementar respostas do Watson em casos de perguntas dinâmicas.

---

## Arquitetura

O sistema foi estruturado em módulos independentes, cada um desempenhando um papel específico para garantir flexibilidade e manutenção simplificada:

1. **`llm.py`**:
   - Centraliza o fluxo de conversação.
   - Integra modelos de linguagem, análise de texto e geração de perguntas.
2. **`audio_player.py`**:
   - Reproduz os arquivos de áudio gerados pelo módulo de síntese de voz.
   - Implementa reprodução assíncrona para evitar interrupções na interação.
3. **`text_to_speech.py`**:
   - Conecta ao serviço IBM Watson Text-to-Speech para gerar arquivos de áudio.
   - Oferece suporte para diferentes idiomas e estilos de voz.
4. **`watson_assistant.py`**:
   - Gerencia sessões e interações contextuais com o Watson Assistant.

---

## Tecnologias Utilizadas

- **Bibliotecas e APIs**:
  - **speech_recognition**: Reconhecimento de fala para transcrição em texto.
  - **pyttsx3**: Solução offline para síntese de voz, usada como fallback.
  - **spaCy**: Processamento de linguagem natural para análise de entidades.
  - **ollama**: Integração com o modelo Llama 3.2 para geração de texto e perguntas.
  - **IBM Watson**:
    - Text-to-Speech para síntese de voz.
    - Assistant V2 para interações personalizadas.
- **Frameworks**:
  - **pygame**: Gerenciamento de reprodução de áudio.
  - **Threading**: Execução de tarefas assíncronas para melhorar a performance.

---

## Conclusão

A Sprint 5 alcançou um marco importante no desenvolvimento do sistema, integrando componentes avançados de linguagem natural e tecnologias de voz. O chatbot agora é capaz de:

- **Compreender e analisar entradas complexas** fornecidas pelo usuário.
- **Fornecer respostas naturais** e contextualizadas, ajustando-se ao fluxo da conversa.
- **Interagir de maneira multissensorial**, combinando áudio, texto e processamento de linguagem.

Essa evolução posiciona o sistema como uma solução robusta para interações dinâmicas e humanizadas. A arquitetura modular garante escalabilidade e facilidade de manutenção, permitindo expansões futuras. Com base nas funcionalidades entregues, o sistema está pronto para atender a cenários mais complexos e personalizados, tornando-se uma ferramenta poderosa para suporte ao usuário e coleta de informações.

---
