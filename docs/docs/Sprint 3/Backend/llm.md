---
title: Estrutura do serviço de llm no backend
sidebar_position: 4
---

## **1.0** Introdução

Este documento tem como objetivo apresentar a estrutura do serviço de Large Language Models (LLM) no backend, detalhando como ele foi projetado para suportar operações de processamento de linguagem natural. O serviço é uma peça central na integração de modelos avançados, permitindo interações eficientes e flexíveis com diferentes partes do sistema.

Além disso, o documento explora a organização do serviço, suas funcionalidades principais e como ele se conecta a outros módulos do backend. Essa abordagem busca fornecer uma compreensão abrangente e técnica, tanto para desenvolvedores quanto para profissionais interessados em entender sua arquitetura e potencialidades.

## **2.0** Estrutura de arquivos

A estrutura de arquivos do serviço de LLM é organizada de forma a facilitar a manutenção e a expansão do código. Ela é composta por diferentes módulos, cada um responsável por uma parte específica do serviço. A seguir, são apresentados os principais arquivos e diretórios do serviço:

### **2.1** `api/endpoints/llm.py`

Este arquivo contém a definição dos endpoints do serviço de LLM, que são responsáveis por receber as requisições dos clientes e processá-las de acordo com as operações solicitadas. Ele é o ponto de entrada para as operações de processamento de linguagem natural, permitindo a interação com os modelos de LLM.

Segue abaixo o mesmo:

```python
from typing import List
from fastapi import APIRouter, HTTPException, UploadFile, File, Depends
from schemas.answer import AnswerCreate, AnswerSchema, AnswerResponse
from schemas.question import QuestionCreate, QuestionSchema, QuestionResponse
from services.llm.llm import WatsonService  # Classe implementada anteriormente
import logging

router = APIRouter(prefix="/llm", tags=["LLM"])

# Instância do WatsonService
assistant_config = {
    "apikey": "REDACTED",
    "url": "REDACTED",
    "version": "REDACTED",
    "assistant_id": "REDACTED"
}
stt_config = {
    "apikey": "REDACTED",
    "url": "REDACTED"
}
watson_service = WatsonService(assistant_config, stt_config)


@router.post("/questions", response_model=QuestionSchema)
async def add_question(question_data: QuestionCreate):
    """
    Adiciona uma nova pergunta ao banco de perguntas.
    """
    question = await watson_service.add_question(question_data)
    return question


@router.get("/questions", response_model=QuestionResponse)
async def list_questions():
    """
    Lista todas as perguntas cadastradas.
    """
    questions = await watson_service.get_questions()
    return QuestionResponse(questions=questions)


@router.post("/answers", response_model=AnswerSchema)
async def save_answer(answer_data: AnswerCreate):
    """
    Salva a resposta de uma pergunta específica.
    """
    answer = await watson_service.save_answer(answer_data)
    return answer


@router.get("/answers", response_model=AnswerResponse)
async def list_answers():
    """
    Lista todas as respostas cadastradas.
    """
    answers = await watson_service.get_answers()
    return AnswerResponse(answers=answers)


@router.post("/transcribe", response_model=dict)
async def transcribe_audio(file: UploadFile = File(...)):
    """
    Transcreve um arquivo de áudio para texto usando o Watson Speech to Text.
    """
    try:
        file_path = f"/tmp/{file.filename}"
        with open(file_path, "wb") as buffer:
            buffer.write(await file.read())

        transcript = await watson_service.transcribe_audio(file_path)
        if not transcript:
            raise HTTPException(status_code=500, detail="Erro ao transcrever o áudio")
        return {"transcript": transcript}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/assistant", response_model=dict)
async def interact_with_assistant(user_input: str):
    """
    Interage com o Watson Assistant enviando uma mensagem e retornando a resposta.
    """
    try:
        logging.info(f"Passou por /assistant com a mensagem: {user_input}")
        response = await watson_service.send_message_to_assistant(user_input)
        if not response:
            raise HTTPException(
                status_code=500, detail="Erro: Resposta do Watson Assistant está vazia ou inválida."
            )
        return {"response": response}
    except HTTPException as e:
        raise e
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Erro inesperado: {str(e)}"
        )



@router.post("/run-anamnese", response_model=dict)
async def run_anamnese():
    """
    Executa a anamnese interativa com perguntas e salva as respostas.
    """
    try:
        await watson_service.run_anamnese()
        answers = await watson_service.get_answers()
        return {"message": "Anamnese concluída", "answers": answers}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

Neste arquivo, são definidos os endpoints `/llm/questions`, `/llm/answers`, `/llm/transcribe`, `/llm/assistant` e `/llm/run-anamnese`, que permitem adicionar perguntas, salvar respostas, transcrever áudio, interagir com o Watson Assistant e executar a anamnese interativa, respectivamente.

Cada endpoints tem uma finalidade específica, como adicionar perguntas ao banco de dados, salvar respostas, transcrever áudio para texto, interagir com o assistente virtual e executar a anamnese interativa. Eles são implementados por meio de funções assíncronas que chamam métodos da classe `WatsonService`, responsável por realizar as operações de processamento de linguagem natural.

### **2.2** `schemas/question.py`

Este arquivo contém as definições dos esquemas de dados relacionados às perguntas, que são utilizados para validar e serializar os dados recebidos nos endpoints do serviço de LLM. Ele define as estruturas de dados `QuestionCreate`, `QuestionSchema` e `QuestionResponse`, que representam as perguntas enviadas pelos clientes, as perguntas armazenadas no banco de dados e as respostas retornadas pelos endpoints, respectivamente.

Segue abaixo o mesmo:

```python
from typing import Optional, List
from pydantic import BaseModel, Field, ConfigDict


class QuestionCreate(BaseModel):
    text: str = Field(..., description="The text of the question")
    q_type: str = Field(..., description="The type of the question")
    options: Optional[List[str]] = Field(
        None, description="The options of the question if applicable"
    )


class QuestionSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int = Field(..., description="The ID of the question")
    text: str
    q_type: str
    options: Optional[List[str]]


class QuestionResponse(BaseModel):
    questions: List[QuestionSchema]
```

Neste arquivo, são definidas as classes `QuestionCreate`, `QuestionSchema` e `QuestionResponse`, que representam as estruturas de dados relacionadas às perguntas. A classe `QuestionCreate` é utilizada para validar os dados recebidos nos endpoints de adição de perguntas, enquanto as classes `QuestionSchema` e `QuestionResponse` são utilizadas para serializar as perguntas armazenadas no banco de dados e as respostas retornadas pelos endpoints, respectivamente.

### **2.3** `schemas/answer.py`

Este arquivo contém as definições dos esquemas de dados relacionados às respostas, que são utilizados para validar e serializar os dados recebidos nos endpoints do serviço de LLM. Ele define as estruturas de dados `AnswerCreate`, `AnswerSchema` e `AnswerResponse`, que representam as respostas enviadas pelos clientes, as respostas armazenadas no banco de dados e as respostas retornadas pelos endpoints, respectivamente.

Segue abaixo o mesmo:

```python
from pydantic import BaseModel, Field, ConfigDict
from datetime import datetime
from typing import List


class AnswerCreate(BaseModel):
    id_patient: int = Field(..., description="The ID of the patient")
    id_question: int = Field(..., description="The ID of the question")
    answer: str = Field(..., description="The answer to the question")


class AnswerSchema(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int = Field(..., description="The ID of the answer")
    id_patient: int
    id_question: int
    answer: str
    date_hour: datetime = Field(default_factory=datetime.now)


class AnswerResponse(BaseModel):
    answers: List[AnswerSchema]
```

Neste arquivo, são definidas as classes `AnswerCreate`, `AnswerSchema` e `AnswerResponse`, que representam as estruturas de dados relacionadas às respostas. A classe `AnswerCreate` é utilizada para validar os dados recebidos nos endpoints de salvamento de respostas, enquanto as classes `AnswerSchema` e `AnswerResponse` são utilizadas para serializar as respostas armazenadas no banco de dados e as respostas retornadas pelos endpoints, respectivamente.

### **2.4** `services/llm/llm.py`

Este arquivo contém a implementação da classe `WatsonService`, responsável por realizar as operações de processamento de linguagem natural no serviço de LLM. Ela é responsável por interagir com os modelos de LLM, realizar a anam

Segue abaixo o mesmo:

```python
import logging
from ibm_watson import AssistantV2, SpeechToTextV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from typing import Optional, List
from schemas.answer import AnswerCreate, AnswerSchema
from schemas.question import QuestionCreate, QuestionSchema

# Configuração básica do logger
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("watson_service.log"),
        logging.StreamHandler()
    ]
)

class WatsonService:
    def __init__(self, assistant_config: dict, stt_config: dict):
        logging.info("Inicializando WatsonService...")

        # Configurações do Watson Assistant
        try:
            self.assistant_authenticator = IAMAuthenticator(assistant_config["apikey"])
            self.assistant = AssistantV2(
                version=assistant_config["version"],
                authenticator=self.assistant_authenticator
            )
            self.assistant.set_service_url(assistant_config["url"])
            self.assistant_id = assistant_config["assistant_id"]
            logging.info("Watson Assistant configurado com sucesso.")
        except Exception as e:
            logging.error(f"Erro ao configurar Watson Assistant: {e}")
            raise e

        # Configurações do Watson Speech to Text
        try:
            self.stt_authenticator = IAMAuthenticator(stt_config["apikey"])
            self.speech_to_text = SpeechToTextV1(authenticator=self.stt_authenticator)
            self.speech_to_text.set_service_url(stt_config["url"])
            logging.info("Watson Speech to Text configurado com sucesso.")
        except Exception as e:
            logging.error(f"Erro ao configurar Watson Speech to Text: {e}")
            raise e

        # Armazena respostas e perguntas
        self.answers: List[AnswerSchema] = []
        self.questions: List[QuestionSchema] = []

    async def transcribe_audio(self, file_path: str) -> Optional[str]:
        """
        Converte áudio em texto usando o Watson Speech to Text.
        """
        logging.info(f"Iniciando transcrição de áudio: {file_path}")
        try:
            with open(file_path, 'rb') as audio_file:
                response = self.speech_to_text.recognize(
                    audio=audio_file,
                    content_type='audio/wav'
                ).get_result()
            transcript = response['results'][0]['alternatives'][0]['transcript']
            logging.info(f"Transcrição concluída com sucesso: {transcript}")
            return transcript
        except Exception as e:
            logging.error(f"Erro ao transcrever áudio: {e}")
            return None

    async def send_message_to_assistant(self, text: str) -> Optional[str]:
        """
        Envia mensagem ao Watson Assistant e retorna a resposta.
        """
        logging.info(f"Enviando mensagem para o assistente: {text}")
        try:
            response = self.assistant.message_stateless(
                assistant_id=self.assistant_id,
                input={
                    'message_type': 'text',
                    'text': text,
                    'options': {
                        'return_context': True
                    }
                }
            ).get_result()
            logging.debug(f"Resposta do assistente recebida: {response}")

            if 'output' in response and 'generic' in response['output']:
                generic_responses = response['output']['generic']
                if len(generic_responses) > 0 and 'text' in generic_responses[0]:
                    logging.info("Mensagem do assistente processada com sucesso.")
                    return generic_responses[0]['text']
                else:
                    logging.warning("Resposta genérica não encontrada ou inválida.")
                    raise ValueError("Resposta genérica não encontrada ou inválida.")
            else:
                logging.warning("Resposta do Watson Assistant não contém saída válida.")
                raise ValueError("Resposta do Watson Assistant não contém saída válida.")
        except Exception as e:
            logging.error(f"Erro ao enviar mensagem ao assistente: {e}")
            return None

    async def add_question(self, question_data: QuestionCreate) -> QuestionSchema:
        """
        Adiciona uma nova pergunta à lista de perguntas.
        """
        logging.info(f"Adicionando nova pergunta: {question_data}")
        question_id = len(self.questions) + 1
        question = QuestionSchema(id=question_id, **question_data.dict())
        self.questions.append(question)
        logging.debug(f"Pergunta adicionada com sucesso: {question}")
        return question

    async def get_questions(self) -> List[QuestionSchema]:
        """
        Retorna todas as perguntas.
        """
        logging.info("Recuperando todas as perguntas.")
        return self.questions

    async def save_answer(self, answer_data: AnswerCreate) -> AnswerSchema:
        """
        Salva uma nova resposta.
        """
        logging.info(f"Salvando nova resposta: {answer_data}")
        answer_id = len(self.answers) + 1
        answer = AnswerSchema(id=answer_id, **answer_data.dict())
        self.answers.append(answer)
        logging.debug(f"Resposta salva com sucesso: {answer}")
        return answer

    async def get_answers(self) -> List[AnswerSchema]:
        """
        Retorna todas as respostas.
        """
        logging.info("Recuperando todas as respostas.")
        return self.answers

    async def run_anamnese(self):
        """
        Realiza a anamnese interativa com o paciente.
        """
        logging.info("Iniciando a anamnese interativa.")
        print("Iniciando a anamnese com o paciente...\n")
        for question in self.questions:
            logging.info(f"Pergunta: {question.text}")
            print(f"Assistente: {question.text}")
            user_input = input("Paciente: ")
            answer_data = AnswerCreate(
                id_patient=1,  # ID fictício do paciente
                id_question=question.id,
                answer=user_input
            )
            await self.save_answer(answer_data)
        logging.info("Anamnese concluída. Respostas registradas.")
        print("Anamnese concluída. Respostas registradas.")
```

Neste arquivo, é implementada a classe `WatsonService`, que é responsável por realizar as operações de processamento de linguagem natural no serviço de LLM. Ela interage com os modelos

### **2.5** `services/llm/base.py`

Este arquivo contém a definição da classe base `BaseService`, que é utilizada como base para os serviços de LLM. Ela define métodos e propriedades comuns a todos os serviços, permitindo a reutilização de código e a padronização da implementação.

Segue abaixo o mesmo:

```python
from abc import ABC, abstractmethod
from typing import Optional


class WatsonBase(ABC):
    @abstractmethod
    async def transcribe_audio(self, file_path: str) -> Optional[str]:
        """
        Converte áudio em texto usando o Watson Speech to Text.
        """
        pass

    @abstractmethod
    async def send_message_to_assistant(self, text: str) -> Optional[str]:
        """
        Envia mensagem ao Watson Assistant e retorna a resposta.
        """
        pass

    @abstractmethod
    async def run_anamnese(self):
        """
        Realiza a anamnese interativa com o paciente.
        """
        pass
```

Neste arquivo, é definida a classe abstrata `BaseService`, que serve como base para os serviços de LLM. Ela define métodos abstratos para as operações de transcrição de áudio, interação com o assistente virtual e execução da anamnese interativa, permitindo a implementação dessas operações em classes concretas.

## **3.0** Conclusão

Este documento apresentou a estrutura do serviço de LLM no backend, detalhando como ele foi projetado para suportar operações de processamento de linguagem natural. Através da organização dos arquivos e diretórios, foi possível explorar as funcionalidades principais do serviço, bem como sua conexão com outros módulos do backend.

Além disso, foram apresentados os principais arquivos e suas respectivas funcionalidades, como a definição dos endpoints, a validação dos dados recebidos, a interação com os modelos de LLM e a execução da anamnese interativa. Essa abordagem permitiu uma compreensão abrangente e técnica da arquitetura do serviço, facilitando sua manutenção e expansão no futuro.

Por fim, o documento ressaltou a importância do serviço de LLM como uma peça central na integração de modelos avançados, permitindo interações eficientes e flexíveis com diferentes partes do sistema. Sua arquitetura bem definida e modularizada contribui para a escalabilidade e robustez do sistema, garantindo um alto nível de desempenho e confiabilidade em operações de processamento de linguagem natural.