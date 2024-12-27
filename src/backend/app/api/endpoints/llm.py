from fastapi import APIRouter, HTTPException, UploadFile, File
from app.schemas.answer import AnswerCreate, AnswerSchema, AnswerResponse
from app.schemas.question import QuestionSchema, QuestionResponse
from app.services.llm.llm import OpenAIService
import logging

router = APIRouter(prefix="/llm", tags=["LLM"])

# Initialize OpenAIService
openai_service = OpenAIService(api_key="chave")

@router.post("/generate-question", response_model=QuestionSchema)
async def generate_question(context: str, patient_id: str):
    """
    Gera uma nova pergunta com base no contexto fornecido e salva no banco.
    """
    try:
        question = await openai_service.generate_and_save_question(context, patient_id)
        if not question.id:
            raise HTTPException(status_code=500, detail="Erro ao gerar pergunta.")
        return question
    except Exception as e:
        logging.error(f"Erro ao gerar pergunta: {e}")
        raise HTTPException(status_code=500, detail="Erro ao gerar pergunta.")

@router.post("/save-answer", response_model=AnswerSchema)
async def save_answer(answer_data: AnswerCreate):
    """
    Salva a resposta de uma pergunta no banco de dados.
    """
    try:
        created_answer = await openai_service.answer_service.create_answer(answer_data)
        return created_answer
    except Exception as e:
        logging.error(f"Erro ao salvar resposta: {e}")
        raise HTTPException(status_code=500, detail="Erro ao salvar resposta.")

@router.get("/questions/{patient_id}", response_model=QuestionResponse)
async def get_questions_by_patient(patient_id: str):
    """
    Lista todas as perguntas associadas a um paciente específico.
    """
    try:
        questions = await openai_service.question_service.get_question_by_patient_id(patient_id)
        return QuestionResponse(questions=questions)
    except Exception as e:
        logging.error(f"Erro ao listar perguntas: {e}")
        raise HTTPException(status_code=500, detail="Erro ao listar perguntas.")

@router.get("/answers", response_model=AnswerResponse)
async def list_all_answers():
    """
    Lista todas as respostas armazenadas.
    """
    try:
        answers = await openai_service.answer_service.get_answers()
        return AnswerResponse(answers=answers)
    except Exception as e:
        logging.error(f"Erro ao listar respostas: {e}")
        raise HTTPException(status_code=500, detail="Erro ao listar respostas.")

@router.post("/transcribe", response_model=dict)
async def transcribe_audio(file: UploadFile = File(...)):
    """
    Recebe um arquivo de áudio e o transcreve em texto usando reconhecimento de fala.
    """
    try:
        with open(f"/tmp/{file.filename}", "wb") as buffer:
            buffer.write(await file.read())

        transcript = openai_service.transcribe_audio()
        if not transcript:
            raise HTTPException(status_code=500, detail="Erro ao transcrever áudio.")
        return {"transcript": transcript}
    except Exception as e:
        logging.error(f"Erro ao transcrever áudio: {e}")
        raise HTTPException(status_code=500, detail="Erro ao transcrever áudio.")

@router.post("/run-anamnese", response_model=dict)
async def run_anamnese(patient_id: str):
    """
    Inicia a anamnese interativa com um paciente e salva as respostas no banco de dados.
    """
    try:
        await openai_service.run_anamnese(patient_id)
        return {"message": "Anamnese concluída com sucesso."}
    except Exception as e:
        logging.error(f"Erro ao executar anamnese: {e}")
        raise HTTPException(status_code=500, detail="Erro ao executar anamnese.")
