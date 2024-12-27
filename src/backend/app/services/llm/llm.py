import logging
import speech_recognition as sr
import spacy
import openai
import tempfile
import os
from playsound import playsound
from pathlib import Path
from app.schemas.answer import AnswerCreate
from app.schemas.question import QuestionSchema, QuestionCreate
from app.services.crud_answers.answer import AnswerService
from app.services.crud_questions.question import QuestionService
from app.core.db import mongo_db
from typing import Optional


class OpenAIService:
    def __init__(self, api_key: str):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Initializing OpenAIService...")

        self.api_key = api_key
        openai.api_key = self.api_key

        self.logger.info("Loading NER model...")
        self.nlp = spacy.load("pt_core_news_sm")

        # Inject MongoDB collections into services
        self.answer_service = AnswerService(mongo_db.get_collection("answers"))
        self.question_service = QuestionService(mongo_db.get_collection("questions"))

    def detect_stop_intent(self, text: str) -> bool:
        """
        Detects if the user wants to stop the anamnese session.
        Returns True if a stop intent is detected, otherwise False.
        """
        stop_keywords = ["encerrar", "parar", "acabou", "finalizar"]
        text_lower = text.lower()
        return any(keyword in text_lower for keyword in stop_keywords)

    def speak(self, text: str):
        """Converts text to speech using OpenAI's TTS API and plays it."""
        try:
            # Caminho para salvar o arquivo de áudio temporário
            speech_file_path = Path(__file__).parent / "speech.mp3"
            
            # Chama a API OpenAI para gerar o áudio em MP3
            response = openai.audio.speech.create(
                model="tts-1",  # Modelos: "tts-1" ou "tts-1-hd"
                voice="alloy",  # Opções de voz: alloy, echo, fable, onyx, nova, shimmer
                input=text
            )

            # Salva o arquivo MP3
            response.stream_to_file(speech_file_path)

            # Reproduz o arquivo MP3 usando playsound
            playsound(str(speech_file_path))

            # Remove o arquivo após a reprodução
            os.remove(speech_file_path)

        except Exception as e:
            print(f"Error generating speech: {e}")

    def transcribe_audio(self) -> Optional[str]:
        """Converts audio to text using OpenAI's Whisper API."""
        recognizer = sr.Recognizer()
        
        try:
            with sr.Microphone() as source:
                self.logger.info("Listening for audio input...")
                audio_data = recognizer.listen(source, timeout=10)
            
            # Salva o áudio em um arquivo temporário (formato WAV)
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_file:
                tmp_file.write(audio_data.get_wav_data())
                tmp_file_path = tmp_file.name

            # Envia o áudio para a API Whisper
            with open(tmp_file_path, "rb") as audio_file:
                self.logger.info("Sending audio to Whisper API...")
                response = openai.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file
                )

            # Extrai o texto transcrito
            transcription = response.text
            self.logger.info(f"Transcription: {transcription}")

            # Remove o arquivo temporário
            os.remove(tmp_file_path)

            return transcription

        except sr.WaitTimeoutError:
            self.logger.error("Microphone timed out while listening.")
            self.speak("Desculpe, não ouvi sua resposta. Pode repetir?")
            return None

        except Exception as e:
            self.logger.error(f"Error during transcription: {e}")
            self.speak("Desculpe, ocorreu um erro durante a transcrição.")
            return None

    def analyze_text(self, text: str):
        """Analyzes text for named entities."""
        doc = self.nlp(text)
        entities = [(ent.text, ent.label_) for ent in doc.ents]
        return entities

    async def generate_and_save_question(self, context: str, patient_id: str) -> QuestionSchema:
        """Generates a new question using the OpenAI API and saves it."""
        input_prompt = f"{context}\nQuais perguntas devo fazer a seguir?"

        try:
            response = openai.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": "Você é um assistente útil para conduzir anamneses médicas."},
                    {"role": "user", "content": input_prompt}
                ]
            )

            question_text = response.choices[0].message.content.strip()

            if not question_text or question_text == context:
                question_text = "Obrigado, isso é tudo por agora."

            question_data = QuestionCreate(text=question_text, patient_id=patient_id)
            created_question = await self.question_service.create_question(question_data)
            self.logger.info(f"New question generated and saved: {created_question.text}")
            return created_question

        except Exception as e:
            self.logger.error(f"Error calling OpenAI API: {e}")
            return QuestionSchema(id="", text="Erro ao gerar pergunta.")

    async def run_anamnese(self, patient_id: str):
        """Runs an interactive anamnese session with a patient."""
        self.speak("Olá, eu sou a Cora. Vou fazer algumas perguntas sobre sua saúde. Responda com sinceridade, por favor.")
        context = ""
        is_running = True  # Flag para controlar o loop

        try:
            questions = await self.question_service.get_question_by_patient_id(patient_id)
            question_index = 0

            while is_running:
                # Escolhe pergunta da lista ou gera uma nova
                if question_index < len(questions):
                    current_question = questions[question_index]
                    question_index += 1
                else:
                    self.logger.info("Generating a new question...")
                    current_question = await self.generate_and_save_question(context, patient_id)

                    # Se falhar ao gerar pergunta, encerra a sessão
                    if not current_question or not current_question.text.strip():
                        self.logger.error("Failed to generate a new question.")
                        self.speak("Desculpe, houve um erro ao gerar uma nova pergunta.")
                        break

                self.logger.info(f"Asking question: {current_question.text}")
                self.speak(current_question.text)

                # Captura a resposta do usuário
                user_input = self.transcribe_audio()
                if not user_input or not user_input.strip():
                    self.speak("Desculpe, não entendi. Poderia repetir, por favor?")
                    continue

                self.logger.info(f"User input: {user_input}")

                # Verifica se há intenção de encerrar
                if self.detect_stop_intent(user_input):
                    self.speak("Conversa encerrada. Obrigado!")
                    is_running = False
                    break

                # Salva a resposta no banco de dados
                answer_data = AnswerCreate(
                    id_patient=patient_id,
                    id_question=current_question.id,
                    answer=user_input
                )
                try:
                    await self.answer_service.create_answer(answer_data)
                    self.logger.info("Answer saved successfully.")
                except Exception as e:
                    self.logger.error(f"Error saving answer: {e}")
                    self.speak("Houve um erro ao salvar sua resposta. Vamos continuar.")

                # Atualiza o contexto com a resposta do usuário
                context += f" {user_input}"

                # Gera nova pergunta baseada no contexto
                if question_index >= len(questions):
                    self.logger.info("Generating follow-up question based on context.")
                    next_question = await self.generate_and_save_question(context, patient_id)

                    if not next_question or not next_question.text.strip():
                        self.logger.error("Failed to generate a follow-up question.")
                        self.speak("Desculpe, houve um erro ao gerar uma nova pergunta.")
                        break

                    # Adiciona nova pergunta à lista, mas só fala uma vez
                    questions.append(next_question)

        except Exception as e:
            self.logger.error(f"Error in anamnese session: {e}")
            self.speak("Houve um erro na sessão de anamnese. Encerrando.")