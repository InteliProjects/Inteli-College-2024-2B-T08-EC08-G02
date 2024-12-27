import speech_recognition as sr
import pyttsx3
import openai
import logging
import spacy
from dotenv import load_dotenv
import os

logging.basicConfig(level=logging.INFO)

# Carregar variáveis de ambiente do arquivo .env
load_dotenv()

# Configurar chave da API OpenAI
openai.api_key = os.getenv("OPENAI_API_KEY")
if not openai.api_key:
    logging.error("Chave da API OpenAI não encontrada. Verifique o arquivo .env.")
    exit(1)

# Configuração do sintetizador de voz
engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.9)

logging.info("Carregando modelo de NER...")
nlp = spacy.load("pt_core_news_sm")

def speak(text):
    engine.say(text)
    engine.runAndWait()

def transcribe_audio():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        logging.info("Aguardando fala do usuário...")
        try:
            audio_data = recognizer.listen(source, timeout=10)
            text = recognizer.recognize_google(audio_data, language="pt-BR")
            logging.info(f"Você disse: {text}")
            return text.lower()
        except sr.UnknownValueError:
            logging.warning("Não consegui entender o áudio.")
            speak("Desculpe, não consegui entender. Poderia repetir?")
            return None
        except sr.RequestError as e:
            logging.error(f"Erro no serviço de reconhecimento de fala: {e}")
            return None

def analyze_text(text):
    doc = nlp(text)
    entities = [(ent.text, ent.label_) for ent in doc.ents]
    return entities

def generate_question(context):
    prompt = f"""
   Você é um médico virtual conduzindo uma anamnese, faça apenas uma pergunta por vez. Baseado no seguinte contexto da conversa:
    {context}

    Faça uma pergunta por vez, em relação aos seguintes tópicos sobre a saúde do paciente:
    - Se o paciente sente dor
    - Localização de dores
    - Intensidade de dores
    - Duração de dores
    - Febre
    - Dificuldade de respirar ou outros sintomas respiratórios
    - Saúde mental (ansiedade, estresse, tristeza)
    - Falta de energia
    - Alterações no sono ou disposição
    - Alterações intestinais ou digestivas
    - Queixas adicionais do paciente

    Se o paciente já mencionou algum dos tópicos acima, elabore perguntas para aprofundar ou explorar outras áreas.
    """

    try:
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "Você é um assistente médico especializado em anamnese."},
                {"role": "user", "content": prompt}
            ]
        )
        question = response.choices[0].message['content'].strip()
        return question
    except Exception as e:
        logging.error(f"Erro ao gerar pergunta: {e}")
        return "Houve um problema ao gerar a próxima pergunta. Podemos tentar novamente?"

# Função principal da anamnese
def chatbot_anamnese():
    speak("Olá, eu sou a Cora. Vou fazer algumas perguntas sobre sua saúde. Por favor, responda com sinceridade, entendido?")
    user_input = transcribe_audio()

    if not user_input:
        speak("Não consegui entender. Vamos começar?")
        user_input = transcribe_audio()

    if any(word in user_input for word in ["sim", "entendido", "positivo", "ok"]):
        context = ""
        while True:
            question = generate_question(context)
            speak(question)

            user_input = transcribe_audio()
            if not user_input:
                continue

            context += f" {user_input}"

            if "encerrar" in user_input or "parar" in user_input:
                speak("Conversa encerrada. Obrigado!")
                break

            entities = analyze_text(user_input)
            logging.info(f"Entidades reconhecidas: {entities}")

    elif any(word in user_input for word in ["não", "negativo", "não quero"]):
        speak("Irei então solicitar que um profissional venha até você.")
    else:
        speak("Desculpe, não entendi. Vou encerrar por aqui. Obrigado!")

if __name__ == "__main__":
    logging.info("Iniciando chatbot de anamnese...")
    try:
        chatbot_anamnese()
    except Exception as e:
        logging.error(f"Erro durante a execução: {e}")
