import speech_recognition as sr
import pyttsx3
import logging
import spacy
import ollama

response = ollama.chat(model="llama3.2", messages=[{"role": "user", "content": "Seu prompt aqui"}])
print(response)

logging.basicConfig(level=logging.INFO)

engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.9)

# Carregar modelo de NER (spaCy)
logging.info("Carregando modelo de NER...")
nlp = spacy.load("pt_core_news_sm")

# Função para síntese de fala
def speak(text):
    engine.say(text)
    engine.runAndWait()

# Função para reconhecimento de fala
def transcribe_audio():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        logging.info("Aguardando fala do usuário...")
        try:
            audio_data = recognizer.listen(source, timeout=10)
            text = recognizer.recognize_google(audio_data, language="pt-BR")
            logging.info(f"Você disse: {text}")
            return text
        except sr.UnknownValueError:
            logging.warning("Não consegui entender o áudio.")
            speak("Desculpe, não consegui entender. Poderia repetir?")
            return None
        except sr.RequestError as e:
            logging.error(f"Erro no serviço de reconhecimento de fala: {e}")
            return None

# Função para análise de entidades e intenções
def analyze_text(text):
    doc = nlp(text)
    entities = [(ent.text, ent.label_) for ent in doc.ents]
    return entities

# Função para gerar próxima pergunta
def generate_question(context, default_questions):
    # Utilize contexto para adaptar a conversa
    input_prompt = f"{context}\nQuais perguntas devo fazer a seguir?"
    
    # Alterado para usar Llama 3.2
    response = ollama.chat(model="llama-3.2", messages=[{"role": "user", "content": input_prompt}])
    
    question = response['text'].split(":")[-1].strip()

    # Se o modelo não retornar algo válido, pegue a próxima pergunta padrão
    if not question or question == context:
        return default_questions.pop(0) if default_questions else "Obrigado, isso é tudo por agora."
    return question

# Função principal da anamnese
def chatbot_anamnese():
    speak("Olá, eu sou a Cora. Vou fazer algumas perguntas sobre sua saúde. Por favor, responda com sinceridade, entendido?")
    context = ""
    default_questions = [
        "Você está com dor? Se sim, onde?",
        "Qual a intensidade da sua dor em uma escala de 1 a 10?",
        "Você está com alguma dificuldade para respirar?",
        "Como você está emocionalmente? Sente-se ansioso ou triste?",
        "Teve febre recentemente?",
        "Há alguma queixa adicional que gostaria de relatar?"
    ]

    while True:
        user_input = transcribe_audio()
        if not user_input:
            continue

        # Analisar intenções e entidades
        entities = analyze_text(user_input)
        logging.info(f"Entidades reconhecidas: {entities}")

        # Atualizar contexto
        context += f" {user_input}"

        # Checar intenção de encerrar
        if "encerrar" in user_input.lower():
            speak("Conversa encerrada. Obrigado!")
            break

        # Gerar próxima pergunta
        next_question = generate_question(context, default_questions)
        speak(next_question)

# Início do chatbot
if __name__ == "__main__":
    chatbot_anamnese()
