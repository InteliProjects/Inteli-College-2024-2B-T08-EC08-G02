---
title: Integração da tela de paciente
sidebar_position: 2
---

## **1.0** Introdução

Este documento apresenta o funcionamento da integração da tela de informações do paciente com os endpoints da API. Ele detalha os endpoints utilizados, os dados de entrada e saída (JSONs), e como eles foram integrados ao sistema.

## **2.0** Funcionamento dos Endpoints

A seguir, apresentamos os endpoints, suas finalidades, exemplos de uso e os JSONs envolvidos.

### **2.1** Busca de informações do paciente

#### Endpoint:
- **`GET /api/patients/:id`**

#### Finalidade:
Retorna informações detalhadas do paciente com base no seu ID.

#### Exemplo de Requisição:
```http
GET http://localhost:8000/api/patients/123
```

#### Exemplo de Resposta:
```json
{
  "id": 123,
  "name": "José Silva",
  "age": 65,
  "gender": "Masculino",
  "height": 170,
  "weight": 75,
  "professional_id": "456"
}
```

#### Integração no Código:
```tsx
useEffect(() => {
  if (patient_id) {
    fetch(`http://localhost:8000/api/patients/${patient_id}`)
      .then((res) => res.json())
      .then((data) => setPatientData(data))
      .catch((err) => console.error("Erro ao buscar dados do paciente:", err));
  }
}, [patient_id]);
```

Logo, o código acima é responsável por buscar as informações do paciente com base no ID fornecido. O retorno é armazenado no estado `patientData`, que é utilizado para exibir as informações na tela.

### **2.2** Busca de perguntas associadas ao paciente

#### Endpoint:
- **`GET /api/question/by-patient/:id`**

#### Finalidade:
Retorna uma lista de perguntas relacionadas ao paciente.

#### Exemplo de Requisição:
```http
GET http://localhost:8000/api/question/by-patient/123
```

#### Exemplo de Resposta:
```json
{
  "questions": [
    {
      "id": "q1",
      "text": "Como está sua dor hoje?",
      "date_created": "2024-12-01T10:00:00Z"
    },
    {
      "id": "q2",
      "text": "Você tomou a medicação prescrita?",
      "date_created": "2024-12-02T14:00:00Z"
    }
  ]
}
```

#### Integração no Código:
```tsx
useEffect(() => {
  if (patient_id) {
    fetch(`http://localhost:8000/api/question/by-patient/${patient_id}`)
      .then((res) => res.json())
      .then((data) => {
        const questionsWithAnswers = data.questions.map((q: any) => ({
          ...q,
          answer: null, // Inicializa sem resposta
        }));
        setQuestions(questionsWithAnswers);
      })
      .catch((err) => console.error("Erro ao buscar perguntas:", err));
  }
}, [patient_id]);
```

Logo, o código acima é responsável por buscar as perguntas relacionadas ao paciente com base no ID fornecido. O retorno é armazenado no estado `questions`, que é utilizado para exibir as perguntas na tela.

### **2.3** Busca de respostas associadas ao paciente

#### Endpoint:
- **`GET /api/answer/patient/:id`**

#### Finalidade:
Retorna respostas para as perguntas do paciente.

#### Exemplo de Requisição:
```http
GET http://localhost:8000/api/answer/patient/123
```

#### Exemplo de Resposta:
```json
{
  "answers": [
    {
      "id_question": "q1",
      "answer": "A dor está moderada.",
      "date_hour": "2024-12-01T10:30:00Z"
    },
    {
      "id_question": "q2",
      "answer": "Sim, tomei todos os medicamentos.",
      "date_hour": "2024-12-02T14:30:00Z"
    }
  ]
}
```

#### Integração no Código:
```tsx
fetch(`http://localhost:8000/api/answer/patient/${patient_id}`)
  .then((res) => res.json())
  .then((data) => {
    const answers = data.answers;
    setQuestions((prev) =>
      prev.map((q) => ({
        ...q,
        answer: answers.find((a: any) => a.id_question === q.id) || null,
      }))
    );
  })
  .catch((err) => console.error("Erro ao buscar respostas:", err));
```

Logo, o código acima é responsável por buscar as respostas associadas ao paciente com base no ID fornecido. As respostas são então vinculadas às perguntas correspondentes, atualizando o estado `questions` com as respostas.

### **2.4** Adição de nova pergunta

#### Endpoint:
- **`POST /api/question`**

#### Finalidade:
Adiciona uma nova pergunta ao paciente.

#### Exemplo de Requisição:
```http
POST http://localhost:8000/api/question
Content-Type: application/json

{
  "text": "Qual é o seu nível de energia hoje?",
  "patient_id": 123
}
```

#### Exemplo de Resposta:
```json
{
  "id": "q3",
  "text": "Qual é o seu nível de energia hoje?",
  "date_created": "2024-12-08T09:00:00Z"
}
```

#### Integração no Código:
```tsx
const handleAddQuestion = () => {
  if (!newQuestion.trim()) return;

  const questionData = {
    text: newQuestion,
    patient_id,
  };

  fetch("http://localhost:8000/api/question", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(questionData),
  })
    .then((res) => res.json())
    .then((newQuestion) => {
      setQuestions((prev) => [...prev, { ...newQuestion, answer: null }]);
      setNewQuestion(""); // Limpa o campo de entrada
    })
    .catch((err) => console.error("Erro ao adicionar pergunta:", err));
};
```

Logo, o código acima é responsável por adicionar uma nova pergunta ao paciente. O texto da pergunta é enviado ao servidor, que retorna a pergunta com um ID e data de criação. A nova pergunta é então adicionada ao estado `questions` para exibição na tela.

### **2.5** Edição de uma pergunta existente

#### Endpoint:
- **`PUT /api/question/:id`**

#### Finalidade:
Atualiza o texto de uma pergunta existente.

#### Exemplo de Requisição:
```http
PUT http://localhost:8000/api/question/q1
Content-Type: application/json

{
  "text": "Como você está se sentindo hoje?",
  "patient_id": 123
}
```

#### Exemplo de Resposta:
```http
Status: 200 OK
```

#### Integração no Código:
```tsx
const saveEditedQuestion = (questionId: string, newText: string) => {
  const questionData = {
    text: newText,
    patient_id, // Inclui o ID do paciente no payload
  };

  fetch(`http://localhost:8000/api/question/${questionId}`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(questionData),
  })
    .then(() => {
      setQuestions((prev) =>
        prev.map((q) => (q.id === questionId ? { ...q, text: newText } : q))
      );
      setEditingQuestion(null); // Sai do modo de edição
    })
    .catch((err) => console.error("Erro ao salvar pergunta editada:", err));
};
```

Logo, o código acima é responsável por atualizar o texto de uma pergunta existente. O novo texto é enviado ao servidor, que atualiza a pergunta correspondente. O estado `questions` é então atualizado com a pergunta editada.

## **3.0** Conclusão

Os endpoints descritos permitem a comunicação entre a tela de informações do paciente e o servidor. A integração foi realizada de forma modular, com destaque para o consumo eficiente de dados e a manipulação sincronizada de estados no front-end. Esses endpoints garantem que as informações exibidas estejam sempre atualizadas e permitem interações como adição e edição de perguntas.

Com a integração dos endpoints, a tela de informações do paciente se torna uma ferramenta eficaz para o monitoramento e acompanhamento dos pacientes, fornecendo dados precisos e atualizados para os profissionais de saúde.