"use client";

import { useSearchParams } from "next/navigation";
import React, { useEffect, useState } from "react";
import styles from "./patientInfo.module.css";
import PatientDetailsModal from "@/components/PatientDetailsModal/PatientDetailsModal";
import { getUserRoleFromToken  } from "@/components/utils/auth";
import AuthGuard from "@/components/authGuard/authGuard";

const PatientInfo: React.FC = () => {
  const searchParams = useSearchParams();
  const patient_id = searchParams.get("patient_id"); // ID do paciente
  const [patientData, setPatientData] = useState<any>(null);
  const [questions, setQuestions] = useState<any[]>([]); // Lista de perguntas e respostas
  const [newQuestion, setNewQuestion] = useState<string>(""); // Nova pergunta
  const [editingQuestion, setEditingQuestion] = useState<string | null>(null); // ID da pergunta em edição
  const [isModalOpen, setIsModalOpen] = useState(false); // Modal de adicionar paciente
  const [redirectPath, setRedirectPath] = useState<string>("/login");

  const openModal = () => setIsModalOpen(true);
  const closeModal = () => setIsModalOpen(false);

  // Fetch patient data
  useEffect(() => {
    if (patient_id) {
      fetch(`http://localhost:8000/api/patients/${patient_id}`)
        .then((res) => res.json())
        .then((data) => setPatientData(data))
        .catch((err) => console.error("Erro ao buscar dados do paciente:", err));
    }
  }, [patient_id]);

  // Fetch questions and associate answers
  useEffect(() => {
    if (patient_id) {
      fetch(`http://localhost:8000/api/question/by-patient/${patient_id}`)
        .then((res) => res.json())
        .then((data) => {
          const questionsWithAnswers = data.questions.map((q: any) => ({
            ...q,
            answer: null, // Initialize without answer
          }));
          setQuestions(questionsWithAnswers);

          // Fetch answers for the questions
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
            });
        })
        .catch((err) => console.error("Erro ao buscar perguntas e respostas:", err));
    }
  }, [patient_id]);

  useEffect(() => {
    const role = getUserRoleFromToken();
    if (role == "nurse") {
      setRedirectPath("/nurseHome");
    }
    else if (role == "doctor") {
        setRedirectPath("/doctorHome");
    }
  }, []);

  // Add a new question
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
        setNewQuestion(""); // Clear the input
      })
      .catch((err) => console.error("Erro ao adicionar pergunta:", err));
  };

  // Enable editing for a question
  const enableEditing = (questionId: string) => {
    setEditingQuestion(questionId);
  };

  // Save edited question
  const saveEditedQuestion = (questionId: string, newText: string) => {
    const questionData = {
      text: newText,
      patient_id, // Inclui o ID do paciente como parte do payload
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
  

  // Render the page
  if (!patientData) {
    return <div className={styles.screen}>Carregando informações...</div>;
  }

  return (
    <AuthGuard allowedRoles={["doctor", "nurse"]}>
    <div className={styles.screen}>
      <div className={styles.header}>
        <h1>Informações do Paciente</h1>
        <button
          className={styles.button}
          onClick={() => (window.location.href = redirectPath)}
        >
          <p>Todos os Pacientes</p>
        </button>
      </div>
  
      <div className={styles.patientHeader}>
        <div className={styles.PhotoAndName}>
          <img
            src="/FakePeople/jose.jpg"
            alt={`Foto de ${patientData.name}`}
            className={styles.photo}
          />
          <div className={styles.nameAndRobot}>
            <div className={styles.nameAndId}>
              <h2 className={styles.patientName}>{patientData.name}</h2>
              <h2 className={styles.patientID}>#{patientData.id}</h2>
            </div>
            <p className={styles.robot}>
              Profissional Responsável: {patientData.professional_id || "N/A"}
            </p>
          </div>
        </div>
        {/* Add the button to open modal */}
        <button className={styles.DarkButton} onClick={openModal}>
          <p>Ficha do Paciente</p>
        </button>
      </div>
  
      {/* Patient Modal */}
      {isModalOpen && (
        <PatientDetailsModal
          patientData={patientData} // Pass patient data as props
          onClose={closeModal}
        />
      )}
  
      {/* Questions Section */}
      <div className={styles.content}>
        <section className={styles.questions}>
          <div className={styles.TitleAndButton}>
            <h1 className={styles.subtitle}>Perguntas</h1>
            <div className={styles.buttons}>
              <button className={styles.DarkButton} onClick={handleAddQuestion}>
                <p>Nova Pergunta</p>
              </button>
            </div>
          </div>
          <input
            type="text"
            value={newQuestion}
            onChange={(e) => setNewQuestion(e.target.value)}
            placeholder="Digite sua pergunta aqui..."
            className={styles.questionInput}
          />
          <div className={styles.questionsList}>
            {questions.map((question) => (
              <div key={question.id} className={styles.question}>
                <div className={styles.questionRow}>
                  {editingQuestion === question.id && !question.answer ? (
                    <input
                      type="text"
                      defaultValue={question.text}
                      onBlur={(e) =>
                        saveEditedQuestion(question.id, e.target.value)
                      }
                      autoFocus
                      className={styles.questionInput}
                    />
                  ) : (
                    <>
                      <p className={styles.awnserText}>{question.text}</p>
                      {!question.answer && (
                        <span
                          className={styles.editIcon}
                          onClick={() => enableEditing(question.id)}
                        >
                          ✏️
                        </span>
                      )}
                    </>
                  )}
                </div>
                <p className={styles.awnserText}>
                  Resposta: {question.answer?.answer || "Sem resposta"}
                </p>
                <p className={styles.awnserTextData}>
                  {question.answer?.date_hour
                    ? new Date(question.answer.date_hour).toLocaleString()
                    : "Sem data"}
                </p>
              </div>
            ))}
          </div>
        </section>
      </div>
    </div>
  </AuthGuard>
  );
};

export default PatientInfo;
