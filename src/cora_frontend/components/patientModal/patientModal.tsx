import React, { useState } from "react";
import { createPatient } from "@/app/services/apiService";
import "./patientmodal.css";

interface PatientModalProps {
  onClose: () => void;
  onPatientCreated: (patient: any) => void; // Callback para atualizar a lista de pacientes
}

const PatientModal: React.FC<PatientModalProps> = ({ onClose, onPatientCreated }) => {
  const [formData, setFormData] = useState({
    name: "",
    date_of_birth: "",
    sex: "",
    id_room: "",
    current_condition: "",
  });

  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState("");

  // Lidar com alterações nos campos de entrada
  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prevData) => ({ ...prevData, [name]: value }));
  };

  // Enviar dados para o backend
  const handleSubmit = async (event?: React.MouseEvent) => {
    event?.preventDefault();
  
    if (isSubmitting) return;
  
    const professionalId = "some-professional-id"; // Substituir com lógica real
    const patientData = {
      ...formData,
      id_room: Number(formData.id_room),
      professional_id: professionalId,
    };
  
    console.log("Payload enviado:", patientData); // Debug para verificar o payload
  
    setIsSubmitting(true);
    try {
      const createdPatient = await createPatient(patientData);
      console.log("Paciente criado com sucesso:", createdPatient); // Debug
      onPatientCreated(createdPatient); // Atualiza a lista no pai
      onClose();
    } catch (err) {
      console.error("Erro ao criar paciente:", err);
      setError("Erro ao criar paciente. Tente novamente.");
    } finally {
      setIsSubmitting(false);
    }
  };  

  return (
    <>
      <div className="modal-overlay" onClick={onClose}></div>
      <div className="patient-modal">
        <header className="modal-header">
          <h1>Registrar Novo Paciente</h1>
          <button className="close-button" onClick={onClose}>
            &times;
          </button>
        </header>
        <p className="modal-description">Preencha os campos abaixo para adicionar um novo paciente.</p>
        <hr className="divider" />
        <div className="patient-info">
          <div className="patient-header">
            <div className="avatar-placeholder">👤</div>
            <div className="patient-name">
              <h2>Dados do Paciente</h2>
            </div>
          </div>
          <section className="personal-info">
            <h3>Informações Pessoais:</h3>
            <div className="info-grid">
              <div className="info-item">
                <label>Nome</label>
                <input
                  type="text"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  className="input-field"
                  placeholder="Nome do Paciente"
                />
              </div>
              <div className="info-item">
                <label>Data de Nascimento</label>
                <input
                  type="date"
                  name="date_of_birth"
                  value={formData.date_of_birth}
                  onChange={handleInputChange}
                  className="input-field"
                />
              </div>
              <div className="info-item">
                <label>Sexo</label>
                <select
                  name="sex"
                  value={formData.sex}
                  onChange={handleInputChange}
                  className="input-field"
                >
                  <option value="">Selecione</option>
                  <option value="M">Masculino</option>
                  <option value="F">Feminino</option>
                </select>
              </div>
              <div className="info-item">
                <label>ID do Quarto</label>
                <input
                  type="number"
                  name="id_room"
                  value={formData.id_room}
                  onChange={handleInputChange}
                  className="input-field"
                  placeholder="Número do Quarto"
                />
              </div>
              <div className="info-item">
                <label>Condição Atual</label>
                <input
                  type="text"
                  name="current_condition"
                  value={formData.current_condition}
                  onChange={handleInputChange}
                  className="input-field"
                  placeholder="Condição Atual"
                />
              </div>
            </div>
          </section>
          {error && <p className="error-message">{error}</p>}
          <section className="clinical-data">
            <h3>Ações:</h3>
            <div className="button-group">
              <button className="delete-button" onClick={onClose}>
                Cancelar
              </button>
              <button className="edit-button" onClick={handleSubmit} disabled={isSubmitting}>
                {isSubmitting ? "Enviando..." : "Registrar Paciente"}
              </button>
            </div>
          </section>
        </div>
      </div>
    </>
  );
};

export default PatientModal;
