import React from "react";
import "./patientDetails.css";

interface PatientDetailsModalProps {
  onClose: () => void;
  patientData: {
    name: string;
    date_of_birth: string;
    sex: string;
    id_room: number | null;
    current_condition: string;
    professional_id: string | null;
  };
}

const PatientDetailsModal: React.FC<PatientDetailsModalProps> = ({ onClose, patientData }) => {
  return (
    <>
      <div className="modal-overlay" onClick={onClose}></div>
      <div className="patient-modal">
        <header className="modal-header">
          <h1>Ficha do Paciente</h1>
          <button className="close-button" onClick={onClose}>
            &times;
          </button>
        </header>
        <hr className="divider" />
        <div className="patient-info">
        <div className="patient-header">
            <div className="avatar-placeholder">
                <img src="/FakePeople/jose.jpg" alt="Patient" />
            </div>
            <div className="patient-info-text">
                <div className="patient-name">{patientData.name}</div>
                <div className="patient-id">#{patientData.professional_id}</div>
            </div>
            </div>


          <section className="personal-info">
            <h3>Informações Pessoais:</h3>
            <div className="info-grid">
              <div className="info-item">
                <label>Data de Nascimento:</label>
                <p>{new Date(patientData.date_of_birth).toLocaleDateString()}</p>
              </div>
              <div className="info-item">
                <label>Sexo:</label>
                <p>{patientData.sex || "Não informado"}</p>
              </div>
              <div className="info-item">
                <label>ID do Quarto:</label>
                <p>{patientData.id_room !== null ? patientData.id_room : "Não designado"}</p>
              </div>
              <div className="info-item">
                <label>Condição Atual:</label>
                <p>{patientData.current_condition || "Não especificada"}</p>
              </div>
            </div>
          </section>
        </div>
      </div>
    </>
  );
};

export default PatientDetailsModal;
