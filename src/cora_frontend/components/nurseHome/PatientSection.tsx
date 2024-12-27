import React, { useState } from "react";
import PatientList from "./PatientList";
import PatientModal from "../patientModal/patientModal";
import styles from "./patientSection.module.css";

interface PatientSectionProps {
  patients: any[];
  onNewPatient: (patient: any) => void; // Callback to handle a newly created patient
  showModalButton?: boolean; // Optional button to open the modal
}

const PatientSection: React.FC<PatientSectionProps> = ({ 
    patients, 
    showModalButton = false 
  }) => {
    const [isModalOpen, setIsModalOpen] = useState(false);
  
    const openModal = () => setIsModalOpen(true); // Open the modal
    const closeModal = () => setIsModalOpen(false); // Close the modal
  
    return (
      <div className={styles.patientSection}>
        <div className={styles.TitleAndButton}>
          <h2 className={styles.subTitle}>Todos os Pacientes</h2>
          {showModalButton && (
            <button className={styles.modalButton} onClick={openModal}>
              Adicionar Paciente
            </button>
          )}
        </div>
        <PatientList patients={patients}  />
        
        {isModalOpen && (
          <PatientModal 
            onClose={closeModal} 
            onPatientCreated={(newPatient) => {
              closeModal();
            }}
          />
        )}
      </div>
    );
  };
  

export default PatientSection;
