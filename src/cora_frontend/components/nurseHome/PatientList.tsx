import React from "react";
import PatientCard from "../../components/patientsCard/patientCard";
import styles from "./patientList.module.css";

interface PatientListProps {
    patients: any[];
    onDeletePatient: (patientId: string) => void;
}

const PatientList: React.FC<PatientListProps> = ({ patients, onDeletePatient }) => (
    <div className={styles.patients}>
        {patients.map((patient) => (
            <PatientCard
                key={patient.id}
                title={patient.name}
                description={`Idade: ${
                    new Date().getFullYear() - new Date(patient.date_of_birth).getFullYear()
                } anos ${patient.current_condition ? `Condição: ${patient.current_condition}` : ""}`}
                imageUrl="/FakePeople/jose.jpg"
                notification={false}
                patientId={patient.id} // Pass patientId here
                // onDelete={() => onDeletePatient(patient.id)}
            />
        ))}
    </div>
);

export default PatientList;
