"use client";

import React, { useEffect, useState } from "react";
import styles from "./doctorHome.module.css";
import Modal from "../../components/newPatientModal/newPatient";
import AuthGuard from "@/components/authGuard/authGuard";
import { getPatients, searchPatientByName, attrelatePatient, deattrelatePatient } from "@/app/services/apiService";
import PatientSection from "@/components/nurseHome/PatientSection";

const DoctorHome: React.FC = () => {
    const [patients, setPatients] = useState<any[]>([]);
    const [searchTerm, setSearchTerm] = useState("");
    const [isModalOpen, setIsModalOpen] = useState(false);
    const [newPatientId, setNewPatientId] = useState("");
    const [noPatients, setNoPatients] = useState(false);

    useEffect(() => {
        const fetchPatients = async () => {
            try {
                const data = await getPatients();
                if (data?.patients?.length) {
                    setPatients(data.patients);
                    setNoPatients(false);
                } else {
                    setNoPatients(true);
                }
            } catch (error: any) {
                if (error.response?.status === 404) {
                    setNoPatients(true);
                } else {
                    console.error("Error fetching patients:", error);
                }
            }
        };

        fetchPatients();
    }, []);

    const handleSearch = async () => {
        try {
            const patient = await searchPatientByName(searchTerm);
            if (patient) {
                setPatients([patient]);
                setNoPatients(false);
            } else {
                setNoPatients(true);
            }
        } catch (error) {
            console.error("Error searching patient:", error);
            setNoPatients(true);
        }
    };

    const handleAttrelate = async (patientID: string) => {
        if (!patientID) {
            console.error("Patient ID is required to attrelate.");
            return;
        }

        try {
            await attrelatePatient(patientID);
            setNewPatientId("");
            setIsModalOpen(false);
            const data = await getPatients();
            setPatients(data.patients || []);
        } catch (error) {
            console.error("Error associating patient:", error);
        }
    };

    const handleDeattrelate = async (patientID: string) => {
        try {
            await deattrelatePatient(patientID);
            const data = await getPatients();
            setPatients(data.patients || []);
        } catch (error) {
            console.error("Error disassociating patient:", error);
        }
    };

    const openModal = () => setIsModalOpen(true);
    const closeModal = () => setIsModalOpen(false);

    return (
       <AuthGuard allowedRoles={["doctor"]}>
            <div className={styles.screen}>
                <div className={styles.header}>
                    <h1 className="Title">Pacientes Registrados</h1>
                    <input
                        type="text"
                        name="patientSearch"
                        placeholder="Procurar por nome de pacientes"
                        className={styles.input}
                        value={searchTerm}
                        onChange={(e) => setSearchTerm(e.target.value)}
                    />
                    <button className={styles.button} onClick={openModal}>
                        <p className={styles.buttonText}>Adicionar Paciente</p>
                    </button>
                </div>

                <div className={styles.content}>
                    {noPatients ? (
                        <p className={styles.emptyMessage}>Nenhum paciente encontrado.</p>
                    ) : (
                        <PatientSection
                            patients={patients}
                            onNewPatient={openModal}
                            onDeletePatient={handleDeattrelate}
                        />
                    )}
                </div>

                {isModalOpen && (
                    <Modal
                        isOpen={isModalOpen}
                        onClose={closeModal}
                        title="Registrar novo paciente"
                        description="Insira a identificação do paciente para atribuí-lo aos seus cuidados."
                        label="Identificação do Paciente:"
                        inputValue={newPatientId}
                        onInputChange={(value) => setNewPatientId(value)}
                    >
                        <div style={{ display: "flex", justifyContent: "space-between", marginTop: "20px" }}>
                            <button className={styles.button} onClick={closeModal}>
                                Cancelar
                            </button>
                            <button className={styles.button} onClick={() => handleAttrelate(newPatientId)}>
                                Registrar Paciente
                            </button>
                        </div>
                    </Modal>
                )}
            </div>
       </AuthGuard> 
    );
};

export default DoctorHome;
