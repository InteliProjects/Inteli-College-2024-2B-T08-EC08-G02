import React, { useEffect, useState } from "react";
import Header from "./Header";
import PatientSection from "./PatientSection";
import PatientModal from "../patientModal/patientModal";
import AuthGuard from "@/components/authGuard/authGuard";
import { getAllPatients, searchPatientByName } from "@/app/services/apiService";
import styles from "./nurseHome.module.css";

const NurseHome: React.FC = () => {
  const [patients, setPatients] = useState<any[]>([]);
  const [searchTerm, setSearchTerm] = useState("");
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [selectedPatient, setSelectedPatient] = useState<any>(null);

  useEffect(() => {
    const fetchPatients = async () => {
      try {
        const data = await getAllPatients();
        setPatients(data.patients);
      } catch (error) {
        console.error("Error fetching patients:", error);
      }
    };

    fetchPatients();
  }, []);

  const handleSearch = async () => {
    try {
      const patient = await searchPatientByName(searchTerm);
      setPatients([patient]);
    } catch (error) {
      console.error("Error searching patient:", error);
    }
  };


  const handleCloseModal = () => {
    setIsModalOpen(false);
    setSelectedPatient(null);
  };

  return (
    <AuthGuard allowedRoles={["nurse"]}>
      <div className={styles.screen}>
        <Header
          searchTerm={searchTerm}
          onSearchTermChange={setSearchTerm}
          onSearch={handleSearch}
        />
        <div className={styles.content}>
          <PatientSection
            patients={patients}
            showModalButton={true}
          />
        </div>
        {isModalOpen && (
          <PatientModal
            onClose={handleCloseModal}
            onPatientCreated={(newPatient) => {
              setPatients((prev) => [...prev, newPatient]);
            }}
          />
        )}
      </div>
    </AuthGuard>
  );
};

export default NurseHome;
