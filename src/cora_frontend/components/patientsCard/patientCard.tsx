"use client";

import React from "react";
import { useRouter } from "next/navigation";
import styles from "./patientCard.module.css";

interface PatientCardProps {
  title: string;
  description: string;
  imageUrl: string;
  patientId: string; // Add patientId to the props
  notification?: boolean;
  onDelete?: () => void; // Callback to delete the patient
}

const PatientCard: React.FC<PatientCardProps> = ({
  title,
  description,
  imageUrl,
  patientId, // Use patientId
  notification = false,
}) => {
  const router = useRouter();

  const handleClick = () => {
    router.push(`/patientInfo?patient_id=${patientId}`); // Pass patient_id as a query parameter
  };
  
  return (
    <div className={styles.cardContainer}>
      <button className={styles.card} onClick={handleClick}>
        {imageUrl && <img src={imageUrl} alt={title} className={styles.image} />}
        <div className={styles.content}>
          <h2 className={styles.title}>{title}</h2>
          <p className={styles.description}>{description}</p>
        </div>
        {notification && <div className={styles.notification}>🔔</div>}
      </button>
    </div>
  );
};

export default PatientCard;
