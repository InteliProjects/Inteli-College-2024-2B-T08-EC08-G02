import React, { useState } from "react";
import styles from "./RobotCard.module.css";
import Modal from "../robotPatientModal/Modal";

interface RobotCardProps {
  robotId: string; // new prop
  robot: string;
  local: string;
  status: string;
}

const RobotCard: React.FC<RobotCardProps> = ({ robotId, robot, local, status }) => {
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleOpenModal = () => setIsModalOpen(true);
  const handleCloseModal = () => setIsModalOpen(false);

  return (
    <div className={styles.container}>
      <div className={styles.icon}></div>
      <div className={styles.details}>
        <div className={styles.detailItem}>
          <span className={styles.textBold}>{robot}</span>
        </div>
        <div className={styles.detailItem}>
          <span className={styles.textBold}>Local:</span>
          <span className={styles.textNormal}>{local}</span>
        </div>
        <div className={styles.detailItem}>
          <span className={styles.textBold}>Status:</span>
          <span className={styles.textNormal}>{status}</span>
        </div>
      </div>
      <div className={styles.buttonContainer}>
        <div className={styles.button} onClick={handleOpenModal}>
          <div className={styles.buttonText}>Destinar Pacientes</div>
        </div>
      </div>

      {/* Modal Component */}
      <Modal isOpen={isModalOpen} onClose={handleCloseModal} robotId={robotId} />
    </div>
  );
};

export default RobotCard;
