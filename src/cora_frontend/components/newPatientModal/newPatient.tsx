import React from "react";
import styles from "./newPatient.module.css";

interface ModalProps {
  isOpen: boolean;
  onClose: () => void;
  title: string;
  description: string;
  label?: string;
  inputValue?: string;
  onInputChange?: (value: string) => void;
  children?: React.ReactNode;
}

const Modal: React.FC<ModalProps> = ({ 
  isOpen, 
  onClose, 
  title, 
  description, 
  label, 
  inputValue, 
  onInputChange, 
  children 
}) => {
  if (!isOpen) return null;

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        <h2 className={styles.title}>{title}</h2>
        <p className={styles.description}>{description}</p>
        {label && (
          <div style={{ marginBottom: "20px" }}>
            <label htmlFor="modal-input" style={{ marginBottom: "10px", display: "block",}} className={styles.description}>
              {label}
            </label>
            <input
              type="text"
              id="modal-input"
              value={inputValue}
              onChange={(e) => onInputChange?.(e.target.value)}
              style={{ padding: "10px", width: "100%" }}
            />
          </div>
        )}
        <div className={styles.content}>{children}</div>
        <div className={styles.actions}></div>
      </div>
    </div>
  );
};

export default Modal;
