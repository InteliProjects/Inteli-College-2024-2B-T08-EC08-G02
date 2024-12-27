import React, { useState } from "react";
import styles from "../css/profile.module.css";

export const Membro = ({ nome, imagem, linkedin }) => {
  const [hovered, setHovered] = useState(false);

  const redirecionarParaSite = () => {
    window.open(linkedin, "_blank");
  };

  const containerClassName = `${styles.profile_container} ${hovered ? styles.hovered : ""}`;

  return (
    <div
      className={containerClassName}
      onClick={redirecionarParaSite}
      onMouseEnter={() => setHovered(true)}
      onMouseLeave={() => setHovered(false)}
    >
      <img src={imagem} alt={nome} className={styles.image} />
      <div className={styles.text_container}>
        <p className={styles.name}>{nome}</p>
        <p className={styles.description}>Aluno(a) de Engenharia de computação</p>
      </div>
    </div>
  );
};