import React from "react";
import styles from "./SearchBar.module.css";

interface SearchBarProps {
  searchTerm: string;
  onSearchTermChange: (term: string) => void;
}

const SearchBar: React.FC<SearchBarProps> = ({ searchTerm, onSearchTermChange }) => (
  <div className={styles.searchBar}>
    {/* Input field for live search */}
    <input
      type="text"
      name="patientSearch"
      placeholder="Procurar por nome de pacientes"
      className={styles.input}
      value={searchTerm}
      onChange={(e) => onSearchTermChange(e.target.value)}
    />

    {/* Button to navigate to robots page */}
    <button className={styles.button} onClick={() => (window.location.href = "/robotsNurse")}>
      <p className={styles.buttonText}>Robôs</p>
    </button>
  </div>
);

export default SearchBar;
