import React, { useState } from "react";
import SearchBar from "./SearchBar";
import styles from "./header.module.css";
import PatientModal from "../patientModal/patientModal";

interface HeaderProps {
    searchTerm: string;
    onSearchTermChange: (term: string) => void;
    onSearch: () => void;
}

const Header: React.FC<HeaderProps> = ({ searchTerm, onSearchTermChange, onSearch }) => {

    return (
        <div className={styles.header}>
            <h1 className={styles.title}>Pacientes Registrados</h1>
            <div className={styles.searchContainer}>
                <SearchBar
                    searchTerm={searchTerm}
                    onSearchTermChange={onSearchTermChange}
                    onSearch={onSearch}
                />
            </div>
        </div>
    );
};

export default Header;
