"use client";

import React, { useEffect, useState } from "react";
import RobotCard from "../../components/RobotCard/RobotCard";
import styles from "./robots_nurse.module.css";
import AuthGuard from "@/components/authGuard/authGuard";
import { getUserRoleFromToken  } from "@/components/utils/auth";

interface Robot {
  id: string;
  name: string;
  current_location: string;
  status: string;
}

const RobotsPage: React.FC = () => {
  const [robots, setRobots] = useState<Robot[]>([]);
  const [filteredRobots, setFilteredRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState<string>("");
  const [redirectPath, setRedirectPath] = useState<string>("/login");

  useEffect(() => {
    const fetchRobots = async () => {
      try {
        const response = await fetch("http://localhost:8000/api/robots", {
          headers: { "Content-Type": "application/json" },
        });

        if (!response.ok) {
          throw new Error("Failed to fetch robots");
        }

        const data = await response.json();
        setRobots(data.robots); // Assuming the API response is { "robots": [ ... ] }
        setFilteredRobots(data.robots);
      } catch (err) {
        setError(err.message);
      } finally {
        setLoading(false);
      }
    };

    fetchRobots();
  }, []);

  useEffect(() => {
      const role = getUserRoleFromToken();
      if (role == "nurse") {
        setRedirectPath("/nurseHome");
      }
      else if (role == "doctor") {
          setRedirectPath("/doctorHome");
      }
    }, []);

  const handleSearch = (event: React.ChangeEvent<HTMLInputElement>) => {
    const query = event.target.value.toLowerCase();
    setSearchQuery(query);
    setFilteredRobots(
      robots.filter(
        (robot) =>
          robot.name.toLowerCase().includes(query) ||
          robot.current_location.toLowerCase().includes(query) ||
          robot.status.toLowerCase().includes(query)
      )
    );
  };

  if (loading) {
    return <div className={styles.screen}>Carregando robôs...</div>;
  }

  if (error) {
    return <div className={styles.screen}>Erro ao carregar robôs: {error}</div>;
  }

  return (
    <AuthGuard allowedRoles={["doctor", "nurse"]}>
    <div className={styles.screen}>
      <header className={styles.header}>
      <div className={styles.headerTitle}>Robôs Registrados</div>
      <div className={styles.headerActions}>
        <div className={styles.searchContainer}>
          <input
            type="text"
            placeholder="Pesquisar robôs..."
            className={styles.searchBar}
            value={searchQuery}
            onChange={handleSearch}
          />
          <div className={styles.searchIcon}>
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="16" height="16" fill="#0A3975">
              <path d="M10 2a8 8 0 105.293 14.293l5.707 5.707-1.414 1.414-5.707-5.707A8 8 0 1010 2zm0 2a6 6 0 110 12 6 6 0 010-12z"></path>
            </svg>
          </div>
        </div>
      </div>
      <button className={styles.patientsButton} onClick={() => (window.location.href = redirectPath)}>Todos os Pacientes</button>
    </header>



      {/* Section Content */}
      <div className={styles.content}>
        <section className={styles.titleSection}>
          <div className={styles.title}>Robots</div>
        </section>

        {/* Robots List */}
        <section className={styles.robotsContainer}>
        {filteredRobots.map((robot) => (
          <RobotCard
            key={robot.id}
            robotId={robot.id}
            robot={robot.name}
            local={robot.current_location || "Localização desconhecida"}
            status={robot.status}
          />
        ))}

        </section>
      </div>
    </div>
    </AuthGuard>
  );
};

export default RobotsPage;