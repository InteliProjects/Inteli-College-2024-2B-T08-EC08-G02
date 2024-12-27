"use client";
import React, { useState } from "react";
import { useRouter } from "next/navigation";
import styles from "./login.module.css";

const Login: React.FC = () => {
  const router = useRouter();
  const [login, setLogin] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();

    try {
      const loginResponse = await fetch("http://localhost:8000/api/auth/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ name: login, password }),
      });

      if (loginResponse.ok) {
        const loginData = await loginResponse.json();
        localStorage.setItem("token", loginData.access_token);

        const roleResponse = await fetch("http://localhost:8000/api/auth/protected_route", {
          method: "POST",
          headers: {
            Authorization: `Bearer ${loginData.access_token}`,
            "Content-Type": "application/json",
          },
        });

        if (roleResponse.ok) {
          const roleData = await roleResponse.json();
          const userRole = roleData.user.role;

          if (userRole === "doctor") {
            router.push("/doctorHome");
          } else if (userRole === "nurse") {
            router.push("/nurseHome");
          } else {
            setError("User role is not recognized.");
          }
        } else {
          setError("Failed to fetch user role.");
        }
      } else {
        const errorData = await loginResponse.json();
        setError(errorData.detail || "Login failed.");
      }
    } catch (error) {
      setError("Something went wrong, please try again later.");
      console.error("Error:", error);
    }
  };

  return (
    <div className={styles.screen}>
      <div className={styles.loginCard}>
        <div className={styles.logoAndDescription}>
          <img src="/cora-logo.png" alt="Cora Logo" className={styles.logo} />
          <p className={styles.description}>
            O suporte automatizado que você precisa.
          </p>
        </div>
        <form className={styles.form} onSubmit={handleSubmit}>
          <div className={styles.inputs}>
            <input
              type="text"
              name="login"
              placeholder="Login"
              className={styles.input}
              value={login}
              onChange={(e) => setLogin(e.target.value)}
            />
            <input
              type="password"
              name="password"
              placeholder="Password"
              className={styles.input}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
            />
          </div>
          <button type="submit" className={styles.button}>
            Enter
          </button>
        </form>
        {error && <p className={styles.error}>{error}</p>}
      </div>
    </div>
  );
};

export default Login;
