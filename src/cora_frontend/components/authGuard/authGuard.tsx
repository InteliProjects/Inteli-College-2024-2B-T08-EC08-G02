"use client";

import React, { useEffect, useState } from "react";
import { useRouter } from "next/navigation";

interface AuthGuardProps {
  children: React.ReactNode;
  allowedRoles: string[]; // Papéis permitidos para acessar a rota
}

const AuthGuard: React.FC<AuthGuardProps> = ({ children, allowedRoles }) => {
  const router = useRouter();
  const [isAuthorized, setIsAuthorized] = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const token = localStorage.getItem("token");

    if (!token) {
      router.push("/login"); // Redireciona para o login se o token não existir
      return;
    }

    const validateToken = async () => {
      try {
        const response = await fetch("http://localhost:8000/api/auth/protected_route", {
          method: "POST",
          headers: {
            Authorization: `Bearer ${token}`,
            "Content-Type": "application/json",
          },
        });

        if (response.ok) {
          const data = await response.json();
          const userRole = data.user.role; // Obtém o papel do usuário retornado pelo backend

          if (allowedRoles.includes(userRole)) {
            console.log("User is authorized");
            setIsAuthorized(true); // Usuário autorizado
          } else {
            router.push("/login"); // Redireciona para rota de acesso negado
          }
        } else {
          localStorage.removeItem("token"); // Remove token inválido
          router.push("/login"); // Redireciona para o login
        }
      } catch (error) {
        console.error("Erro ao validar token:", error);
        localStorage.removeItem("token"); // Remove token inválido em caso de erro
        router.push("/login"); // Redireciona para o login
      } finally {
        setLoading(false); // Define o estado de carregamento como falso
      }
    };

    validateToken();
  }, [allowedRoles, router]);

  if (loading) {
    return <p>Carregando...</p>; // Exibe uma mensagem ou spinner enquanto carrega
  }

  if (!isAuthorized) {
    return null; // Evita renderizar o conteúdo até que o usuário seja autorizado
  }

  return <>{children}</>; // Renderiza o conteúdo protegido
};

export default AuthGuard;
