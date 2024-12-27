---
title: Integração da tela de login
sidebar_position: 3
---

# Integração da Tela de Login

## **1.0** Introdução

Este documento apresenta como foi realizada a integração da tela de login ao sistema. A tela de login é a porta de entrada do sistema, sendo responsável por autenticar os usuários e direcioná-los às telas apropriadas com base no seu papel.


## **2.0** Desenvolvimento

A integração utilizou bibliotecas especializadas para autenticação segura e gerenciamento de tokens. As principais tecnologias e bibliotecas empregadas foram:

- **`bcrypt` (versão 3.2.0):** Para hash e validação de senhas.
- **`jsonwebtoken` (versão 8.5.1):** Para geração e validação de tokens JWT.

O sistema realiza a autenticação por meio de um **token JWT**, que é gerado no backend ao validar as credenciais do usuário. Esse token é armazenado no **localStorage** do navegador e usado para autenticar requisições subsequentes.



## **3.0** Funcionamento

A tela de login apresenta dois campos para o usuário inserir **email** e **senha**. Após clicar no botão de login:

1. As credenciais são enviadas ao backend para validação.
2. Caso sejam válidas, o backend retorna um token JWT.
3. O token é armazenado no **localStorage** para autenticação de futuras requisições.
4. O papel do usuário (médico, enfermeiro, etc.) é obtido por meio de uma rota protegida e utilizado para redirecionamento.

:::info
O token JWT tem validade de 30 minutos. Após esse período, o usuário deve fazer login novamente.
:::



## **4.0** Código da Integração

Abaixo está o código do componente de login, destacando os pontos principais de integração com o backend:

### **4.1** Requisição de Login

```tsx
const loginResponse = await fetch("http://localhost:8000/api/auth/login", {
  method: "POST",
  headers: {
    "Content-Type": "application/json",
  },
  body: JSON.stringify({ name: login, password }),
});
```

#### Detalhes:
- **Endpoint:** `POST /api/auth/login`
- **Finalidade:** Valida as credenciais do usuário e retorna o token JWT.

#### Exemplo de Requisição:
```json
{
  "name": "usuario123",
  "password": "senha123"
}
```

#### Exemplo de Resposta:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```



### **4.2** Validação de Rota Protegida

```tsx
const roleResponse = await fetch("http://localhost:8000/api/auth/protected_route", {
  method: "POST",
  headers: {
    Authorization: `Bearer ${loginData.access_token}`,
    "Content-Type": "application/json",
  },
});
```

#### Detalhes:
- **Endpoint:** `POST /api/auth/protected_route`
- **Finalidade:** Verifica o token de autenticação e retorna o papel do usuário.

#### Exemplo de Requisição:
```http
POST /api/auth/protected_route
Authorization: Bearer <access_token>
Content-Type: application/json
```

#### Exemplo de Resposta:
```json
{
  "user": {
    "role": "doctor"
  }
}
```



### **4.3** Redirecionamento

Com base no papel do usuário retornado pela rota protegida, o sistema redireciona para a tela correspondente:

```tsx
if (userRole === "doctor") {
  router.push("/doctorHome");
} else if (userRole === "nurse") {
  router.push("/nurseHome");
} else {
  setError("User role is not recognized.");
}
```



## **5.0** Conclusão

A integração da tela de login foi realizada com sucesso, utilizando práticas seguras de autenticação e gerenciamento de sessões. O fluxo de autenticação baseado em tokens JWT garante a segurança e a escalabilidade do sistema. O redirecionamento dinâmico com base no papel do usuário melhora a experiência e a personalização. 

Este modelo é modular e facilmente adaptável para novas funcionalidades ou papéis de usuário que possam ser introduzidos no futuro.