---
title: Integração da Tela nurseHome
sidebar_position: 5
---

# Integração da Tela ```nurseHome```

## **1.0** Introdução

A tela `nurseHome` é voltada para enfermeiros, possibilitando acessar informações gerais sobre todos os pacientes cadastrados. Sua integração é projetada para proporcionar uma visão abrangente dos dados dos pacientes.

## **2.0** Funcionalidades

1. **Listar Todos os Pacientes**:
   - **Endpoint**: `GET /patients/`
   - **Finalidade**: Retorna todos os pacientes cadastrados no sistema.
   - **Exemplo de Resposta**:
     ```json
     {
         "patients": [
             {
                 "id": "12345",
                 "name": "João da Silva",
                 "dob": "1980-01-15",
                 "conditions": ["Hipertensão"]
             },
             {
                 "id": "54321",
                 "name": "Maria Oliveira",
                 "dob": "1990-05-20",
                 "conditions": ["Diabetes"]
             }
         ]
     }
     ```

2. **Procurar Paciente por Nome**:
   - **Endpoint**: `GET /patients/search?name={name}`
   - **Finalidade**: Permite buscar pacientes pelo nome.

## **3.0** Código de Integração

### **3.1** Listar Pacientes
```tsx
const patientsResponse = await fetch(`http://localhost:8000/patients/`, {
  method: "GET",
  headers: {
    Authorization: `Bearer ${accessToken}`,
    "Content-Type": "application/json",
  },
});
```
### 3.2 Buscar Paciente por Nome
```tsx
const searchResponse = await fetch(
  `http://localhost:8000/patients/search?name=${searchName}`,
  {
    method: "GET",
    headers: {
      Authorization: `Bearer ${accessToken}`,
      "Content-Type": "application/json",
    },
  }
);
```
## 4.0 Conclusão
A tela ```nurseHome``` oferece um painel eficiente para consulta de pacientes, garantindo acesso fácil e seguro às informações necessárias. A integração utiliza autenticação JWT para proteção dos dados.