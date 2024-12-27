---
title: Integração da Tela doctorHome
sidebar_position: 4
---

# Integração da Tela doctorHome

## **1.0** Introdução

A tela `doctorHome` é destinada aos médicos, permitindo gerenciar pacientes associados. A integração com o backend possibilita buscar, associar e desassociar pacientes de forma eficiente e segura.

## **2.0** Funcionalidades

1. **Buscar Pacientes Associados ao Médico**:
   - **Endpoint**: `GET /patients/professional/{userId}`
   - **Finalidade**: Retorna todos os pacientes associados ao médico atual.
   - **Exemplo de Resposta**:
     ```json
     {
         "patients": [
             {
                 "id": "12345",
                 "name": "João da Silva",
                 "dob": "1980-01-15",
                 "conditions": ["Hipertensão"]
             }
         ]
     }
     ```

2. **Procurar Paciente por Nome**:
   - **Endpoint**: `GET /patients/search?name={name}`
   - **Finalidade**: Busca pacientes pelo nome.
   - **Exemplo de Resposta**:
     ```json
     {
         "id": "54321",
         "name": "Maria Oliveira",
         "dob": "1990-05-20",
         "conditions": ["Diabetes"]
     }
     ```

3. **Associar Paciente ao Médico**:
   - **Endpoint**: `PUT /patients/{patientId}/associate/{userId}`
   - **Finalidade**: Associa um paciente ao médico atual.

4. **Desassociar Paciente**:
   - **Endpoint**: `PUT /patients/{patientId}/disassociate`
   - **Finalidade**: Remove a associação do paciente ao médico.

## **3.0** Código de Integração

### **3.1** Buscar Pacientes Associados
```tsx
const patientsResponse = await fetch(
  `http://localhost:8000/patients/professional/${userId}`,
  {
    method: "GET",
    headers: {
      Authorization: `Bearer ${accessToken}`,
      "Content-Type": "application/json",
    },
  }
);
```
### 3.2 Associar Paciente
```tsx
const associateResponse = await fetch(
  `http://localhost:8000/patients/${patientId}/associate/${userId}`,
  {
    method: "PUT",
    headers: {
      Authorization: `Bearer ${accessToken}`,
      "Content-Type": "application/json",
    },
  }
);
```
## 4.0 Conclusão: 
A tela ```doctorHome``` foi integrada com sucesso, permitindo que médicos gerenciem seus pacientes de forma eficiente. O uso de autenticação JWT garante a segurança das operações.