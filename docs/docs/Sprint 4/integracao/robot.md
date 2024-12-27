---
title: Integração da tela de robôs
sidebar_position: 3
---



## **1.0** Introdução

Este documento descreve a integração da página de robôs com a API do sistema. A página exibe uma lista de robôs registrados, permite a busca por nome, localização ou status, e redireciona o usuário com base no papel obtido do token.

## **2.0** Funcionamento dos Endpoints

A seguir, detalhamos os endpoints utilizados, seus exemplos de requisição e resposta, e como eles foram integrados no código.



### **2.1** Listagem de Robôs

#### Endpoint:
- **`GET /api/robots`**

#### Finalidade:
Retorna a lista de todos os robôs registrados no sistema, incluindo informações como nome, localização e status.

#### Exemplo de Requisição:
```http
GET http://localhost:8000/api/robots
Content-Type: application/json
```

#### Exemplo de Resposta:
```json
{
  "robots": [
    {
      "id": "r1",
      "name": "Robô Assistente 1",
      "current_location": "Enfermaria 2",
      "status": "Ativo"
    },
    {
      "id": "r2",
      "name": "Robô Assistente 2",
      "current_location": "Sala de Emergência",
      "status": "Em Manutenção"
    }
  ]
}
```

#### Integração no Código:
```tsx
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
      setRobots(data.robots); // Preenche a lista de robôs
      setFilteredRobots(data.robots); // Inicializa os robôs filtrados
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  fetchRobots();
}, []);
```

- **Funcionamento:** Os dados retornados do endpoint são armazenados nos estados `robots` e `filteredRobots`. O estado `filteredRobots` é usado para exibir robôs que atendem ao critério de pesquisa.



### **2.2** Busca por Robôs

A busca é realizada localmente no front-end, utilizando o estado `robots`.

#### Lógica de Busca:
```tsx
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
```

- **Funcionamento:** A função filtra os robôs com base no texto digitado pelo usuário (`searchQuery`). O resultado da busca é armazenado em `filteredRobots`, que é utilizado para renderizar a lista exibida.



### **2.3** Controle de Redirecionamento

#### Finalidade:
Define o caminho de redirecionamento com base no papel do usuário (médico ou enfermeiro).

#### Decodificação do Token:
```tsx
useEffect(() => {
  const role = getUserRoleFromToken();
  if (role === "nurse") {
    setRedirectPath("/nurseHome");
  } else if (role === "doctor") {
    setRedirectPath("/doctorHome");
  }
}, []);
```

- **Funcionamento:** A função `getUserRoleFromToken` decodifica o token JWT do usuário para determinar o papel (`role`) e ajusta o caminho de redirecionamento (`redirectPath`) de acordo.



## **3.0** Exibição dos Dados

A exibição dos robôs é feita utilizando o componente `RobotCard`.

#### Estrutura do Componente:
```tsx
<RobotCard
  key={robot.id}
  robot={robot.name}
  local={robot.current_location || "Localização desconhecida"}
  status={robot.status}
/>
```

- **Entrada de Dados:** Cada robô é passado como propriedade para o componente `RobotCard`, que exibe as informações de forma estilizada.



## **4.0** Conclusão

A integração da página de robôs utiliza um único endpoint para obter os dados e processa a busca localmente no front-end. A implementação é eficiente, com controle adequado de estados para busca, exibição e tratamento de erros. O redirecionamento baseado no papel do usuário garante que cada grupo tenha acesso à funcionalidade adequada.