---
title: Design Patterns
sidebar_position: 1
---

# 1. Design Patterns

*Design patterns* (ou padrões de projeto) são soluções reutilizáveis para problemas comuns no desenvolvimento de software. Eles são como "receitas" que ajudam a resolver desafios de design de forma organizada e eficiente, evitando retrabalho. A seguir, tem-se os padrões de projeto que iremos utilizar.

:::warning
Os padrões de projeto aqui apresentados são preliminares e podem ser ajustados posteriormente durante o desenvolvimento do projeto.
:::

---

## 1.1 Observer Pattern (Padrão Observador)

No projeto, o **Observer Pattern** é ideal para implementar o sistema de notificação entre o backend e o dashboard acessado pelos médicos e enfermeiros. Cada vez que o robo coleta novos dados de um paciente, o backend precisa notificar os profissionais de saúde envolvidos.
O **Observer Pattern** permite que objetos (observadores) sejam notificados de mudanças em outro objeto (sujeito) sem a necessidade de uma ligação rígida entre eles. No nosso caso, os médicos e enfermeiros se comportam como "observadores" que precisam ser atualizados quando há novas informações dos pacientes.
As vantagens de fazer a utilização desse padrão de projeto é de que o robo ou backend não precisa saber quem será notificado, apenas notifica os observadores. Além disso, novos tipos de observadores (como alertas para familiares, por exemplo) podem ser adicionados sem modificar o código existente. Permite também atualizações em tempo real, garantindo que os profissionais de saúde estejam sempre informados. 

### 1.1.1 Exemplo de implementação 

- **Classe Sujeito**: Representa o robo ou a API no backend que coleta e armazena as respostas dos pacientes.
```python
class Paciente:
    def __init__(self, nome):
        self._observers = []
        self.nome = nome
    
    def adicionar_observer(self, observer):
        self._observers.append(observer)
    
    def remover_observer(self, observer):
        self._observers.remove(observer)

    def notificar_observers(self, dados_paciente):
        for observer in self._observers:
            observer.update(dados_paciente)
    
    def nova_resposta(self, resposta):
        print(f"Nova resposta do paciente {self.nome}: {resposta}")
        self.notificar_observers(resposta)
```

- **Classe Observer**: Representa o médico ou enfermeiro que precisa ser notificado quando há uma nova resposta.
```python
class Medico:
    def update(self, dados_paciente):
        print(f"Notificação recebida pelo médico: Dados do paciente atualizados: {dados_paciente}")

class Enfermeiro:
    def update(self, dados_paciente):
        print(f"Notificação recebida pelo enfermeiro: Dados do paciente atualizados: {dados_paciente}")
```

- **Uso do Observer Pattern**:
```python
paciente = Paciente("João")

medico = Medico()
enfermeiro = Enfermeiro()

paciente.adicionar_observer(medico)
paciente.adicionar_observer(enfermeiro)

paciente.nova_resposta("Classificação de dor: 7")
```

---

## 1.2 Factory Pattern (Padrão de Fábrica)

O robo faz uma série de perguntas aos pacientes. Dependendo da resposta inicial ("Sim" ou "Não" sobre dor), ele pode gerar perguntas específicas para cada região do corpo ou classificação de dor. O **Factory Pattern** é útil para criar essas perguntas de forma dinâmica e eficiente, baseando-se no estado atual do paciente.
O **Factory Pattern** permite a criação de objetos (perguntas, nesse caso) sem expor diretamente a lógica de instanciamento ao cliente (robo). A fábrica decide qual pergunta criar com base nas condições atuais, como respostas anteriores.
Sendo assim, as vantagens de utilizar este padrão são que novos tipos de perguntas podem ser adicionados facilmente sem modificar o código do robo. O robo não precisa saber qual pergunta exata fazer, apenas invoca a fábrica para criar as perguntas. Além disso, Permite reutilizar a lógica de criação de objetos em diferentes partes do sistema.   

### 1.2.1 Exemplo de implementação

- **Classe Pergunta**: Define uma interface comum para diferentes tipos de perguntas.
```python
class Pergunta:
    def exibir(self):
        pass
```

- **Classes de Perguntas Concretas**:
```python
class Pergunta:
    def __init__(self, tipo=None, regiao=None):
        self.tipo = tipo
        self.regiao = regiao

    def exibir_pergunta(self):
        if self.tipo == "sentimento":
            return self.pergunta_sentimento()
        elif self.tipo == "dor":
            return self.pergunta_dor()
        elif self.tipo == "classificacao_dor" and self.regiao:
            return self.pergunta_classificacao_dor()
        else:
            return "Tipo de pergunta inválido."

    def pergunta_sentimento(self):
        return "Como você está se sentindo?"

    def pergunta_dor(self):
        return "Você está com alguma dor? Se sim, em qual região?"

    def pergunta_classificacao_dor(self):
        return f"Classifique sua dor na região {self.regiao}, de 0 a 10."

```

- **Classe Fábrica**: Decide qual pergunta criar com base na lógica de negócio.
```python
class PerguntaFactory:
    def criar_pergunta(tipo, regiao=None):
        if tipo == "sentimento":
            return PerguntaSentimento()
        elif tipo == "dor":
            return PerguntaDor()
        elif tipo == "classificacao_dor":
            return PerguntaClassificacaoDor(regiao)
```

- **Uso do Factory Pattern**:
```python
factory = PerguntaFactory()

# Pergunta inicial
pergunta1 = factory.criar_pergunta("sentimento")
print(pergunta1.exibir())

# Pergunta sobre dor
pergunta2 = factory.criar_pergunta("dor")
print(pergunta2.exibir())

# Pergunta de classificação de dor para uma região específica
pergunta3 = factory.criar_pergunta("classificacao_dor", regiao="costas")
print(pergunta3.exibir())
```
---

## 1.3 Command Pattern (Padrão de Comando)

No projeto, o robo executa uma série de comandos, como iniciar uma interação com o paciente, fazer uma pergunta ou capturar uma resposta. O **Command Pattern** encapsula essas ações como objetos, permitindo que elas sejam passadas, armazenadas e até desfeitas ou refeitas.
O **Command Pattern** permite encapsular cada ação que o robo executa como um objeto independente. Isso torna mais fácil controlar a execução de comandos e estender o comportamento do robô de maneira modular.
OS benefícios de utilizar esse padrão é que novos comandos podem ser criados sem modificar o código existente. É possível armazenar comandos para serem refeitos ou desfeitos, a lógica de cada ação do robo é separada em classes de comandos, o que facilita a manutenção.

### 1.3.1 Exemplo de implementação

- **Interface Comando**: Define a estrutura de um comando.
```python
class Comando:
    def executar(self):
        pass
```

- **Classes de Comando Concretas**:
```python
class Pergunta:
    def __init__(self, tipo=None, regiao=None):
        self.tipo = tipo
        self.regiao = regiao

    def exibir_pergunta(self):
        if self.tipo == "sentimento":
            return self.pergunta_sentimento()
        elif self.tipo == "dor":
            return self.pergunta_dor()
        elif self.tipo == "classificacao_dor" and self.regiao:
            return self.pergunta_classificacao_dor()
        else:
            return "Tipo de pergunta inválido."

    def pergunta_sentimento(self):
        return "Como você está se sentindo?"
    
    def pergunta_dor(self):
        return "Você está com alguma dor? Se sim, em qual região?"

    def pergunta_classificacao_dor(self):
        return f"Classifique sua dor na região {self.regiao}, de 0 a 10."
```

- **Classe Invocador**: Responsável por armazenar e executar comandos.
```python
class TurtleBot:
    def __init__(self):
        self._comandos = []
    
    def adicionar_comando(self, comando):
        self._comandos.append(comando)
    
    def executar_comandos(self):
        for comando in self._comandos:
            print(comando.executar())
```

- **Uso do Command Pattern**:
```python
bot = TurtleBot()

comando1 = PerguntarSentimento()
comando2 = PerguntarDor()
comando3 = ClassificarDor("costas")

bot.adicionar_comando(comando1)
bot.adicionar_comando(comando2)
bot.adicionar_comando(comando3)

bot.executar_comandos()
```

---

## 1.4 Model-View-Controller (MVC)

O padrão MVC é aplicável ao design do dashboard que será acessado pelos médicos e enfermeiros para visualizar os dados coletados. Ele separa a lógica de negócio, a apresentação dos dados e a interação do usuário.
O Model-View-Controller (MVC) é amplamente utilizado para separar a lógica de aplicação em três componentes principais, melhorando a organização e a manutenibilidade do código.
Neste padrão de projeto, cada componente tem uma responsabilidade clara e distinta, cada componente pode ser testado de forma isolada. A separação das camadas facilita modificações sem afetar o sistema inteiro.
- Model: Responsável pelos dados do sistema (ex: pacientes, respostas, perguntas).
- View: Responsável pela apresentação das informações ao usuário (dashboard).
- Controller: Lida com a lógica de controle da aplicação e a interação do usuário.

### 1.4.1 Exemplo de Implementação

- Model: Gerencia os dados dos pacientes e respostas.
- View: Dashboard para médicos.
- Controller: Faz a intermediação entre o frontend e o backend, realizando a coleta e envio de dados.

---

## 1.5 Conclusão

O uso de design patterns no projeto ajuda a organizar o código, promover a reutilização de soluções e melhorar a manutenção do sistema. A aplicação de padrões como Observer, Factory, Command, e MVC é essencial para resolver os desafios de comunicação entre o robô, backend e o dashboard, garantindo uma implementação eficiente e flexível.