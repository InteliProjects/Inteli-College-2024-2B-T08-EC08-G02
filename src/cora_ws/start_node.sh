#!/bin/bash

# Caminho para o diretório onde está o ambiente virtual
VENV_DIR=~./venv

# Caminho para o requirements.txt
REQUIREMENTS=~./src/cora/requirements.txt

# Verifica se o ambiente virtual já existe
if [ ! -d "$VENV_DIR" ]; then
  echo "Criando o ambiente virtual..."
  python3 -m venv $VENV_DIR
fi

# Ativa o ambiente virtual
echo "Ativando o ambiente virtual..."
source $VENV_DIR/bin/activate

# Verifica se o arquivo requirements.txt existe antes de instalar as dependências
if [ -f "$REQUIREMENTS" ]; then
  echo "Instalando dependências..."
  pip install -r $REQUIREMENTS
else
  echo "Erro: Arquivo requirements.txt não encontrado em $REQUIREMENTS"
  deactivate
  exit 1
fi

# Fonte o ambiente ROS 2 (certifique-se que o ROS 2 esteja configurado corretamente)
echo "Configurando o ambiente ROS 2..."
if [ -f /install/setup.bash ]; then
  source ./install/setup.bash
else
  echo "Erro: Arquivo setup.bash não encontrado, certifique-se de ter rodado 'colcon build'."
  deactivate
  exit 1
fi

# Roda o nó ROS 2 do pacote cora
echo "Iniciando o nó cora..."
ros2 run cora cora

# Desativa o ambiente virtual após a execução
deactivate
