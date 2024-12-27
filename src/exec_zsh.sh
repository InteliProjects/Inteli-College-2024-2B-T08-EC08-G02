#!/bin/zsh

# Executa ao sair do script
trap "echo 'Limpando... (apagando venv)'; rm -rf $(pwd)/venv" EXIT

python3 -m venv venv

source venv/bin/activate

pip install typer inquirer python-dotenv

cd cora_ws/cora 
# Pega o caminho da instaância do venv
TYPER_PATH=$(pip show typer | grep "Location:" | awk '{print $2}')

# Se não achar
if [ -z "$TYPER_PATH" ]; then
  echo "Typer not found. Please make sure it is installed."
  exit 1
fi

# Exporta a variável de ambiente PYTHONPATH com o caminho do venv
export PYTHONPATH="$PYTHONPATH:$TYPER_PATH"

# Builda e roda o ros2
colcon build

source install/local_setup.zsh

export SSH_HOST="10.128.0.62"
export SSH_USER="cora"
export SSH_PASSWORD="Password123!"

# TODO: Change ros2 run code
ros2 run cora cora
