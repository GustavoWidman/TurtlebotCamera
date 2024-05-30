# TurtlebotCamera - Gustavo Wagon Widman

## Descrição

Essa atividade constrói acima da atividade [TurtlebotTeleop](https://github.com/GustavoWidman/TurtlebotTeleop) a adiciona algumas funcionalidades, como a adição de disponibilização de diversos endpoints para a teleoperação do TurtleBOT, e a adição de um endpoint de websocket para a visualização da câmera do TurtleBOT. O pacote inclui uma aplicação Python que utiliza o framework ROS (Robot Operating System) para controlar o TurtleBOT utilizando o teclado (WASD ou setas) e visualizar a câmera do TurtleBOT em um navegador web. No navegador web, é possível visualizar a câmera do TurtleBOT em tempo real e controlar o TurtleBOT utilizando as setas na tela, além de poder parar o TurtleBOT e chamar o serviço de emergência. Por via de mais um endpoint de websocket, o site também é capaz de mostrar ao usuário o estado atual do TurtleBOT, como a velocidade linear e angular.

## Dependências

Certifique-se de que você possui o `ROS2 Humble` e o `colcon` instalado em sua máquina. Caso não tenha, siga as instruções de instalação amigáveis (nao oficiais) aqui [ROS2 Humble Unofficial Installation](https://rmnicola.github.io/m6-ec-encontros/E01/ros) ou siga as instruções oficiais [ROS2 Installation](https://docs.ros.org/en/humble/Installation.html). Também é necessário ter o simulador `webots` instalado em sua máquina. Para instalar o simulador, rode o seguinte comando:

```bash
sudo apt instal webots
```

O script `run.sh` cuidara de instalar as outras dependências necessárias para o pacote. Note que o `run.sh` necessita do pacote `python3.10-venv` para funcionar. Caso não tenha, instale-o com o seguinte comando:

```bash
sudo apt install python3.10-venv
```

Lista de dependências de Python:

- [inquirerpy](https://pypi.org/project/inquirerpy/) v0.3.4
- [opencv-python](https://pypi.org/project/opencv-python/) v4.9.0.80
- [websockets](https://pypi.org/project/websockets/) v12.0
- [fastapi](https://pypi.org/project/fastapi/) v0.111.0
- [uvicorn](https://pypi.org/project/uvicorn/) v0.30.0

O frontend do pacote foi feito em HTML, CSS e JavaScript, e não necessita de nenhuma dependência adicional. Ele automaticamente baixa e utiliza o [Bulma CSS](https://bulma.io/) para a estilização do site.

## Instalação e Execução

Para instalar o pacote, basta clonar o repositório:

```bash
git clone https://github.com/GustavoWidman/TurtlebotCamera.git
```

E, em seguida, rodar o script de instalação e execução:

```bash
cd TurtlebotCamera
chmod +x run.sh # averiguar se é necessário
./run.sh
```

Bem, já que o pacote foi instalado e esta rodando, para ver o TurtleBOT se movendo, abra um outro terminal e rode o script do simulador `simulator.sh` (ele cuidará de abrir o simulador `webots`):

```bash
chmod +x simulator.sh # averiguar se é necessário
./simulator.sh
```

Agora pronto! O TurtleBOT está pronto para ser controlado. Para controlar o TurtleBOT, basta seguir as instruções que aparecerão no terminal. O TurtleBOT pode ser controlado teleoperadoramente pelo terminal, da mesma maneira que no pacote [TurtlebotTeleop](https://github.com/GustavoWidman/TurtlebotTeleop) ou pelo site que será aberto automaticamente no navegador. Para acessar o site, abra o navegador e acesse o endereço `http://localhost:8000`. Note que o site só funcionará se o pacote estiver rodando. Controlando o TurtleBOT pelo terminal também mudará o estado do TurtleBOT no site, e vice-versa.

## Disclaimer

Por favor nao tente rodar o pacote em qualquer diretório que nao seja o `TurtlebotCamera/src` (usando instalação manual) ou o `TurtlebotCamera` (usando o script `run.sh`). O pacote foi feito para rodar nesses diretórios e pode não funcionar corretamente em outros diretórios por causa de problemas de importação do arquivo do frontend.

## Instalação Manual

Caso prefira instalar manualmente, siga os seguintes passos, começando na pasta `TurtlebotCamera` (base do repositório):

1. Crie um ambiente virtual Python:

```bash
python3 -m venv venv
source venv/bin/activate
```

2. Instale as dependências de Python:

```bash
pip install -r requirements.txt
```

3. Localize e adicione o seu venv ao ROS2:

```bash
export PYTHONPATH="$PYTHONPATH:$(pip show setuptools | grep "Location: " | awk '{print $2}')"
```

4. Compile o pacote:

```bash
cd src
colcon build
source install/setup.bash # ou setup.zsh se estiver usando zsh
```

5. Rode o pacote desejado:

```bash
ros2 run ros_turtlebot_camera main # para rodar o pacote de teleoperação
ros2 run ros_turtlebot_camera emergency # para rodar o pacote de chamada de emergência
```

## Serviço de Emergência

O pacote `main` inclui um serviço de emergência que pode ser chamado a qualquer momento. Para chamar o serviço de emergência, basta rodar o seguinte comando:

```bash
ros2 service call /emergency_stop std_srvs/srv/Empty
```

Ou rodar o script `emergency.sh`, que roda o pacote `emergency`:

```bash
chmod +x emergency.sh # averiguar se é necessário
./emergency.sh
```

O serviço de emergência imediatamente para o TurtleBOT (caso esteja em movimento) e encerra o programa (caso esteja rodando). O serviço de emergência é uma maneira segura de parar o TurtleBOT em caso de emergência. Note que por causa da natureza do serviço de emergência, chamá-lo irá encerrar o programa imediatamente, então nunca receberá uma resposta do serviço de emergência, ficando preso no comando mesmo depois do programa ter sido encerrado. Isso é normal e esperado, e não é um erro. Para sair do comando, basta apertar `Ctrl+C` após chamar o serviço de emergência.

Note que também é possível chamar o serviço de emergência pelo pacote desenvolvido `emergency`. Para chamar o serviço de emergência pelo pacote `emergency`, consulte a seção "Instalação Manual" e rode o pacote `emergency`:

```bash
ros2 run ros_turtlebot_camera emergency
```

O pacote `emergency` (rodado por `emergency.sh` ou diretamente usando `ros2 run`) roda o serviço de emergência da maneira esperada e nao aguarda a sua resposta, encerrando o programa imediatamente, levando a uma experiência de usuário mais amigável.

## API

O pacote `main` inclui uma API que pode ser acessada pelo site ou por qualquer outro programa que deseje controlar o TurtleBOT. A API inclui os seguintes endpoints:

- GET `/control?direction=<direction>`: Controla o TurtleBOT na direção especificada.
	- A direção pode ser `forward`, `backward`, `left`, `right`, ou `stopped` (default é `stopped` caso não seja especificado).
	- Exemplo: `http://localhost:8000/control?direction=forward`.

- GET `/emergency`: Chama o serviço de emergência, parando o TurtleBOT e encerrando o programa de maneira imediata.
	- Exemplo: `http://localhost:8000/emergency`.

- GET `/stop`: Para o TurtleBOT, encerrando o programa de maneira segura.
	- Exemplo: `http://localhost:8000/stop`.

- WS `/image`: WebSocket que transmite a imagem da câmera do TurtleBOT em tempo real.
	- Exemplo: `ws://localhost:8000/image`.
	- O WebSocket transmite a imagem da câmera do TurtleBOT em tempo real. A imagem é transmitida em formato JSON, com a chave `bytes` contendo a imagem JPEG em base64 e a chave `timestamp` contendo o timestamp UTC em UNIX.
	- Formato do JSON transmitido: `{"bytes": "<base64 image>", "timestamp": <UTC UNIX timestamp (int)>}`.

- WS `/state`: WebSocket que transmite o estado atual do TurtleBOT em tempo real.
	- Exemplo: `ws://localhost:8000/state
	- O WebSocket transmite o estado atual do TurtleBOT em tempo real. O estado é transmitido em formato JSON, com a chave `state` contendo a direção atual do TurtleBOT e a chave `speed` contendo a velocidade linear e angular do TurtleBOT (chaves `linear` e `angular` respectivamente)
	- Formato do JSON transmitido: `{"state": <"forward", "backward", "left", "right", "stopped">, "speed": {"linear": <linear speed (float)>, "angular": <angular speed (float)>}}`.

## Demonstração

A seguir eu demonstro o pacote em funcionamento, controlando o TurtleBOT no simulador `webots` usando a interface e depois o terminal, também chamando o serviço de emergência:

