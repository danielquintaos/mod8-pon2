Este guia oferece um caminho simplificado para a criação, configuração e utilização de pacotes em ROS2, abrangendo desde a criação de um workspace até a execução de pacotes e launch files personalizados.

**1. Criação de um Workspace ROS**
- **Início:** Crie uma pasta chamada `meu_workspace` e uma subpasta `src` dentro dela:
  ```bash
  mkdir -p meu_workspace/src
  ```
- **Adicionar Pacote:** Entre na pasta `src` e clone um pacote de exemplo:
  ```bash
  cd meu_workspace/src
  git clone https://github.com/ros/ros_tutorials.git -b humble
  ```

**2. Instalação e Configuração de Dependências**
- **Instalar rosdep:** Instale o `rosdep`, não incluso na instalação padrão do ROS:
  ```bash
  sudo apt install python3-rosdep
  ```
- **Configurar rosdep:** Inicialize e atualize o `rosdep`:
  ```bash
  sudo rosdep init
  rosdep update
  ```
- **Resolver Dependências:** Volte para a raiz do workspace e instale as dependências:
  ```bash
  cd meu_workspace
  rosdep install -i --from-path src --rosdistro humble -y
  ```

**3. Compilação do Pacote**
- **Build do Pacote:** Use `colcon build` para compilar o pacote. Ignore avisos sobre o `setuptools`, ou instale uma versão específica se necessário:
  ```bash
  colcon build
  # Em caso de problemas com setuptools
  pip install setuptools==58.2.0
  ```
- **Source no Setup Script:** Para executar o pacote, ative o script de configuração do workspace:
  ```bash
  source install/local_setup.bash # ou setup.zsh para zsh
  ```

**4. Criando seu Próprio Pacote ROS**
- **Criação de Pacote Pré-preenchido:**
  ```bash
  ros2 pkg create --build-type ament_python --node-name my_node my_package
  ```
  Depois do build, ative o script de setup:
  ```bash
  source install/local_setup.bash # ou setup.zsh para zsh
  ```
  Execute o pacote:
  ```bash
  ros2 run my_package my_node
  ```

- **Criação de Pacote Vazio:**
  - Crie um pacote chamado `ola_mundo` e um script `ola.py`:
    ```bash
    ros2 pkg create --build-type ament_python ola_mundo
    cd ola_mundo/ola_mundo
    touch ola.py
    ```
  - Preencha `ola.py` com um script simples e configure os arquivos `package.xml` e `setup.py` com informações do pacote e dependências.
  - Após configurar, retorne à raiz do workspace, compile e execute o pacote:
    ```bash
    colcon build
    source install/local_setup.bash # ou setup.zsh para zsh
    ros2 run ola_mundo ola
    ```

**5. Criação e Integração de Launch Files**
- **Criação de Launch Files:** Crie um arquivo de launch `test_launch.py` no diretório `launch` do seu pacote. Exemplo de conteúdo para `test_launch.py`:
  ```python
  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package='turtlesim',
              executable='turtlesim_node',
              name='sim',
              output='screen'
          ),
          Node(
              package='turtlesim',
              executable='turtle_teleop_key',
              name='teleop',
              prefix = 'gnome-terminal --',
              output='screen'
          )
      ])
  ```
- **Integração com Pacote:** Modifique o `setup.py` do seu pacote para incluir a pasta `launch` e seus arquivos no build. Depois, compile novamente o pacote e execute o launch file com:
  ```bash
  colcon build
  ros2 launch my_package test_launch.py
  ```

