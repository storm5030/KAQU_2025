# KAQU_2025
최효린

- [1] ROS2 Humble
    
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
    
- [2] Terminator
    
    Ubuntu Software - Terminator 
    
    ```bash
    sudo apt update
    sudo apt install terminator
    ```
    
- [3] VSCode
    
    ```bash
    sudo apt update
    sudo apt install wget gpg
    ```
    
    ```bash
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
    sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
    ```
    
    ```bash
    sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] \
    https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
    ```
    
    ```bash
    sudo apt update
    sudo apt install code
    ```
    
    VSCode Extensions (좌측 5번째 아이콘) 추천 익스텐션
    
    - Git Graph
    - Material Icon Theme
- [4] .bashrc 설정
    
    ```bash
    #터미널
    code ~/.bashrc
    
    #.bashrc파일 맨 아래에 작성
    alias sb="source ~/.bashrc; echo \"bashrc is reloaded!\""
    #alias로 설정
    alias ros_domain="export ROS_DOMAIN_ID=13; echo \"ROS_DOMAIN_ID=13\""
    alias humble="source /opt/ros/humble/setup.bash; ros_domain; echo \"ROS2 Humble is activated!\""
    
    #터미널 켜질 때마다 실행하려면
    source /opt/ros/humble/setup.bash
    echo "ROS2 Humble is activated!"
    export ROS_DOMAIN_ID=13
    
    #
    source ~/.bashrc
    ```
    
- [5] 워크스페이스 빌드
    
    ```bash
    #워크스페이스 폴더 생성(이름은 마음대로 / -p 옵션: 상위 경로도 생성)
    mkdir -p ~/kaqu_ws/src
    cd ~/kaqu_ws
    colcon build --symlink-install
    ```
    
- [6] 깃허브 코드 가져오기
    
    https://github.com/ 깃허브 회원가입
    
    ```bash
    sudo apt-get install git
    git --version
    git config --global user.name #유저 이름
    git config --global user.mail #유저 이메일
    
    #깃 정보 확인
    **git config -l**
    ```
    
    ```bash
    #코드 가져오기
    cd ~/kaqu_ws/src
    git clone https://github.com/storm5030/KAQU_2025.git
    ```
    
    ```bash
    #저장소 연결 확인
    cd ~/kaqu_ws/src/KAQU_2025
    git remote -v
    
    #코드 업데이트 (PULL): 원격저장소에서 불러와 덮어쓰
    git pull origin main
    ```
    
    https://github.com/storm5030/KAQU_2025.git
    
- [7] Dynamixel SDK 설치
    
    ```bash
    sudo apt install python3-pip
    pip3 install dynamixel-sdk
    ```
    
- [8] Gazebo Fortress  설치
    
    https://gazebosim.org/docs/fortress/install_ubuntu/
    
    ```jsx
    sudo apt-get update
    sudo apt-get install lsb-release gnupg
    
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install ignition-fortress
    ```
    
- [9] gz_ros2_control 패키지 설치 필요
    
    ```jsx
    sudo rosdep init
    rosdep update
    ```
    
    https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html
    
    ```jsx
    sudo apt update
    sudo apt install ros-humble-gz-ros2-control ros-humble-gz-ros2-control-demos
    sudo apt install ros-humble-ros2-control
    sudo apt install ros-humble-ros2-controllers
    
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```
    
    .bashrc에 추가 (code ~/.bashrc) 맨 아래에 추가
    
    ```bash
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib
    ```
    
    source ~/.bashrc 입력하여 업데이트
    
- [10] Rviz joint state publisher 설치
    
    ```bash
    sudo apt update
    sudo apt install ros-humble-joint-state-publisher-gui
    ```
    
- [11] 실행
    - 별도 실행파일 (디버깅용)
        
        ```bash
        humble
        cd ~/kaqu_ws
        colcon build --symlink-install
        #터미네이터에서 화면분할 5개 이상 ctrl+shift+E(가로) 또는 ctrl_shift+O(세로)
        
        #(.bashrc에 추가 안한 경우)각 터미널에서 모두
        humble
        source install/setup.bash
        
        #1번 터미널
        ros2 run kaqu_controller RobotManagerNode 
        
        #2번 터미널
        ros2 run kaqu_controller AnglePulbisherNode
        
        #3번 터미널
        ros2 launch kaqu_input_manager kaqu_teleop.launch.py
        
        #4번 터미널
        ros2 launch kaqu_gazebo_sim kaqu_gazebo_sim.launch.py
        
        #5번 터미널 (하드웨어 가동시)
        ros2 run kaqu_hardware_interfacing bulk_read_write
        ```
        
    - 통합 실행파일
        
        ```bash
        humble
        cd ~/kaqu_ws
        colcon build --symlink-install
        source install/setup.bash
        ros2 launch kaqu kaqu.launch.py
        ```
        
- [12] Rviz실행
    
    ```bash
    
    ros2 launch kaqu_gazebo_sim kaqu_rviz.launch.py
    ```
    
    로봇이 안 보일 경우 Fixed Frame → base link 선택
    
    Add → RobotModel
    
    RobotModel에서 Description Topic → /robot_description 선택
