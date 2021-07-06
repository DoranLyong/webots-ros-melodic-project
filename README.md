# Webots World (ongoing)

[실행환경]
* Ubuntu 18.04 
* ROS Melodic  
* Nvidia GPU driver (CUDA 11.2가 가능한 GPU 디바이스)



### 로컬 PC에 __universal_robot__ ROS 패키지 설치하기 
(1) 로컬 PC에 [ROS Melodic](http://wiki.ros.org/melodic)이 설치 된 것으로 간주한다 <br/>
(2) [universal_robots](http://wiki.ros.org/universal_robots) 패키지를 ROS 버전에 맞게 설치한다 (i.e., ```melodic```) <br/>
  * ```apt-get``` 이 아닌 ```git clone```으로 소스빌드 할 것을 권장한다 

```bash
# 로컬 PC의 ~/.bashrc 에서 

export ROS_IP=x.x.x.x   # 로컬 PC의 이더넷 IP 
export ROS_HOSTNAME=$ROS_IP
export ROS_ROS_MASTER_URI=http://$ROS_IP:11311   # eth IP of your local PC for ROS Master 
```
***

### Webots World 설치하기 
#### 1. Install Docker and Nvidia-docker 
* [docker.com](https://docs.docker.com/engine/install/ubuntu/)으로 가서 자신의 컴퓨터 환경에 맞는 Docker Engine을 설치한다 
* 루트(root) 권한 설정; ```sudo``` 없이 도커 사용하기 
  ``` bash
  ~$ sudo usermod -aG docker $USER #현재 접속중인 사용자에게 권한주기 
  ```
* [docker.com/compose](https://docs.docker.com/compose/install/)에서 Docker Compose를 설치한다 
* [nvidia.com](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)를 참고해서 Nvidia-docker를 설치한다 
  * 단, 사전에 로컬 PC에서 ```~$ watch nvidia-smi``` 명령어가 가능해야한다. 
  * 위의 명령어가 안 된다면 본인의 환경에 맞게 GPU 드라이버를 설치해야 한다 


#### 2. Pull and Install the docker image 
* 현재 깃허브 레포지토리에 있는 파일을 가지고 아래 명령어를 순서대로 실행한다 
```bash
~$ sudo chmod a+x make_webots-ros_container.bash run_webots-ros_container.bash  # 실행 권한 부여 

~$ ./make_webots-ros_container.bash  # 도커 이미지 설치 및 컨테이너 생성 

~$ ./run_webots-ros_container.bash

```


<br/>

* 위의 명령어가 실행되면 ```webots_melodic``` 이란 이름으로 도커 컨테이너가 생성된다 
* 해당 컨테이너를 다시 실행하고 싶다면 로컬 PC의 터미널에서 ```~$ ./run_webots-ros_container.bash``` 명령을 다시 실행한다 
* 도커 컨테이너의 ```~/.bashrc```에서 ```ROS_IP```, ```ROS_HOSTNAME```, ```ROS_MASTER_URI``` 환경 변수 값을 아래와 같이 할당한다 [(참고 링크)](https://www.ybliu.com/2020/05/ros-remote-debgging-and-communication.html): 

```bash 
# 도커의 ~/.bashrc 에서 

export ROS_IP=172.17.0.x  # in the docker container for webots world
                          # 현재 생성된 도커 컨테이너의 주소를 확인하고 입력하기  
export ROS_HOSTNAME=$ROS_IP
export ROS_ROS_MASTER_URI=http://172.17.0.1:11311   # docker IP of your local PC for ROS Master 
```


  

#### 3. Run the webots world and Connect with ROS (여기서 부터는 도커 환경에서...)
* 도커 환경에서 아래 명령어를 각각 실행한다 (각각 새로운 도커 터미널에서 실행할 것 )
```bash
~# webots 

~# ~/ur_e_webots.bash   # webots world의 스텝을 실행시킬 수 있는 ROS 패키지 실행 

~# ~/sensor_enable.bash  # webots world의 센서 장비가 토픽을 ROS_MASTER터로 발행하도록 서비스 요청(call)

~# python ~/objects_random_place.py   # 물체 위치 랜덤 
```

* 위의 명령은 다음과 같이 간략하게 실행시킬 수 있다 
```bash
~# ~/run_WebotsWorld_all.bash  # webots -> sensor_enable.bash -> python objects_random_place.py 순으로 실행됨 

~# python ~/objects_random_place.py   # 새로운 도커 터미널에서 실행할 것 
```

***
### 로컬 PC에서 Webots World 제어하기 


#### 4. 영상 데이터 Subscribe 테스트 
* 여기까지 수행했다면 webots world 에서 ROS Master로 RGB 카메라 및 Range Finder 데이터 토픽(topic)을 발행한다 
* 아래 토픽 버스 이름을 통해 영상 정보를 확인할 수 있다 
* [Image topic subscribe](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/catkin_ws/src/ur_e_webots/scripts/webots_ros_tutorial.py) 예시 
```bash 
# RGB image topic 
/CAM/camera/image

# Depth image topic 
/CAM/range_finder/range_image
```

##### ※ Camera world → Robot world 변환 행렬 구하기 
```bash 
[RGB & depth_map devices]

- Translation[x, y, z] : 1.35272 , 1.3315 , 5.97883e-07
- Rotation(quaternions)[w, x, y, z] : 0.653284 , -0.270596 , 0.65328 , 0.270597
```



* [3D Rotation Converter 계산기](https://www.andre-gaschler.com/rotationconverter/)




#### 5. UR10 로봇 제어
* 자세한 UR10 로봇 제어과 관련된 내용은 [메뉴얼](https://cyberbotics.com/doc/guide/ure)을 참고한다. 
* webots 시뮬레이션 상의 UR10e 로봇을 제어하기 위해서는 webots 상의 controller를 ```<extern>``` 값으로 할당해야 한다. 
* 제어 코드는 [ROS 통신으로 센싱 값을 주고 받으면서](https://github.com/cyberbotics/webots_ros/blob/master/scripts/ros_controller.py) 처리된 제어 신호를 [webots world의 로봇 객체에 할당하는](https://github.com/cyberbotics/webots_ros/blob/master/scripts/ros_python.py) 방식으로 코딩한다 

##### (1) MoveIt 활용 예시 
* [User Guide](https://cyberbotics.com/doc/guide/ure?tab-language=python#ros) 참고 

  ```bash  
  # webots 센싱 데이터와 제어신호를 주고받기 위한 패키지 예시 
  ~$ roslaunch ur_e_webots ur10e.launch
  
  # MoveIt 연동 패키지 예시 
  ~$ roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch
  
  ~$ roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true   
  ```
* 여기까지 실행이 끝났다면 webots의 가상환경이 ROS rviz에 연동된 것을 확인할 수 있다 
* 콘솔 명령을 통해 MoveIt에 간단한 명령을 입력하는 인터페이스 튜토리얼을 보고 싶다면, 다시 새로운 터미널을 켜고 해당 도커 컨테이너를 실행한 뒤 다음 ROS 패키지를 실행한다: 
  ```bash
  ~$ rosrun moveit_tutorials move_webots_ur_e_python_interface.py  
  ```
  * ```x, y, z``` 좌표를 입력하면 moveit 상의 머니퓰레이터가 움직이면서 webots 가상환경과 연동된 것을 확인할 수 있다 
  * (ex) ```0.8, -0.3, 0.8``` 

##### (2) 커스텀 제어 예시 - Grippers & URe joints 제어 
* [webots_ros를 통한 컨트롤러 예시](https://github.com/cyberbotics/webots_ros/tree/master/scripts) 참고 
* [사용된 Gripper의 joints 정보](https://cyberbotics.com/doc/guide/gripper-actuators#robotiq-3f-gripper) 및 [UR10e 로봇의 joints 정보](https://cyberbotics.com/doc/guide/ure?tab-language=python) 참고 
* webots controller 코드 예시 (without ROS) [.c 버전](https://github.com/cyberbotics/webots/blob/master/projects/robots/universal_robots/controllers/ure_can_grasper/ure_can_grasper.c)  / [.py 버전](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/controllers/ure_grasper_tutorial/ure_grasper_tutorial.py)



##### (3) webots-ROS 개발 Tip
* [User Guide](https://cyberbotics.com/doc/guide/ure?tab-language=python#ros)의 ```roslaunch ur_e_webots ur10e.launch``` 패키지를 참고한다 
* 위의 패키지는 [ur_e_webots 깃허브](https://github.com/cyberbotics/webots/tree/released/projects/robots/universal_robots/resources/ros_package/ur_e_webots)에서 확인 할 수 있다 
* 해당 패키지의 ```scripts``` 디렉토리로 가면 [universal_robots_ros.py](https://github.com/cyberbotics/webots/blob/released/projects/robots/universal_robots/resources/ros_package/ur_e_webots/scripts/universal_robots_ros.py)가 있다. 
* 이 코드를 보면 [ure_grasper_tutorial.py](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/controllers/ure_grasper_tutorial/ure_grasper_tutorial.py) 코드와 유사한 형태를 가지고 있다 : 
  ```python
  from controller import Robot

  # create the Robot instance.
  robot = Robot()

  # get the time step of the current world.
  timestep = int(robot.getBasicTimeStep())

  [... 생략...] 

  # Main loop:
  while robot.step(timestep) != -1:  # 이 루프는 webots world가 실행되기 위해 반드시 필요하다 
    pass 
  ``` 
* 즉, webots 상의 로봇을 제어하는 코드는 [ure_grasper_tutorial.py](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/controllers/ure_grasper_tutorial/ure_grasper_tutorial.py)를 참고하고, webots world의 센싱 데이터를 받고 처리한 뒤에 다시 webots world의 로봇을 제어하기 위한 코드는 [ur_e_webots](https://github.com/cyberbotics/webots/tree/released/projects/robots/universal_robots/resources/ros_package/ur_e_webots) 패키지를 참고한다 
  



#### 6. 기타 
* 그 이외 webots과 관련된 내용은 [webots_ros github](https://github.com/cyberbotics/webots_ros) 및 [ros.org의 webots 위키](http://wiki.ros.org/webots) 참고. 

***

#### 7. 대회 준비를 위한 Tip
* 경기는 물리적으로 분리된 두 대의 컴퓨터로 진행된다. 
* 한 대는 ```ROS_MASTER```와 ```webots world```가 실행될 경연 당일 준비될 경기장 컴퓨터(주최 측에서 준비), 다른 한 대는 경기 참여자의 개인 컴퓨터이다. 
* 경기장 컴퓨터는 ```ROS_MASTER``` 및 ```webots world``` 가 실행되는 서버 역할이다. 
* 경기 참여자의 ```ROS_HOST```는 ```ROS_MASTER``` 에 연결되어 ```webots world```로 부터 센싱 데이터를 받고 UR10e 로봇 제어를 통해 재활용 쓰레기를 분리수거 하는 과업(task)을 수행한다. 
* 즉, 경기 참여자가 개인 로컬 PC에 기본적으로 갖춰야 할 코드는 [roslaunch ur_e_webots ur10e.launch](https://cyberbotics.com/doc/guide/ure?tab-language=python#ros) 와 같이 webots world의 스텝을 실행시킬 수 있는 패키지와 [visual_sensor_enable.py](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/catkin_ws/src/ur_e_webots/scripts/visual_sensor_enable.py) 패키지처럼 webots world의 센서 객체들이 ```ROS_MASTER```로 센싱 값을 토픽으로 발행하도록 요청하는 노드이다. 
  * webots world에서 발행하는 RGB와 depth 영상 토픽은 [webots_ros_tutorial.py](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/catkin_ws/src/ur_e_webots/scripts/webots_ros_tutorial.py)와 같은 형태로 ```Subscribe``` 하면 된다
  * ```UR10e``` 로봇의 제어는 위에서 소개된 대로 [ur_e_webots](https://github.com/cyberbotics/webots/tree/released/projects/robots/universal_robots/resources/ros_package/ur_e_webots/scripts) 밑바탕으로 webots world의 스텝이 실행 가능하도록 패키지를 구성하고 [ure_grasper_tutorial.py](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/controllers/ure_grasper_tutorial/ure_grasper_tutorial.py)를 참고해서 [Gripper](https://cyberbotics.com/doc/guide/gripper-actuators#robotiq-3f-gripper) 기능을 추가할 수 있다. 
  * 개발을 위한 라이브러리는 [cyberbotics.com](https://cyberbotics.com/doc/guide/index), [webots github](https://github.com/cyberbotics/webots/tree/released/projects/robots/universal_robots/resources/ros_package/ur_e_webots), [webots_ros github](https://github.com/cyberbotics/webots_ros) 그리고 [ros.org](http://wiki.ros.org/webots_ros)에서 확인할 수 있다. 


***

#### Reference 
* [ROS](http://wiki.ros.org/Distributions) & [webots](https://cyberbotics.com/) 설치 
* [universal_robots](http://wiki.ros.org/universal_robots) 설치 
* [WEBOTS_HOME](https://cyberbotics.com/doc/guide/tutorial-8-using-ros) 환경변수 설정 
* [UR 로봇 프로젝트 파일을 catkin_ws로 옮기고](https://cyberbotics.com/doc/guide/ure) rviz에 띄우기 
* webots의  URe 로봇 모델을 ROS로 보내기 위해서는 → webots 월드에서 URe의 컨트롤러를 ```extern```

