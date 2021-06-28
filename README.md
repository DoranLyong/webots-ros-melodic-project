# webots-ros-melodic-project (미완성)

#### Install Docker and Nvidia-docker 
* [docker.com](https://docs.docker.com/engine/install/ubuntu/)으로 가서 자신의 컴퓨터 환경에 맞는 Docker Engine을 설치한다 
* 루트(root) 권한 설정; ```sudo``` 없이 도커 사용하기 
  ``` bash
  ~$ sudo usermod -aG docker $USER #현재 접속중인 사용자에게 권한주기 
  ```
* [docker.com/compose](https://docs.docker.com/compose/install/)에서 Docker Compose를 설치한다 
* [nvidia.com](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)를 참고해서 Nvidia-docker를 설치한다 
  * 단, 사전에 로컬 PC에서 ```~$ watch nvidia-smi``` 명령어가 가능해야한다. 
  * 위의 명령어가 안 된다면 본인의 환경에 맞게 GPU 드라이버를 설치해야 한다 


#### Pull and Install the docker image 
도커 설치가 완료됐다면 다음과 같이 명령어를 실행한다: 
```bash 
~$ xhost + 

~$ sudo chmod a+x run_webots_ros.bash

~$ ./run_webots_ros.bash
```
* 위의 명령어가 실행되면 ```webots_melodic``` 이란 이름으로 도커 컨테이너가 생성된다 
* 해당 컨테이너를 다시 실행하고 싶다면 로컬 PC의 터미널에서 다음 명령어를 실행한다 
  ``` bash
  ~$ docker start webots_melodic 
  ~$ docker exec -it webots_melodic /bin/bash
  ```


- [ ] ```--net```, ```--ipc``` 붙이니까 rviz가 안되네... 이부분 해결 필요 



#### Run webots and Connect with ROS 

* ```~$ webots``` 명령어를 기입하면 webots 가상환경이 실행된다. 
* 새로운 터미널에서 ```webots_melodic``` 컨테이너를 각각 실행해서 다음과 같이 명령어를 각각 기입한다: 
  * 컨테이너 실행은 맨 위의 [Pull and Install the docker image](https://github.com/DoranLyong/webots-ros-melodic-project/blob/main/README.md#pull-and-install-the-docker-image) 부분 참고 
  ```bash 
  ~$ roscore
  
  ~$ roslaunch ur_e_webots ur10e.launch
  
  ~$ roslaunch ur10_e_moveit_config ur10_e_moveit_planning_execution.launch
  
  ~$ roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true   
  ```
* 여기까지 실행이 끝났다면 webots의 가상환경이 ROS rviz에 연동된 것을 확인할 수 있다 
* 콘솔 명령을 통한 간단한 인터페이스 튜토리얼을 보고 싶다면 다시 새로운 터미널을 켜고 해당 도커 컨테이너를 실행한 뒤 다음 ROS 패키지를 실행한다: 
  ```bash
  ~$ rosrun moveit_tutorials move_webots_ur_e_python_interface.py  
  ```
  * ```x, y, z``` 좌표를 입력하면 moveit 상의 머니퓰레이터가 움직이면서 webots 가상환경과 연동된 것을 확인할 수 있다 
  * (ex) ```0.8, -0.3, 0.8``` 
  



- [ ] 추후 ```roslaunch```로 명령어를 간소화 시키자 
- [ ] 로컬 PC의 ROS와 도커 컨테이너 상의 ROS가 서로 같은 roscore에 연결되도록 수정 
- [ ] 로봇 핸드의 pick-up 기능 코드 추가 




#### Reference 
* [ROS](http://wiki.ros.org/Distributions) & [webots](https://cyberbotics.com/) 설치 
* [universal_robots](http://wiki.ros.org/universal_robots) 설치 
* [WEBOTS_HOME](https://cyberbotics.com/doc/guide/tutorial-8-using-ros) 환경변수 설정 
* [UR 로봇 프로젝트 파일을 catkin_ws로 옮기고](https://cyberbotics.com/doc/guide/ure) rviz에 띄우기 
* webots 모델을 ROS로 보내기 위해서는 → webots 에서 물체 모델에 ```extern```
