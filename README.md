# webots-ros-melodic-project

#### Install Docker and Nvidia-docker 
* [docker.com](https://docs.docker.com/engine/install/ubuntu/)으로 가서 자신의 컴퓨터 환경에 맞는 Docker Engine을 설치한다 
* 루트(root) 권한 설정; ```sudo``` 없이 도커 사용하기 
  ``` bash
  ~$ sudo usermod -aG docker $USER #현재 접속중인 사용자에게 권한주기 
  ```
* [docker.com/compose](https://docs.docker.com/compose/install/)에서 Docker Compose를 설치한다 
* [nvidia.com](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)를 참고해서 Nvidia-docker를 설치한다 
  * 단, 사전에 로컬 PC에서 ```watch nvidia-smi``` 명령어가 가능해야한다. 
  * 위의 명령어가 안 된다면 본인의 환경에 맞게 GPU 드라이버를 


#### Pull and Install the docker image 
도커 설치가 완료됐다면 다음과 같이 명령어를 실행한다: 
```bash 
~$ sudo chmod a+x run_webots_ros.bash

~$ ./run_webots_ros.bash
```
* 위의 명령어가 실행되면 


- [ ] ```--net```, ```--ipc``` 붙이니까 rviz가 안되네... 이부분 해결해필요 









#### Reference 
* [ROS](http://wiki.ros.org/Distributions) & [webots](https://cyberbotics.com/) 설치 
* [universal_robots](http://wiki.ros.org/universal_robots) 설치 
* [WEBOTS_HOME](https://cyberbotics.com/doc/guide/tutorial-8-using-ros) 환경변수 설정 
* [UR 로봇 프로젝트 파일을 catkin_ws로 옮기고](https://cyberbotics.com/doc/guide/ure) rviz에 띄우기 
* webots 모델을 ROS로 보내기 위해서는 → webots 에서 물체 모델에 ```extern```
