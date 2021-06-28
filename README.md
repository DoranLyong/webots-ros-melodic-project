# webots-ros-melodic-project

#### Install Docker and Nvidia-docker 
* [docker.com](https://docs.docker.com/engine/install/ubuntu/)으로 가서 자신의 컴퓨터 환경에 맞는 Docker Engine을 설치한다 
* 루트(root) 권한 설정; ```sudo``` 없이 도커 사용하기 
  ``` bash
  ~$ sudo usermod -aG docker $USER #현재 접속중인 사용자에게 권한주기 
  ```


#### Pull and Install the docker image 
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
