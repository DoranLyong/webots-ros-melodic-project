# webots-ros-melodic-project


#### Pull and Install the docker image 
```bash 
~$ sudo chmod a+x run_webots_ros.bash

~$ ./run_webots_ros.bash
```

- [] ```--net```, ```--ipc``` 붙이니까 rviz가 안되네 
- [] 이부분 해결해야함 









#### Reference 
* [ROS](http://wiki.ros.org/Distributions) & [webots](https://cyberbotics.com/) 설치 
* [universal_robots](http://wiki.ros.org/universal_robots) 설치 
* [WEBOTS_HOME](https://cyberbotics.com/doc/guide/tutorial-8-using-ros) 환경변수 설정 
* [UR 로봇 프로젝트 파일을 catkin_ws로 옮기고](https://cyberbotics.com/doc/guide/ure) rviz에 띄우기 
* webots 모델을 ROS로 보내기 위해서는 → webots 에서 물체 모델에 ```extern```
