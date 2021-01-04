# IPS - Incident Prediction System

## 돌발상황 예측 시스템



- - -
## 1. Introducing Project

<img src =./picture/newsgif.gif width="60%" height="60%">


-대한민국은 골목길이 많고 불법주정차로 가득찬 도로가 만연하다. 운전자가 아무리 조심하더라도 사각지대에서 튀어나오는 여러 위험요소를 모두 피하기란 불가능에 가깝다. 본 프로젝트는 골목길 및 구조물에 가려져 시야가 제한되는경우, 주변 CCTV들이 검출한 위험요소(보행자 및 이륜차 등)정보를 받아와 사각지대에 존재하는 위험요소의 예측 이동경로와 속도를 운전자에게 알려주어 사고를 예방하는 것을 목적으로 한다.   



- - -
## 2. Team

<img src =./picture/건주.jpg width="20%" height="20%">

-김건주
````
* 역할: Kalman Filter 구현/ Collision Risk model 구현/ Thread 관리/ 통합TEST
* Email: kimbatt12@naver.com
````

<img src =./picture/준영.png width="20%" height="20%">

-허준영
````
* 역할:  ROS 통신 구현/ PWM을 이용한 AEB 구현/ 통합TEST/ 보고서 작성
* Email: jass9869@naver.com
````

<img src =./picture/장현.jpg width="20%" height="20%">

-백장현
````
* 역할:  Github 관리/ 단안카메라를 이용한 object의 거리검출/ DarkNet 네트워크 성능비교/ 좌표값 Calibration 구현
* Email: qorwkdgus93@gmail.com
````



- - -
## 3. Project Abstracts

<img src =./picture/프로젝트소개_v2.png width="50%" height="50%"> <img src =./picture/PRJ_TestBed.PNG width="35%" height="35%">

PRJ_TestBed.PNG
In Korea, roads with many alleys and full of illegal parking are rampant. No matter how careful the driver is, it is almost impossible to avoid all the risk factors that pop out of the blind spot. The purpose of this project is to prevent accidents by informing the driver of the predicted path and speed of the risk factors in the blind spot by receiving information on the risk factors (walker, bicycle) detected by the CCTVs in the surrounding area when the visibility is restricted due to being hidden in alleys and structures. 



- - -
## 4. Demonstration Video


   <img src =./picture/viewgif.gif width="60%" height="60%">



- - -
## 5. Hardware Structure & Software Diagram


   ### A. Hardware Structure
   <img src =https://user-images.githubusercontent.com/69332997/103496738-a374fc00-4e82-11eb-9f26-6c76677e2de0.jpg width="50%" height="40%">



   ### B. Sequence Diagram
   <img src =https://user-images.githubusercontent.com/69332997/103496741-a53ebf80-4e82-11eb-8a7e-c2278ed09e8a.JPG width="50%" height="40%">
   
   
   
   ### C. Software Algorithm
   <img src =./picture/시퀀스다이어그램_v3.png width="50%" height="40%">
   
   

- - -
## 6. Software Modularity


   ### A. Calibration Pixel to Global

   ### B. Transform Geometry
   
   ### C. Calculate Crash Risk from Kalman Filter



- - -
## 7. Development Settings

### Development Languages
````
* C/C++
* Python
* Cuda
* CMake
````

### Development Environment
````
* Ubuntu 18.04 LTS
* OpenCV 3.4.0
* Cuda 10.0
* DarkNet Yolo3
* NVIDIA Xavier AGX
* NVIDIA Jetson Nano
* ROS Melodic
* Logitec HD USB Camera (1080p
````
   
### Libraries
````
* DarkNet Yolo3 / Yolo3 tiny / Yolo4 / Yolo4 tiny
* Python
* OpenCV
* ROS
````

  
<!--
## 4. 기타
-->
