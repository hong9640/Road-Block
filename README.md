# 도주 차량 방지 시스템 - Road Block

도주 차량을 막는 경찰관 분들의 **안전을 보장해드리기 위해** 경찰차를 **자율 주행**하여 조직적으로 도주 차량을 막는 시스템 


[📓 기능 명세서](https://www.notion.so/262546b4de0280d3a988ea11e464ff57) <br> 
[🗒️ API 명세서](https://magnificent-lighter-489.notion.site/API-262546b4de0280779251d4816ce29941?pvs=74)<br>
[🎨 Figma Lo-Fi 디자인](https://www.figma.com/design/654U42E22pFG5uiDIWTAFy/%ED%8A%B9%ED%99%94-%ED%8E%98%EC%9D%B4%EC%A7%80-%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83-%EC%B4%88%EC%95%88?node-id=0-1&t=uAdzKVNnHfYwWz8f-1)



## 🖐️ 프로젝트 소개

![alt text](./docs/images/슬라이드1.PNG) 


현재 대한민국에서 도주 차량 사고는 **하루 평균 25건** 정도 발생하는 통계가 있습니다. 
<br><br>

![alt text](./docs/images/슬라이드2.PNG) 


도주 차량 사고는 **챠량 충돌, 인명 피해**를 주었다는 조사 결과가 있었습니다. 지금까지는 경미한 **경찰관 부상** 다행히 없었지만, 이를 간과하기에는 경찰관 분들의 노고와 위험이 없다고는 할 수 없습니다.
<br><br>

![alt text](./docs/images/슬라이드3.PNG) 

이러한 문제점을 바탕으로 **경찰차를 자율주행** 시키고, 도주차량의 위치를 바탕으로 **군집 제어 기능**을 합쳐, 도주 차량을 직접 쫓을 필요 없는 **Road Block** 시스템을 개밯하였습니다.
<br><br>

![alt text](./docs/images/슬라이드4.PNG) 
![alt text](./docs/images/슬라이드5.PNG) 

## ⚙️ 기술 스택

### 시뮬레이터
- Morai Simulator, ROS1, Ubuntu 20.04

![alt text](./docs/images/슬라이드6.PNG) 
![alt text](./docs/images/슬라이드7.PNG) 

### 웹 관제 시스템
- React, FastAPI, MySQL, AWS, WebSocket

![alt text](./docs/images/슬라이드8.PNG) 
![alt text](./docs/images/슬라이드9.PNG)


## 🧱 프로젝트 구조도
![alt text](./docs/images/structure.png)

## ➡️ 프로젝트 순서도
![alt text](./docs/images/workflow.png)

## 💽 DB ERD

![alt text](./docs/images/erd.png)