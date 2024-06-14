### 레파지터리 이용 안내

로보미 ROS 관련 작업 기록

---

### 디렉토리 사용 안내

* robomi_bot 디렉토리에 구현 위치가 로봇 우선 실행이 가능한 코드들 위주로 준비 및 기록

* robomi_svr 디렉토리에 구현 위치가 서버 우선 실행인 경우의 코드를 위주로 준비 및 기록

※ 위 디렉토리의 위치는 리눅스 파일 시스템 루트에 위치 (생성 방법 아래에)

```
sudo mkdir /robomi_bot
sudo chown $USER /robomi_bot
ls -dl /robomi_bot
```
```
sudo mkdir /robomi_svr
sudo chown $USER /robomi_svr
ls -dl /robomi_svr
```

---

### 작업 내용 개요

1. 역량 PR 근거 자료 및 결과물 기록 위주의 작성 방식 지향

---

### 환경 구성 및 이용 방식 기록 (시간 경과 후에도 유효 내용)

※  최종 결과물 대상은 홈 디렉토리 밑에 robomi_bot 와 robomi_svr 작업 디렉토리에 생성

※  각 해당 결과물의 실행이나 주의 사항 관련은 각 디렉토리의 README.md 파일에 작성

※  ROS 관련 포함 시험 환경 구성 방법은 카페 게시물 https://cafe.naver.com/yhrobotics/13471 

---

#### usb cam 2대를 rpi 연결 작동 요령 (결과물 실행 위주의 사례)

```
roscore   # ROS 서버 역할의 pc ip123에서 실행
```

```
roslaunch usb_cam usb_cam_0-test.launch   # ROS IoT 역할의 rpi ip128에서 실행
```

```
roslaunch usb_cam usb_cam_2-test.launch   # ROS IoT 역할의 rpi ip128에서 실행
```

※ 게시물 https://cafe.naver.com/yhrobotics/13491 및 그 이전 관련 내용 참조

---
---

### 환경 구성 및 이용 방식 기록 (시간 경과 후에는 무효 내용)


##### 실습 환경 (ip 및 작업 계정 정보)

※ 운영체제 윈도 사용 기록: pc 최1 ip끝자리 223, 이1: ip끝자리 122, 254, 147 사용 중

1. pc Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu2004@192.168.123.122 (이1)

1. pc Ubuntu 20.04 ROS1 noetic 접속: ssh yh6@192.168.123.123 (박1)

1. pc Ubuntu 20.04 ROS1 noetic 접속: ssh user@192.168.123.125 (최1)

1. pc Ubuntu 18.04 ROS1 melodic 접속: ssh tk@192.168.123.127 (김1)

---

1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.121 (ref. 형6)

1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.124

1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.129 (이1)

1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.19

1. rpi 3B+ Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.128 (최1)

1. rpi 3B+ Rasbian buster 18.04 ROS1 noetic 접속: ssh pi@192.168.123.126 (김1)

※  각 rpi 비번은 '연희6' (1차 삭제 예정 내용)

---
