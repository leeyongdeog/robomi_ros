# robomi_ros

** 여기는 ROS 관련 개발을 협업하는 레포지토리 입니다. **

--

## 개요

1. 역량 PR 근거 자료 및 결과물 위주 작성 방식 지향

---

## usb cam 2대를 rpi 연결 작동 요령 (결과물 실행 위주)

```
roscore   # pc ip123에서 실행
```

```
roslaunch usb_cam usb_cam_0-test.launch   # rpi ip128에서 실행
```

```
roslaunch usb_cam usb_cam_2-test.launch   # rpi ip128에서 실행
```

※ 게시물 https://cafe.naver.com/yhrobotics/13491 및 그 이전 관련 내용 참조

---

## 환경 (ip 및 계정 정보 )

※ 참고: pc 최1 ip끝자리 223, 이1: ip끝자리 122, 254, 147 사용 중

1. pc Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu2004@192.168.123.122
1. pc Ubuntu 20.04 ROS1 noetic 접속: ssh yh6@192.168.123.123
1. pc Ubuntu 18.04 ROS1 melodic 접속: ssh tk@192.168.123.127

1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.121 (ref. 형6)
1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.124
1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.129 (이1)
1. rpi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.19
1. rpi 3B+ Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.128 (최1)
1. rpi 3B+ Rasbian buster 18.04 ROS1 noetic 접속: ssh pi@192.168.123.126 (김1)

※  각 rpi 계정의 홈 밑에 robomi_bot 디렉토리에 실행 결과물 생성

---


