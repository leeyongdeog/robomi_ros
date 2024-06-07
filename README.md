# robomi_ros

** 여기는 ROS 관련 개발을 협업하는 레포지토리 입니다. **

--

## 개요

1. 역량 PR 근거 자료 및 결과물 위주 작성 방식 지향

---

## 환경

※ 참고: pc 최1 ip끝자리 223, 이1: ip끝자리 254, 147 사용 중

pc Ubuntu 20.04 ROS1 noetic 접속: ssh yh6@192.168.123.123
pc Ubuntu 18.04 ROS1 melodic 접속: ssh tk@192.168.123.127

pi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.124
pi 4B Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.129
pi 3B+ Ubuntu 20.04 ROS1 noetic 접속: ssh ubuntu@192.168.123.128
pi 3B+ Rasbian buster 18.04 ROS1 noetic 접속: ssh pi@192.168.123.126

---

## usb cam 2대를 pi 연결 작동 요령 (결과물 실행 위주)

```
roscore   # pc ip123에서 실행
```

```
roslaunch usb_cam usb_cam_0-test.launch   # pi ip128에서 실행
```

```
roslaunch usb_cam usb_cam_2-test.launch   # pi ip128에서 실행
```

※ 게시물 https://cafe.naver.com/yhrobotics/13491 및 그 이전 관련 내용 참조

---

