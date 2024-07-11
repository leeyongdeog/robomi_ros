#!/usr/bin/env python3
from flask import Flask, request, jsonify
import rospy
from geometry_msgs.msg import Twist
from pydub import AudioSegment
import simpleaudio as sa
import os

app = Flask(__name__)

# ROS 초기화
rospy.init_node('flask_control_node', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# 오디오 파일이 저장된 경로
AUDIO_FOLDER = "/home/pi/catkin_ws/src/robomi_ros-main_0708/sound_flask_node/src/sounds"

def play_audio(file_path):
    # 오디오 파일을 로드
    audio = AudioSegment.from_file(file_path)
    # 오디오 파일을 재생
    play_obj = sa.play_buffer(
        audio.raw_data,
        num_channels=audio.channels,
        bytes_per_sample=audio.sample_width,
        sample_rate=audio.frame_rate
    )
    # 재생이 완료될 때까지 기다립니다
    play_obj.wait_done()

@app.route('/start', methods=['GET'])
def start_robot():
    twist = Twist()
    twist.linear.x = 0.3  # 전진 속도 설정 (값은 조정 가능)
    twist.angular.z = 0.0  # 회전 속도 설정 (0으로 고정)
    pub.publish(twist)
    return "Robot moving forward"

@app.route('/stop', methods=['GET'])
def stop_robot():
    twist = Twist()
    twist.linear.x = 0.0  # 전진 속도 0으로 설정 (정지)
    twist.angular.z = 0.0  # 회전 속도 0으로 설정 (정지)
    pub.publish(twist)
    return "Robot stopped"

@app.route('/play_audio', methods=['GET'])
def play_audio_endpoint():
    file_name = request.args.get('file_name')
    if not file_name:
        return jsonify({"error": "file_name parameter is required"}), 400

    file_path = os.path.join(AUDIO_FOLDER, file_name)
    
    if not os.path.exists(file_path):
        return jsonify({"error": "File not found"}), 404

    try:
        play_audio(file_path)
        return jsonify({"status": "Playing audio"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # 통합된 애플리케이션에서 포트 번호 5000 사용
