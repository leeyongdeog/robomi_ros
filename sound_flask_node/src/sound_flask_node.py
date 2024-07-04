import rospy
from flask import Flask, request, jsonify
from pydub import AudioSegment
from pydub.playback import play
import threading
import os

app = Flask(__name__)
sound_folder = os.path.join(os.path.dirname(__file__), 'sounds')
is_playing = False
lock = threading.Lock()

@app.route('/play_audio', methods=['POST'])
def play_audio():
    global is_playing
    try:
        audio_filename = request.form['audio_filename']
        audio_path = os.path.join(sound_folder, audio_filename)
        if not os.path.exists(audio_path):
            return jsonify({"error": "File not found"}), 404

        with lock:
            if is_playing:
                rospy.loginfo("Audio is already playing. Skipping new request.")
                return jsonify({"message": "Audio is already playing. Skipping new request."}), 200

            is_playing = True

        audio = AudioSegment.from_file(audio_path)
        play(audio)
        
        with lock:
            is_playing = False

        return jsonify({"message": "Audio played successfully"}), 200
    except Exception as e:
        with lock:
            is_playing = False
        rospy.logerr("Failed to play audio: %s", e)
        return jsonify({"error": str(e)}), 500

def run_flask_app():
    app.run(host='0.0.0.0', port=5000)

def http_audio_player():
    rospy.init_node('sound_flask_node', anonymous=True)
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.start()
    rospy.spin()

if __name__ == "__main__":
    try:
        http_audio_player()
    except rospy.ROSInterruptException:
        pass