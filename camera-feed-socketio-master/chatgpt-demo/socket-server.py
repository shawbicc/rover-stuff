# server.py
import cv2
import base64
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app)

cameras = [cv2.VideoCapture(i) for i in range(3)]

def capture_camera(camera_index):
    cap = cameras[camera_index]
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        _, buffer = cv2.imencode('.jpg', frame)
        frame_b64 = base64.b64encode(buffer).decode('utf-8')
        socketio.emit(f'camera_{camera_index}_frame', {'data': frame_b64}, namespace='/video')
    cap.release()

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect', namespace='/video')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect', namespace='/video')
def handle_disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    for i in range(len(cameras)):
        thread = threading.Thread(target=capture_camera, args=(i,))
        thread.daemon = True
        thread.start()
    socketio.run(app, host='0.0.0.0', port=5000)
