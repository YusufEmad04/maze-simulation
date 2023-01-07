import threading

import MazeRobot
from flask import Flask, Response
import cv2

def server(robot: MazeRobot):
    app = Flask(__name__)

    def generate_frames():
        while True:
            lock = threading.Lock()
            lock.acquire()

            image = robot.left_image
            image = cv2.resize(image, (0, 0), fx=5, fy=5)

            frame = cv2.imencode('.jpg', image)[1].tobytes()
            lock.release()
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def generate_frames2():
        while True:
            lock = threading.Lock()
            lock.acquire()

            image = robot.right_image
            image = cv2.resize(image, (0, 0), fx=5, fy=5)

            frame = cv2.imencode('.jpg', image)[1].tobytes()
            lock.release()
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


    @app.route('/video_feed')
    def video():
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/video_feed2')
    def video2():
        return Response(generate_frames2(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/<command>')
    def command(command):
        lock = threading.Lock()
        lock.acquire()
        robot.current_status = str(command)
        lock.release()
        return "OK"

    app.run(host='0.0.0.0', port=5000)


