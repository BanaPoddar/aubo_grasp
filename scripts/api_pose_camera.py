import os
from flask import Flask, render_template, Response
import cv2
 
app = Flask(__name__)
 
camera = cv2.VideoCapture(7) # 笔记本自带摄像头，支持macbook
@app.route('/api/video_feed')
def video_feed():
    def generate():
        while True:
            # time.sleep(0.0)
            ret, frame = camera.read()
            if not ret:
                break
            _, jpeg = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(debug=True)
