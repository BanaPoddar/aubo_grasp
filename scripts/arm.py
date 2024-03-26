from flask import Flask, render_template, Response
import cv2
import rospy
import numpy as np
from aubo_grasp.msg import Images
import cv_bridge
 
app = Flask(__name__)
 
#camera = cv2.VideoCapture(7)

bridge = cv_bridge.CvBridge()
color_img_msg = None
 
 
# def cctv_live():
#     while True:
#         success, frame = camera.read()
#         if not success:
#             break
#         else:
#             ret, buffer = cv2.imencode('.jpg', frame)
#             frame = buffer.tobytes()
#         yield (b'--frame\r\n'
#                b'Content-Type:image/jpeg\r\n\r\n' + frame + b'\r\n')
        

def graspnet_camera():
    while True:
        global color_img_msg
        color_img = bridge.imgmsg_to_cv2(color_img_msg, "bgr8")
        ret, buffer = cv2.imencode('.jpg', color_img)
        color_img = buffer.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type:image/jpeg\r\n\r\n' + color_img + b'\r\n')
 
 
@app.route('/')
def index():
    return render_template('cctv.html')
 
 
# @app.route('/video')
# def video():
#     return Response(cctv_live(), mimetype='multipart/x-mixed-replace;boundary=frame')

 
@app.route('/video')
def video():
    return Response(graspnet_camera(), mimetype='multipart/x-mixed-replace;boundary=frame')

def images_callback(msg):
    # Your existing images_callback function with slight modifications to use Flask and threading
    global color_img_msg
    global bridge
    color_img_msg = msg.color_image

if __name__ == "__main__":
    rospy.init_node('camera_subscriber', anonymous=True)
    images_sub=rospy.Subscriber("/camera/aligned_images", Images, images_callback)
    app.run(debug=True)