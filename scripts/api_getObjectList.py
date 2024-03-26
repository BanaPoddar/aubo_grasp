import cv2
import numpy as np
import logging
import base64
from flask import Flask, request, jsonify
from yolo import get_all_objects_info
from flask_cors import CORS
import json
from aubo_grasp.msg import Images
import rospy
import cv_bridge

color_img_msg = None
bridge = cv_bridge.CvBridge()

app = Flask(__name__)
CORS(app)

app.logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
app.logger.addHandler(console_handler)


@app.route('/api/get_objects_info', methods=['POST'])
def get_objects_info():
    global color_img_msg
    img_color = bridge.imgmsg_to_cv2(color_img_msg, "bgr8").astype(np.float32)
    cv2.imwrite("src/aubo_grasp/resources/color.png",img_color)
    # 使用 YOLO 检测物体并获取信息
    objects_info = get_all_objects_info("src/aubo_grasp/resources/color.png")
    # 转换为 JSON 格式的字符串
    objects_info_json = json.dumps(objects_info)
    return jsonify({"code": 200, "msg": "获取物品信息成功！", "data": objects_info_json})

def images_callback(msg):
    # Your existing images_callback function with slight modifications to use Flask and threading
    global color_img_msg
    color_img_msg = msg.color_image

if __name__ == '__main__':
    rospy.init_node('getObjectList', anonymous=True)
    images_sub=rospy.Subscriber("/camera/aligned_images", Images, images_callback)
    app.run(host='127.0.0.1', debug=True)
