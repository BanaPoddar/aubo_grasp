from flask import Flask, jsonify, Response, render_template,request
from flask_socketio import SocketIO
import threading
import cv2
import mediapipe as mp
import numpy as np
import logging
from std_msgs.msg import Float64MultiArray
import rospy
import time
import math
import base64
import cv_bridge
import json
from aubo_grasp.msg import Images
from aubo_grasp.msg import graspMessage
from flask_cors import CORS
from yolo import get_all_objects_info

#初始化mediapipe模型 人体骨骼和手势模型
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
#初始化Flask应用
app = Flask(__name__)
CORS(app)  # 初始化CORS，允许所有来源的请求
#设置日志级别
app.logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
app.logger.addHandler(console_handler)
#定义全局变量存储摄像头图像
color_img = []
bridge = cv_bridge.CvBridge()
#定义全局变量存储抓取信息
grasp_pub = rospy.Publisher('grasp', graspMessage, queue_size=10)
# 定义全局变量存储日志
logs = []
#定义全局变量存储同步模式
in_sync_mode = False
#定义全局变量存储摄像头图像
camera = cv2.VideoCapture(6)

# 添加日志条目
def add_log(log):
    logs.append(log)

#求解关节角度
def calculate_angle(a, b, c):
    a = np.array(a)  # First
    b = np.array(b)  # Mid
    c = np.array(c)  # End
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    if angle > 180.0:
        angle = 360 - angle
    return angle

#求解二维向量的角度
def vector_2d_angle(v1, v2):
    v1_x, v1_y = v1
    v2_x, v2_y = v2
    try:
        angle_ = math.degrees(math.acos((v1_x * v2_x + v1_y * v2_y) / (((v1_x**2 + v1_y**2)**0.5) * ((v2_x**2 + v2_y**2)**0.5))))
    except:
        angle_ = 65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_

#处理姿态和手势
def capture_and_process(pub):
    # Your existing capture_and_process function with slight modifications to use Flask and threading
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose, mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
        last_publish_time = time.time()
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret or frame is None:
                continue  # Skip processing if frame is empty
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results_pose = pose.process(image)
            results_hands = hands.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            try:
                landmarks = results_pose.pose_landmarks.landmark

                shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                            landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
                elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                        landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
                wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                        landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
                left_hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
                            landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
                angle1 = calculate_angle(shoulder, elbow, wrist)
                angle2 = calculate_angle(left_hip, shoulder, elbow)
                # Hand gesture recognition
                if results_hands.multi_hand_landmarks:
                    for hand_landmarks in results_hands.multi_hand_landmarks:
                        hand_local = []
                        for i in range(21):
                            x = hand_landmarks.landmark[i].x * image.shape[1]
                            y = hand_landmarks.landmark[i].y * image.shape[0]
                            hand_local.append((x, y))
                        if hand_local:
                            thumb_tip = hand_local[4]
                            index_tip = hand_local[8]
                            pinky_tip = hand_local[20]
                            thumb_index_angle = vector_2d_angle(
                                (thumb_tip[0] - hand_local[0][0], thumb_tip[1] - hand_local[0][1]),
                                (index_tip[0] - hand_local[0][0], index_tip[1] - hand_local[0][1]))
                            thumb_pinky_angle = vector_2d_angle(
                                (thumb_tip[0] - hand_local[0][0], thumb_tip[1] - hand_local[0][1]),
                                (pinky_tip[0] - hand_local[0][0], pinky_tip[1] - hand_local[0][1]))   
                            cv2.putText(image, f'Thumb-Index Angle: {thumb_index_angle:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            cv2.putText(image, f'Thumb-Pinky Angle: {thumb_pinky_angle:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            current_time = time.time()
                            #每隔0.5s发布一次手臂角度
                            if current_time - last_publish_time >= 0.5:
                                #打印global变量
                                #print(in_sync_mode)
                                angles = Float64MultiArray()
                                angles.data = [angle1, angle2, thumb_index_angle, thumb_pinky_angle,int(in_sync_mode)]
                                pub.publish(angles)
                                last_publish_time = current_time
                cv2.putText(image, str(angle1),
                            tuple(np.multiply(elbow, [640, 480]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                            )
            except:
                pass
            mp_drawing.draw_landmarks(image, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                    mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                    mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                    )
            if results_hands.multi_hand_landmarks:
                for hand_landmarks in results_hands.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            cv2.imshow('Mediapipe Feed', image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
    camera.release()
    cv2.destroyAllWindows()

@app.route('/sync/start_sync_pose', methods=['POST'])
def start_sync_pose_api():
    global in_sync_mode
    in_sync_mode = True
    return jsonify({'message': 'Sync Pose Node Started', 'in_sync_mode': int(in_sync_mode)})

@app.route('/sync/stop_sync_pose', methods=['POST'])
def stop_sync_pose_api():
    global in_sync_mode
    in_sync_mode = False
    return jsonify({'message': 'Sync Pose Node Stopped', 'in_sync_mode': int(in_sync_mode)})

@app.route('/grasp/get_grasp_images', methods=['GET'])
def get_grasp_images():
    global color_img
    # encoded_color = base64.b64encode(color_img).decode('utf-8')
    # encoded_content = {"img_color": color_img}
    return jsonify({"code": 200, "msg": "success", "data": color_img.tolist()})

# 开始随机抓取
@app.route('/grasp/startRandomGrasp', methods=['POST'])
def random_grasp():
    start_time = time.time()
    # 创建一个 grasp_message 对象并设置其属性
    grasp_data = graspMessage()
    grasp_data.id = 1
    grasp_data.flag = True
    # 发布 grasp 数据
    grasp_pub.publish(grasp_data)
    # # 暂停代码的执行，以确保循环按照指定的频率进行
    add_log({
        "timestamp": start_time,
        "action": "开始随机抓取",
        "status": "成功"
    })
    return jsonify({"code": 200, "msg": "请求成功，随机抓取已开始！"})

# 开始抓取指定物品接口
@app.route('/grasp/startItemGrasp', methods=['POST'])
def item_grasp():
    item = request.args.get("item")
    print("item:", item)   
    start_time = time.time()
    # 创建一个 grasp_message 对象并设置其属性
    grasp_data = graspMessage()
    grasp_data.id = 2
    grasp_data.flag = True
    if item == 'box':
        grasp_data.type = 'box'
    elif item == 'bottle':
        grasp_data.type = 'bottle'
    elif item == 'can':
        grasp_data.type = 'can'
    elif item == 'tape':
        grasp_data.type = 'tape'
    elif item == 'hand':
        grasp_data.type = 'hand'
    # 发布 grasp 数据
    grasp_pub.publish(grasp_data)
    # 暂停代码的执行，以确保循环按照指定的频率进行
    add_log({
        "timestamp": start_time,
        "action": "开始抓取",
        "item": item,
        "color": "",
        "status": "成功"
    })
    return jsonify({"code": 200, "msg": f"请求成功，已开始抓取{item}！"})

# 获取日志
@app.route('/grasp/log', methods=['GET'])
def get_logs():
    return jsonify(logs)

# 获取物品信息
@app.route('/grasp/getRecognizedItems', methods=['GET'])
def get_objects_info():
    global color_img
    img_color = color_img.astype(np.float32)
    cv2.imwrite("src/aubo_grasp/resources/color.png",img_color)
    # 使用 YOLO 检测物体并获取信息
    objects_info = get_all_objects_info("src/aubo_grasp/resources/color.png")
    # 转换为 JSON 格式的字符串
    objects_info_json = json.dumps(objects_info)
    return jsonify({"code": 200, "msg": "获取物品信息成功！", "data": objects_info_json})

# 获取姿态摄像头视频流
@app.route('/camera/pose', methods=['GET'])
def video_feed_pose_camera():
    def generatePose():
        while True:
            # time.sleep(0.0)
            ret, frame = camera.read()
            if not ret:
                break
            _, jpeg = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    return Response(generatePose(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 获取物品摄像头视频流
@app.route('/camera/depth', methods=['GET'])
def video_feed_depth_camera():
    def generateDepth():
        while True:
            global color_img
            # time.sleep(0.0)
            ret, buffer = cv2.imencode('.jpg', color_img)
            color_img = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type:image/jpeg\r\n\r\n' + color_img + b'\r\n')
    return Response(generateDepth(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_camera_thread():
    pub = rospy.Publisher('arm_synchronization', Float64MultiArray, queue_size=10)
    capture_and_process(pub)

def images_callback(msg):
    # Your existing images_callback function with slight modifications to use Flask and threading
    global color_img
    global bridge
    color_img_msg = msg.color_image
    color_img = bridge.imgmsg_to_cv2(color_img_msg, "bgr8")

if __name__ == '__main__':
    rospy.init_node('pose_hand_calculate', anonymous=True)
    images_sub=rospy.Subscriber("/camera/aligned_images", Images, images_callback)
    # Start the camera processing in a separate thread
    threading.Thread(target=start_camera_thread, daemon=True).start()
    # Run Flask server
    #socketio.run(sync_mode_app, debug=True)
    app.run(debug=True, use_reloader=False)  # use_reloader=False to prevent the thread from being started twice