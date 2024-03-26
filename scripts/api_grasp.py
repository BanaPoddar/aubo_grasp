import json

import cv2
import numpy as np
import base64
import logging
import time
from flask import Flask, request, jsonify
import rospy
from aubo_grasp.msg import graspMessage
from flask_cors import CORS

app = Flask(__name__)
CORS(app)  # 初始化CORS，允许所有来源的请求
app.logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
app.logger.addHandler(console_handler)

# 定义全局变量存储日志
logs = []

# 添加日志条目
def add_log(log):
    logs.append(log)

# 开始随机抓取
@app.route('/grasp/startRandomGrasp', methods=['POST'])
def random_grasp():
    start_time = time.time()
    grasp_pub = rospy.Publisher('grasp', graspMessage, queue_size=10)
    # rate = rospy.Rate(10)  # 发布频率为10Hz
    # while not rospy.is_shutdown():
    # 创建一个 grasp_message 对象并设置其属性
    grasp_data = graspMessage()
    grasp_data.id = 1
    grasp_data.flag = True
    # 发布 grasp 数据
    grasp_pub.publish(grasp_data)
    # # 暂停代码的执行，以确保循环按照指定的频率进行
    # rate.sleep()
    app.logger.info( {
        "timestamp": start_time,
        "action": "开始随机抓取",
        "status": "成功"
    })
    add_log({
        "timestamp": start_time,
        "action": "开始随机抓取",
        "status": "成功"
    })
    return jsonify({"code": 200, "msg": "请求成功，随机抓取已开始！"})


# 开始抓取指定物品接口
@app.route('/api/item_grasp', methods=['POST'])
def item_grasp():
    item = request.args.get("item")
    start_time = time.time()
    grasp_pub = rospy.Publisher('grasp', graspMessage, queue_size=10)
    # rate = rospy.Rate(10)  # 发布频率为10Hz
    # while not rospy.is_shutdown():
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
    # rate.sleep()
    app.logger.info({
        "timestamp": start_time,
        "action": "开始抓取",
        "item": item,
        "status": "成功"
    })
    add_log({
        "timestamp": start_time,
        "action": "开始抓取",
        "item": item,
        "status": "成功"
    })
    return jsonify({"code": 200, "msg": f"请求成功，已开始抓取{item}！"})

# 开始颜色识别抓取
@app.route('/grasp/startColorGrasp', methods=['POST'])
def startColorGrasp():
    color = request.args.get("color")
    start_time = time.time()
    grasp_pub = rospy.Publisher('grasp', graspMessage, queue_size=10)
    # rate = rospy.Rate(10)  # 发布频率为10Hz
    # while not rospy.is_shutdown():
        # 创建一个 grasp_message 对象并设置其属性
    grasp_data = graspMessage()
    grasp_data.id = 2
    grasp_data.flag = True
    if color == 'red':
        grasp_data.color = 'red'
    elif color == 'blue':
        grasp_data.color = 'blue'
    elif color == 'green':
        grasp_data.color = 'green'
    elif color == 'black':
        grasp_data.color = 'black'
    elif color == 'white':
        grasp_data.color = 'white'
    elif color == 'red':
        grasp_data.color = 'red'
    elif color == 'orange':
        grasp_data.color = 'orange'
    elif color == 'yellow':
        grasp_data.color = 'yellow'
    elif color == 'purple':
        grasp_data.color = 'purple'

    # 发布 grasp 数据
    grasp_pub.publish(grasp_data)
    # 暂停代码的执行，以确保循环按照指定的频率进行
    # rate.sleep()
    app.logger.info({
        "timestamp": start_time,
        "action": "开始抓取",
        "color": color,
        "status": "成功"
    })
    add_log({
        "timestamp": start_time,
        "action": "开始抓取",
        "color": color,
        "status": "成功"
    })
    return jsonify({"code": 200, "msg": "请求成功，颜色识别抓取已开始！"})

# 获取日志
@app.route('/grasp/log', methods=['GET'])
def get_logs():
    return jsonify(logs)

if __name__ == '__main__':
    rospy.init_node('grasp_publisher', anonymous=True)
    app.run(host='0.0.0.0', debug=True)