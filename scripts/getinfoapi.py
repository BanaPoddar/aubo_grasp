import requests
import json
import base64
import numpy as np
import os
from PIL import Image
def get_objects_info():
    url = "http://127.0.0.1:5000/api/get_objects_info"

    headers = {"Content-Type": "application/json"}
    response = requests.post(url, headers=headers)

    if response.status_code == 200:
        content=json.loads(response.content)
        
    else:
        print("获取物体信息失败，状态码：", response.status_code)
        return None

