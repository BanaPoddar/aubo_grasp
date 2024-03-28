from flask import Flask, request, jsonify
from aip import AipSpeech
import base64
from api_flask_start import app

# Baidu API credentials
APP_ID = '45284568'
API_KEY = 'kyxAYXanZ9212FLpy4u3QshO'
SECRET_KEY = 'WmmcGKNr5jtpIN4XFyVQbIEhKC888qsc'

# Initialize Baidu AipSpeech client
client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

@app.route('/voice/convert', methods=['POST'])
def recognize_audio():
    # 获取 POST 请求中的 base64 编码的音频数据
    audio_base64 = request.json.get('audio_base64')
    if not audio_base64:
        return jsonify({'error': 'No audio data provided'}), 400
    try:
        # 将 base64 编码的音频数据解码为二进制数据
        audio_data = base64.b64decode(audio_base64)
        # 调用百度语音识别 API 进行识别
        result = client.asr(audio_data, 'pcm', 16000, {'dev_pid': 1537})
        # 提取识别结果
        if 'result' in result:
            recognized_text = result['result'][0]
            return jsonify({'recognized_text': recognized_text}), 200
        else:
            return jsonify({'error': 'Failed to recognize audio'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500