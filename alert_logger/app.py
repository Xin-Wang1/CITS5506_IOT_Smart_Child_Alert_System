from flask import Flask, request, jsonify
from datetime import datetime
import csv
import os

app = Flask(__name__)

# 日志文件名
LOG_FILE = 'alert_log.csv'

# 初始化 CSV 文件（如果不存在）
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'device', 'alert_count'])

@app.route('/upload', methods=['POST'])
def upload():
    device = request.form.get('device', 'unknown')
    count = request.form.get('count', '0')
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # 写入 CSV
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, device, count])

    return jsonify({'status': 'success', 'message': 'Record saved'}), 200

@app.route('/log', methods=['GET'])
def download_log():
    try:
        with open(LOG_FILE, 'r') as f:
            return f.read(), 200, {'Content-Type': 'text/plain; charset=utf-8'}
    except Exception as e:
        return f"Error reading log: {str(e)}", 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
