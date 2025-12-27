from picamera2 import Picamera2
from flask import Flask, Response
import cv2
import time
import os

app = Flask(__name__)

# Kameranın çözünürlüğünü ortam değişkenlerinden al (varsayılan 640x480)

# CAMERA_WIDTH = 3280
# CAMERA_HEIGHT = 2464

CAMERA_WIDTH = 2592
CAMERA_HEIGHT = 1944
CAMERA_FOV_X = 102
CAMERA_FOV_Y = 66
ROI_FOV_X = 45
ROI_FOV_Y = 30

# Kamerayı başlat
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(1)  # Kameranın oturması için kısa bekleme

def calculate_roi():
    ROI_WIDTH = (CAMERA_WIDTH * ROI_FOV_X) // CAMERA_FOV_X
    ROI_HEIGHT = (CAMERA_HEIGHT * ROI_FOV_Y) // CAMERA_FOV_Y
    ROI_X = (CAMERA_WIDTH - ROI_WIDTH) // 2
    ROI_Y = (CAMERA_HEIGHT - ROI_HEIGHT) // 2
    return ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT

def gen_frames():
    while True:
        # Kameradan bir frame al
        frame = picam2.capture_array()

        
        # print(frame.shape)

        # Gerekirse burada OpenCV ile işlem yapabilirsin:
        # örn: LED tespiti, threshold, çizim vs.

        # JPEG'e çevir
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 35])
        if not ret:
            continue
        jpg_bytes = buffer.tobytes()

        # print("Frame işleme süresi:", time.time() - aa)

        # MJPEG olarak frame ve yolla
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
        
def gen_frames_roi():
    my_roi = calculate_roi()
    while True:
        # Kameradan bir frame al
        frame = picam2.capture_array()

        # frame den belli bir roi de çerveve çıkar
        # ROI_WIDTH = 883
        # ROI_HEIGHT = 762
        # ROI_X = (CAMERA_WIDTH - ROI_WIDTH) // 2
        # ROI_Y = (CAMERA_HEIGHT - ROI_HEIGHT) // 2
        # frame_roi = frame[ ROI_Y:ROI_Y + ROI_HEIGHT, ROI_X:ROI_X + ROI_WIDTH]

        frame_roi = frame[ my_roi[1]:my_roi[1] + my_roi[3], my_roi[0]:my_roi[0] + my_roi[2]]

        # Gerekirse burada OpenCV ile işlem yapabilirsin:
        # örn: LED tespiti, threshold, çizim vs.

        # JPEG'e çevir
        ret, buffer_roi = cv2.imencode('.jpg', frame_roi, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ret:
            continue
        jpg_bytes_roi = buffer_roi.tobytes()

        # print("Frame işleme süresi:", time.time() - aa)

        # MJPEG olarak roi frame ve yolla
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes_roi + b'\r\n')

@app.route('/video_roi')
def video_feed_roi():
    return Response(gen_frames_roi(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return (
        "<html><body>"
        "<h1>Raspberry Pi Camera v2 HTTP Stream</h1>"
        "<img src='/video' />"
        "</body></html>"
    )

if __name__ == "__main__":
    # 0.0.0.0: herkes erişebilsin (aynı ağdaki bilgisayarlar)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
