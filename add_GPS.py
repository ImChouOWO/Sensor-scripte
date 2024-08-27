import cv2
import serial
import pynmea2

# 設定影片路徑
input_video_path = 'video/GPS demo.mp4'
output_video_path = 'video/output_video_with_gps.mp4'
# 使用 Mac 上的 GPS 接收器設備路徑
ser = serial.Serial('/dev/tty.usbmodem1201', baudrate=9600, timeout=1)

# 讀取影片
cap = cv2.VideoCapture(input_video_path)

# 取得影片幀率與尺寸
frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# 定義影片編碼器與輸出影片檔案
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (width, height))

gps_info = ""
font_scale = 1.5
thickness = 2

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 從GPS獲取最新資料
        try:
            line = ser.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                gps_info = f"Time: {msg.timestamp}  Lat: {msg.latitude}  Lon: {msg.longitude}"
        except Exception as e:
            gps_info = f"GPS Error: {e}"

        # 計算文字與底框參數
        text_size = cv2.getTextSize(gps_info, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]

        # 計算底框的座標
        x, y = 10, height - 20
        text_w, text_h = text_size
        box_coords = ((x, y - text_h - 10), (x + text_w + 10, y + 10))

        # 繪製底框 (黑色背景)
        cv2.rectangle(frame, box_coords[0], box_coords[1], (0, 0, 0), cv2.FILLED)

        # 繪製文字 (白色文字)
        cv2.putText(frame, gps_info, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

        # 寫入幀至新影片
        out.write(frame)

except KeyboardInterrupt:
    print("停止讀取")
finally:
    # 釋放資源
    cap.release()
    out.release()
    ser.close()

print("影片已成功合成並儲存為", output_video_path)
