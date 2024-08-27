import cv2
import serial
import pynmea2
import time
import os
import threading

class GPS:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.baudrate = baudrate
        self.timeout = timeout
        self.port = port
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, bytesize=serial.EIGHTBITS)
        self.gps_info = "No GPS Data"
        self.lock = threading.Lock()
        self.running = True

    def GetGPS(self):
        try:
            while self.running:
                line = self.ser.readline().decode('ascii', errors='replace')
                if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                    msg = pynmea2.parse(line)
                    with self.lock:
                        self.gps_info = f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Timestamp: {msg.timestamp}"
        except serial.SerialException as e:
            print(f"GPS Serial Exception: {e}")
        finally:
            self.ser.close()

class IMU:
    def __init__(self, port, baudrate=9600, timeout=0.5):
        self.buf_length = 11
        self.RxBuff = [0] * self.buf_length
        self.ACCData = [0.0] * 8
        self.GYROData = [0.0] * 8
        self.AngleData = [0.0] * 8
        self.FrameState = 0
        self.CheckSum = 0
        self.start = 0
        self.data_length = 0
        self.acc = [0.0] * 3
        self.gyro = [0.0] * 3
        self.Angle = [0.0] * 3

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.imu_info = "No IMU Data"
        self.lock = threading.Lock()
        self.running = True
        print("Serial is Opened:", self.ser.is_open)

    def GetDataDeal(self, list_buf):
        if list_buf[self.buf_length - 1] != self.CheckSum:  # 校验码不正确
            return

        if list_buf[1] == 0x51:  # 加速度输出
            for i in range(6):
                self.ACCData[i] = list_buf[2 + i]  # 有效数据赋值
            self.acc = self.get_acc(self.ACCData)

        elif list_buf[1] == 0x52:  # 角速度输出
            for i in range(6):
                self.GYROData[i] = list_buf[2 + i]  # 有效数据赋值
            self.gyro = self.get_gyro(self.GYROData)

        elif list_buf[1] == 0x53:  # 姿态角度输出
            for i in range(6):
                self.AngleData[i] = list_buf[2 + i]  # 有效数据赋值
            self.Angle = self.get_angle(self.AngleData)

        imu_info = f"acc: {self.acc} \ngyro: {self.gyro} \nangle: {self.Angle}"
        return imu_info

    def DueData(self, inputdata):
        if inputdata == 0x55 and self.start == 0:
            self.start = 1
            self.data_length = 11
            self.CheckSum = 0
            for i in range(11):
                self.RxBuff[i] = 0

        if self.start == 1:
            self.CheckSum += inputdata  # 校验码计算
            self.RxBuff[self.buf_length - self.data_length] = inputdata  # 保存数据
            self.data_length -= 1  # 长度减一
            if self.data_length == 0:  # 接收到完整的数据
                self.CheckSum = (self.CheckSum - inputdata) & 0xff
                self.start = 0  # 清0
                return self.GetDataDeal(self.RxBuff)

    def get_acc(self, datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def get_gyro(self, datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self, datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z

    def read_data(self):
        try:
            while self.running:
                RXdata = self.ser.read(1)
                RXdata = int(RXdata.hex(), 16)
                imu_info = self.DueData(RXdata)
                if imu_info:
                    with self.lock:
                        self.imu_info = imu_info
        except serial.SerialException as e:
            print(f"IMU Serial Exception: {e}")
        finally:
            self.ser.close()

if __name__ == '__main__':
    input_folder = 'video/demo vedio'
    output_folder = 'video/save'
    os.makedirs(output_folder, exist_ok=True)
    video_files = [f for f in os.listdir(input_folder) if f.endswith('.mp4')]

    GPS_port = '/dev/tty.usbmodem11301'
    IMU_port = '/dev/tty.usbserial-0001'
    
    gps = GPS(GPS_port)
    imu = IMU(IMU_port)

    gps_thread = threading.Thread(target=gps.GetGPS, daemon=True)
    imu_thread = threading.Thread(target=imu.read_data, daemon=True)
    gps_thread.start()
    imu_thread.start()

    try:
        for video_file in video_files:
            input_video_path = os.path.join(input_folder, video_file)
            output_video_path = os.path.join(output_folder, f"output_{video_file}")

            cap = cv2.VideoCapture(input_video_path)
            frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (width, height))

            font_scale = 0.7
            thickness = 2

            try:
                while cap.isOpened():
                    ret, frame = cap.read()
                    if not ret:
                        break

                    with gps.lock:
                        gps_info = gps.gps_info

                    with imu.lock:
                        imu_info = imu.imu_info

                    text_size_gps = cv2.getTextSize(gps_info, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                    x, y = 10, height - 70
                    text_w, text_h = text_size_gps
                    box_coords_gps = ((x, y - text_h - 10), (x + text_w + 10, y + 10))
                    cv2.rectangle(frame, box_coords_gps[0], box_coords_gps[1], (0, 0, 0), cv2.FILLED)
                    cv2.putText(frame, gps_info, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

                    text_size_imu = cv2.getTextSize(imu_info, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                    x, y = 10, height - 20
                    text_w, text_h = text_size_imu
                    box_coords_imu = ((x, y - text_h - 10), (x + text_w + 10, y + 10))
                    cv2.rectangle(frame, box_coords_imu[0], box_coords_imu[1], (0, 0, 0), cv2.FILLED)
                    cv2.putText(frame, imu_info, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

                    out.write(frame)

            except KeyboardInterrupt:
                print("停止讀取")
            finally:
                cap.release()
                out.release()

            print(f"影片已成功合成並儲存為 {output_video_path}")

    finally:
        gps.running = False
        imu.running = False
        gps_thread.join()
        imu_thread.join()
        gps.ser.close()
        imu.ser.close()
