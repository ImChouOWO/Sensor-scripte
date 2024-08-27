import serial
import pynmea2

# 使用 Mac 上的 GPS 接收器設備路徑
ser = serial.Serial('/dev/tty.usbmodem11301', baudrate=9600, timeout=1)

try:
    while True:
        line = ser.readline().decode('ascii', errors='replace')
        if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
            msg = pynmea2.parse(line)
            print(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Timestamp: {msg.timestamp}")
except KeyboardInterrupt:
    print("停止讀取")
finally:
    ser.close()
