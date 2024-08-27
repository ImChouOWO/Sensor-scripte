import serial
import time

# 設定串口參數
ser = serial.Serial(
    port='/dev/tty.usbserial-1130',  # 使用你的串口名稱
    baudrate=115200,  # 設定波特率
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1  # 設定超時時間
)

def send_message():
    # 確認串口已經打開
    if ser.is_open:
        print(f"串口 {ser.name} 已經打開")

    # 發送資料
    message = "Hello from Mac!"
    ser.write(message.encode())  # 將字串編碼成 bytes 並發送

    # 確認資料已經發送
    print(f"已發送訊息: {message}")

if __name__ == "__main__":
    try:
        while True:
            send_message()
            time.sleep(1)  # 設置發送間隔時間，避免過於頻繁
    except KeyboardInterrupt:
        print("停止發送訊息")
    finally:
        ser.close()
        print("串口已關閉")
