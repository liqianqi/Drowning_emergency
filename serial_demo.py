import time
import numpy as np
import cv2

import serial           # 导入串口通信库
from time import sleep

ser = serial.Serial()

def port_open_recv():   # 对串口的参数进行配置
    ser.port='/dev/ttyUSB0'
    ser.baudrate=115200
    ser.bytesize=8
    ser.stopbits=1
    ser.parity="N"      # 奇偶校验位
    ser.open()
    if(ser.isOpen()):
        print("串口打开成功！")
    else:
        print("串口打开失败！")
# isOpen() 函数来查看串口的开闭状态


def port_close():
    ser.close()
    if(ser.isOpen()):
        print("串口关闭失败！")
    else:
        print("串口关闭成功！")

def send(send_data):
    if(ser.isOpen()):
        ser.write(send_data.encode('utf-8'))#编码
        print("发送成功",send_data)
    else:
        print("发送失败！")

if __name__ == '__main__':
    port_open_recv()
    while True:
        send_data=(1,2.0,3,4)
        send(send_data)
        sleep(0.5)        # 起到一个延时的效果，这里如果不加上一个while True，程序执行一次就自动跳出了

