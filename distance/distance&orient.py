#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO
import socket
import math
import sys

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIG1 = 27
ECHO1 = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
TRIG2 = 5
ECHO2 = 6

# GPIO1端子の初期設定
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.output(TRIG1, GPIO.LOW)
time.sleep(0.3)

# GPIO2端子の初期設定
# GPIO.setup(TRIG2,GPIO.OUT)
# GPIO.setup(ECHO2,GPIO.IN)
# GPIO.output(TRIG2, GPIO.LOW)
# time.sleep(0.3)

while True:
    try:
        # Trig端子を10us以上High
        GPIO.output(TRIG1, GPIO.HIGH)
        time.sleep(1)  # 1[S]
        GPIO.output(TRIG1, GPIO.LOW)

        # EchoパルスがHighになる時間
        while GPIO.input(ECHO1) == 0:
            echo_on1 = time.time()

        #  EchoパルスがLowになる時間
        while GPIO.input(ECHO1) == 1:
            echo_off1 = time.time()

        # Echoパルスのパルス幅(us)
        echo_pulse_width1 = (echo_off1 - echo_on1) * 1000

        # 距離を算出:Distance in cm = echo pulse width in uS/58
        distance1 = (echo_pulse_width1 * 34.3) / 2
        distance1 = math.floor(distance1 * 10 ** 2) / (10 ** 2)
        print("distance1", distance1, "mm")

        GPIO.output(TRIG2, GPIO.HIGH)
        time.sleep(1)  # 1[S]
        GPIO.output(TRIG2, GPIO.LOW)
        # EchoパルスがHighになる時間
        while GPIO.input(ECHO2) == 0:
            echo_on2 = time.time()
        # EchoパルスがLowになる時間
        while GPIO.input(ECHO2) == 1:
            echo_off2 = time.time()
        # Echoパルスのパルス幅(us)
        echo_pulse_width2 = (echo_off2 - echo_on2) * 1000  # echo_on2 needs chack

        # 距離を算出:Distance in cm = echo pulse width in uS/58
        distance2 = (echo_pulse_width2 * 34.3) / 2
        distance2 = math.floor(distance2 * 10 ** 2) / (10 ** 2)
        print("distance2 ", distance2, "mm")
        print("    distanceR ", distance2, "mm")

    except KeyboardInterrupt:  # Ctrl+Cキーが押された
        GPIO.cleanup()  # GPIOをクリーンアップ
        sys.exit()

##################
# 受信側プログラム#
##################

# 受信側アドレスの設定
# 受信側IP
#  SrcIP = "127.0.0.1"
# 受信側ポート番号
#  SrcPort = 22222
# 受信側アドレスをtupleに格納
#  SrcAddr = (SrcIP, SrcPort)
# バッファサイズ指定
# BUFSIZE = 1024

# ソケット作成
#  udpServSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 受信側アドレスでソケットを設定
#   udpServSock.bind(SrcAddr)

# While文を使用して常に受信待ちのループを実行
#    while True:
# ソケットにデータを受信した場合の処理
# 受信データを変数に設定
#  data, addr = udpServSock.recvfrom(BUFSIZE)
# 受信データと送信アドレスを出力
#            print(data.decode(),"    distanceR ",distance1,"mm","    distanceL ",distance2,"mm")
#   except KeyboardInterrupt:       #Ctrl+Cキーが押された
#          GPIO.cleanup()              #GPIOをクリーンアップ
#         sys.exit()
