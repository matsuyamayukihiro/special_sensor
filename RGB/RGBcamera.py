# カラーのみ表示
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import socket

# カメラR用
pipeline = rs.pipeline()  # カメラR用通信ライン構築
# 設定用ファイル(コンフィグファイル)のこと
config = rs.config()  # カメラR用
# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)  # カメラR用 pipelineをラップするとは?

pipeline_profile = config.resolve(pipeline_wrapper)  # 構成が構成フィルターを解決できるかどうかを確認して、一致するデバイスとストリーム・プロファイルを見つけます?

device1 = pipeline_profile.get_device()  # デバイス情報取得

device_product_line1 = str(device1.get_info(rs.camera_info.product_line))  # info.product_line=個体識別ID?

config.enable_stream(rs.stream.color, 640, 480, rs.format.z16, 30)  # カメラ初期設定

if device_product_line1 == 'L500':  # カメラL用
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

config.enable_device('814412071216')  # 912112073474
pipeline.start(config)

# カメラR起動
pipeline1 = rs.pipeline()  # カメラL用通信ライン構築

# 設定用ファイル(コンフィグファイル)のこと
config1 = rs.config()  # カメラR用

# Get device product line for setting a supporting resolution
pipeline_wrapper1 = rs.pipeline_wrapper(pipeline1)  # カメラR用 pipelineをラップするとは?

pipeline_profile1 = config1.resolve(pipeline_wrapper1)  # 構成が構成フィルターを解決できるかどうかを確認して、一致するデバイスとストリーム・プロファイルを見つけます?

device1 = pipeline_profile1.get_device()  # デバイス情報取得

device_product_line1 = str(device1.get_info(rs.camera_info.product_line))  # info.product_line=個体識別ID?

config1.enable_stream(rs.stream.color, 640, 480, rs.format.z16, 30)  # カメラ初期設定

if device_product_line1 == 'L500':  # カメラL用
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
else:
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

config1.enable_device('838212070171')
pipeline1.start(config1)  # カメラL起動

# メインプログラム
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        framesR = pipeline.wait_for_frames()
        framesL = pipeline1.wait_for_frames()

        color_frameR = framesR.get_color_frame()  # 輪郭検知用フレーム取得
        color_frameL = framesL.get_color_frame()  # 輪郭検知用フレーム取得

        if not color_frameR and color_frameL:
            continue  # 画像データ取得まで繰り返す

        # Convert images to numpy arrays
        color_imageL = np.asanyarray(color_frameL.get_data())  # 生画像配列データの取得
        color_imageR = np.asanyarray(color_frameR.get_data())  # 計測用データの取得
        # 画質変更
        h = 190
        w = 270
        dstR = cv2.resize(color_imageR, dsize=(w, h))
        dstL = cv2.resize(color_imageL, dsize=(w, h))

        # ぼかし加工。
        average_square_size = 10  # ぼかしパラメータ 大きくする程にぼけていく
        sigma_color = 5000  # 色空間に関する標準偏差パラメータ  大きくすると色の平滑化範囲を広げる
        sigma_metric = 1  # 距離空間に関する標準偏差パラメータ  大きくすると色の平滑化範囲を広げる (d==0の時のみ有効)
        img_bilateralR = cv2.bilateralFilter(dstR, average_square_size, sigma_color, sigma_metric)  # Bilateralオペレータを使用して平滑化
        img_bilateralL = cv2.bilateralFilter(dstL, average_square_size, sigma_color, sigma_metric)  # Bilateralオペレータを使用して平滑化

        hsv_imgR = cv2.cvtColor(img_bilateralR, cv2.COLOR_BGR2HSV)  # HSVモデルに変更
        hsv_imgL = cv2.cvtColor(img_bilateralL, cv2.COLOR_BGR2HSV)  # HSVモデルに変更

        # 2値処理
        lower = (0, 7, 140)  # 色相は赤→黄→緑→水色→青→紫でループする
        upper = (359, 255, 255)
        bin_imgR = cv2.inRange(hsv_imgR, lower, upper)  # 2値化   引数1 画像指定  引数2 下限の色  引数3 上限の色
        bin_imgL = cv2.inRange(hsv_imgL, lower, upper)  # 2値化   引数1 画像指定  引数2 下限の色  引数3 上限の色

        contoursR, hierarchyR = cv2.findContours(bin_imgR, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭検出
        contoursL, hierarchyL = cv2.findContours(bin_imgL, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭検出
        contoursR = list(filter(lambda x: cv2.contourArea(x) > 100000, contoursR))  # 小さい輪郭は誤検出として削除する
        contoursL = list(filter(lambda x: cv2.contourArea(x) > 100000, contoursL))  # 小さい輪郭は誤検出として削除する

        # 輪郭処理
        xl = 20  # 横の余白パラメータ
        xr = w - xl
        yr1 = yl1 = 5
        yr2 = yl2 = 7

        # 右側判定
        if 255 == bin_imgR[yr1, xl] and 255 == bin_imgR[yl1, xr]:  # 前判定ゾーンで監視
            dataR = 11  # 止まるモード

        elif 255 == bin_imgR[yr1, xr] and 255 == bin_imgR[yr2, xr]:  # 引数1 y座標  引数2 x座標
            dataR = 22  # 左モード

        elif 255 == bin_imgR[yl1, xl] and 255 == bin_imgR[yl2, xl]:  # 左判定ゾーンで監視
            dataR = 33  # "右モード

        else:
            dataR = 44  # 直進モード

        # 左側判定
        if 255 == bin_imgL[yr1, xl] and 255 == bin_imgL[yl1, xr]:  # 前判定ゾーンで監視
            dataL = 11  # 止まるモード

        elif 255 == bin_imgL[yr1, xr] and 255 == bin_imgL[yr2, xr]:  # 引数1 y座標  引数2 x座標
            dataL = 22  # 左モード

        elif 255 == bin_imgL[yl1, xl] and 255 == bin_imgL[yl2, xl]:  # 左判定ゾーンで監視
            dataL = 33  # 右モード

        else:
            dataL = 44  # 直進

        # 総合判定
        if dataL == 11 or dataR == 11:
            print("停止")
            data = '停止する'

        elif dataL == 22 or dataR == 22:  # 引数1 y座標  引数2 x座標
            print("左に行く")
            data = '左に行く'

        elif dataL == 33 or dataR == 33:  # 左判定ゾーンで監視
            print("右")
            data = '右に行く'

        else:
            print("直進")
            data = '直進する'

        # Show images
        cv2.namedWindow('RealSenseR', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSenseR', bin_imgR)
        cv2.namedWindow('RealSenseL', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSenseL', bin_imgL)
        cv2.waitKey(10)

        ##################
        # 送信側プログラム(メイン用)#
        ##################

        # 送信側アドレスの設定
        #    SrcIP = "192.168.50.23"  # 送信側IP
        #   SrcPort = 11111  # 送信側ポート番号
        #  SrcAddr = (SrcIP,SrcPort) # 送信側アドレスをtupleに格納

        # 受信側アドレスの設定
        #  DstIP = "192.168.50.34
        # 受信側IP
        # DstPort = 22222 # 受信側ポート番号
        # DstAddr = (DstIP,DstPort) # 受信側アドレスをtupleに格納

        # ソケット作成
        # udpClntSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #引数1 IPv4用 or IPv6用か   引数2 TCP用 or UDP用か
        # udpClntSock.bind(SrcAddr) # 送信側アドレスでソケットを設定

        # バイナリに変換
        # data = data.encode('utf-8')

        # 受信側アドレスに送信
        # udpClntSock.sendto(data,DstAddr)

        ##################
        # 送信側プログラム(ローカル用)#
        ##################

        # 送信側アドレスの設定
        SrcIP = "127.0.0.1"
        #      送信側IP
        SrcPort = 11111  # 送信側ポート番号
        SrcAddr = (SrcIP, SrcPort)  # 送信側アドレスをtupleに格納

        # 受信側アドレスの設定
        DstIP = "127.0.0.1"
        # 受信側IP
        DstPort = 22222  # 受信側ポート番号
        DstAddr = (DstIP, DstPort)  # 受信側アドレスをtupleに格納

        # ソケット作成
        udpClntSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 引数1 IPv4用 or IPv6用か   引数2 TCP用 or UDP用か
        udpClntSock.bind(SrcAddr)  # 送信側アドレスでソケットを設定

        # バイナリに変換
        data = data.encode('utf-8')

        # 受信側アドレスに送信
        udpClntSock.sendto(data, DstAddr)

finally:
    # Stop streaming
    pipeline.stop()
