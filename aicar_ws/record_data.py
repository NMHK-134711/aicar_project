#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime
import os
import sys

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        self.bridge = CvBridge()
        self.writer = None
        self.frame_count = 0
        self.is_recording = True

        # --- 비디오 저장 설정 ---
        # 현재 시간을 파일명에 포함하여 중복 방지
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"training_data_{timestamp}.mp4"
        
        # FPS 설정 (카메라 노드의 설정과 맞춰주는 것이 좋습니다. 기본 30)
        self.fps = 30.0 

        print("=" * 50)
        print(f"🎥 데이터 수집을 시작합니다.")
        print(f"구독 토픽: /image_processed")
        print(f"저장 파일: {os.path.abspath(self.filename)}")
        print("종료하려면 'Ctrl + C'를 누르세요.")
        print("=" * 50)

        # --- 구독 설정 ---
        # QoS를 BEST_EFFORT로 설정하면 데이터 유실을 감수하고 최신 데이터를 받지만,
        # 데이터 수집용이므로 기본값(RELIABLE)을 사용하여 최대한 모든 프레임을 받도록 합니다.
        self.subscription = self.create_subscription(
            Image,
            '/image_processed',
            self.image_callback,
            10)

    def image_callback(self, msg):
        if not self.is_recording:
            return

        try:
            # ROS Image -> OpenCV Image (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 첫 프레임이 들어오면 VideoWriter 초기화
            if self.writer is None:
                h, w = cv_image.shape[:2]
                # mp4v 코덱 사용 (보편적인 MP4 코덱)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(self.filename, fourcc, self.fps, (w, h))
                print(f"✅ 비디오 초기화 완료: {w}x{h} @ {self.fps}fps")

            # 프레임 저장
            self.writer.write(cv_image)
            self.frame_count += 1
            
            # 1초(30프레임)마다 진행 상황 출력
            if self.frame_count % 30 == 0:
                # print(f"\r[Recording] Saved frames: {self.frame_count} ({self.frame_count / self.fps:.1f}s)", end="")
                # 터미널 출력 문제 방지를 위해 단순 출력으로 변경
                print(f"[Recording] Saved frames: {self.frame_count} ({self.frame_count / self.fps:.1f}s)")

        except Exception as e:
            print(f"\n❌ 에러 발생: {e}")

    def stop_recording(self):
        """ 종료 시 비디오 파일을 안전하게 닫습니다. """
        self.is_recording = False
        if self.writer is not None:
            self.writer.release()
            print("\n" + "=" * 50)
            print(f"💾 저장 완료: {self.filename}")
            print(f"총 프레임 수: {self.frame_count}")
            print("=" * 50)
        else:
            print("\n⚠️ 저장된 프레임이 없습니다.")

def main(args=None):
    rclpy.init(args=args)
    recorder = VideoRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\n🛑 녹화 종료 요청 받음...")
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()