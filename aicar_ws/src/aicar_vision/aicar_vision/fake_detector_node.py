import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys

# 매핑: 입력 숫자 -> 발행할 문자열
SIGN_MAP = {
    '1': 'stop',
    '2': 'left_turn',
    '3': 'right_turn',
    '4': 'traffic_light_green', # '4'로 하면 horn과 겹쳐서 임시로 4D로 했습니다. (4번 항목이 없어서)
    '5': 'traffic_light_red',
    '6': 'horn',
    '7': 'slow', # 7번 slow도 추가했습니다.
}

MENU = """
---------------------------------
 Fake Sign Detector (aicar_vision) 
---------------------------------
[1] stop
[2] left_turn
[3] right_turn
[4] traffic_light_green
[5] traffic_light_red
[6] horn
[7] slow
---------------------------------
[q] Quit
---------------------------------
Enter number: """

class FakeDetectorNode(Node):
    def __init__(self):
        super().__init__('fake_detector_node')
        self.get_logger().info('Fake Sign Detector Node started.')
        self.publisher_ = self.create_publisher(String, '/sign_detection', 10)
        
        # input()이 rclpy.spin()을 막지 않도록 별도 스레드에서 실행
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.start()

    def input_loop(self):
        print(MENU, end='', flush=True)
        
        while rclpy.ok():
            try:
                # 표준 입력에서 한 줄 읽기
                user_input = sys.stdin.readline().strip()

                if user_input == 'q':
                    break
                
                if user_input in SIGN_MAP:
                    sign_name = SIGN_MAP[user_input]
                    
                    msg = String()
                    msg.data = sign_name
                    self.publisher_.publish(msg)
                    
                    self.get_logger().info(f'Published sign: "{sign_name}"')
                elif user_input: # 빈 입력이 아닐 경우
                    print("\nInvalid input. Please select from the list.")

                print(MENU, end='', flush=True)

            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f'Input loop error: {e}')
                break
        
        # 루프 종료 시 ROS 종료 요청
        self.get_logger().info('Shutting down fake detector node...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetectorNode()
    
    try:
        # 메인 스레드는 ROS 이벤트를 처리
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # input_thread가 종료될 때까지 잠시 대기
        if node.input_thread.is_alive():
            node.input_thread.join(timeout=1.0)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()