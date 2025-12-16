import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_duckie_interfaces.srv import SetColor # WSL에 빌드된 커스텀 서비스

class ServiceToTopicBridge(Node):
    def __init__(self):
        super().__init__('led_service_to_topic_bridge')
        
        # 1. 서비스 서버 생성
        self.srv = self.create_service(
            SetColor,
            '/duckie_led_control',
            self.service_callback)
        
        # 2. 토픽 퍼블리셔 생성
        self.publisher_ = self.create_publisher(
            String,
            '/duckie_led_control_topic',
            10)
        
        self.get_logger().info("LED 서비스-토픽 브릿지 노드가 준비되었습니다.")

    def service_callback(self, request, response):
        color_name = request.color
        self.get_logger().info(f"서비스 요청 수신: '{color_name}', 토픽 발행 중...")
        
        # 서비스 요청 데이터를 String 메시지에 담아 발행
        msg = String()
        msg.data = color_name
        self.publisher_.publish(msg)
        
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    bridge_node = ServiceToTopicBridge()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
