import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped

class ZedObjectListener(Node):
    def __init__(self):
        super().__init__('zed_object_listener')
        self.subscription = self.create_subscription(
            ObjectsStamped, 
            '/zed/zed_node/obj_det/objects',
            self.listener_callback,
            10
        )

        # 프레임 카운트 변수
        self.total_frames = 0  # 전체 프레임 수
        self.detected_person_frames = 0  # 사람(person)이 감지된 프레임 수

    def listener_callback(self, msg):
        self.total_frames += 1  # 매 프레임마다 전체 프레임 수 증가
        person_detected = False  # 현재 프레임에서 사람이 감지되었는지 확인

        for obj in msg.objects:
            label = obj.label  # 객체 라벨
            z_distance = obj.position[2]  # 물체의 Z 좌표 (거리)

            if label.lower() == "person":  # 사람이 감지되었을 경우
                person_detected = True
                self.get_logger().info(f"Label: {label}, Distance (z): {z_distance:.2f} meters")

        if person_detected:
            self.detected_person_frames += 1  # 사람 감지된 프레임 증가

        # 인식률 계산 및 출력
        detection_rate = (self.detected_person_frames / self.total_frames) * 100 if self.total_frames > 0 else 0
        self.get_logger().info(f"Total Frames: {self.total_frames}, Person Detected Frames: {self.detected_person_frames}, Detection Rate: {detection_rate:.2f}%")

def main(args=None):
    rclpy.init(args=args)
    node = ZedObjectListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
        # 프로그램 종료 시 최종 인식률 출력
    finally:
        detection_rate = (node.detected_person_frames / node.total_frames) * 100 if node.total_frames > 0 else 0
        print(f"\nFinal Detection Rate: {detection_rate:.2f}%")
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

