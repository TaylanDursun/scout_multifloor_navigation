#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers

class FloorDetector(Node):
    def __init__(self):
        super().__init__('floor_detector')
        
        # Aruco marker verisini dinliyoruz
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.listener_callback,
            10)
        
        self.current_floor = None
        self.get_logger().info('--- KAT TESPIT SISTEMI BASLATILDI ---')
        self.get_logger().info('Marker bekleniyor...')

    def listener_callback(self, msg):
        # Eğer marker tespit edildiyse
        if len(msg.marker_ids) > 0:
            marker_id = msg.marker_ids[0] # İlk görülen marker'ı al
            
            # ID 0 -> KAT 1
            if marker_id == 0:
                detected_floor = "KAT 1 (Giris Kati)"
                if self.current_floor != 1:
                    self.get_logger().info(f'YENI KONUM TESPIT EDILDI: {detected_floor}')
                    self.get_logger().info('-> Navigasyon Haritasi Floor_1 olarak ayarlaniyor... [SIMULASYON]')
                    self.current_floor = 1

            # ID 1 -> KAT 2
            elif marker_id == 1:
                detected_floor = "KAT 2 (Ust Kat)"
                if self.current_floor != 2:
                    self.get_logger().info(f'YENI KONUM TESPIT EDILDI: {detected_floor}')
                    self.get_logger().info('-> Navigasyon Haritasi Floor_2 olarak ayarlaniyor... [SIMULASYON]')
                    self.current_floor = 2
            
            else:
                self.get_logger().warn(f'Bilinmeyen Marker ID: {marker_id}')

def main(args=None):
    rclpy.init(args=args)
    floor_detector = FloorDetector()
    rclpy.spin(floor_detector)
    floor_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
