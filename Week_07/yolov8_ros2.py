import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference


class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.model  = YOLO('/home/puzzlebot/jetson/best.pt') # Ubicación del Modelo entrenado
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.yolo_pub = self.create_publisher(Yolov8Inference, '/Yolov8_inference', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/inference_result',  10)

        self.get_logger().info('Nodo YOLO iniciado — escuchando /video_source/raw')

    def camera_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        # Preprocesamiento
        img = cv2.resize(img, (320, 240))
        img = cv2.rotate(img, cv2.ROTATE_180)

        # Inferencia
        results = self.model(img)

        # Construir mensaje
        yolo_msg = Yolov8Inference()
        yolo_msg.header.frame_id = 'inference'
        yolo_msg.header.stamp    = self.get_clock().now().to_msg()

        for r in results:
            for box in r.boxes:
                inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                inference_result.class_name = self.model.names[int(c)]
                inference_result.top = int(b[0])
                inference_result.left = int(b[1])
                inference_result.bottom = int(b[2])
                inference_result.right = int(b[3])
                yolo_msg.yolov8_inference.append(inference_result)

        # Publicación del frame para debug
        frame = results[0].plot()
        self.yolo_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        self.yolo_pub.publish(yolo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()