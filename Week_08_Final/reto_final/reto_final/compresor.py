import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageRelay(Node):
    def __init__(self):
	super().__init__('compresor_node')

	# Parametros 
	self.declare_parameter('jpeg_quality',  40)
	self.declare_parameter('input_topic',   '/video_source/raw')
	self.declare_parameter('output_topic',  '/camera/compressed')

	self.jpeg_quality  = self.get_parameter('jpeg_quality').value
	input_topic        = self.get_parameter('input_topic').value
	output_topic       = self.get_parameter('output_topic').value

	# CV Bridge 
	self.bridge = CvBridge()

	# Encode params 
	self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

	# Pub / Sub 
	self.sub = self.create_subscription(
	    Image, input_topic, self.image_callback, 10)
	self.pub = self.create_publisher(
	    CompressedImage, output_topic, 10)

	self.get_logger().info(
	    f'ImageRelay iniciado | '
	    f'{input_topic} → {output_topic} | '
	    f'JPEG quality={self.jpeg_quality}')

    # ── Callback 
    def image_callback(self, msg: Image):
	try:
	    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	except Exception as e:
	    self.get_logger().error(f'cv_bridge error: {e}')
	    return

	# Comprimir a JPEG en memoria (sin escribir a disco)
	ok, buf = cv2.imencode('.jpg', frame, self._encode_params)
	if not ok:
	    self.get_logger().warn('imencode falló, frame descartado')
	    return

	out = CompressedImage()
	out.header = msg.header
	out.format = 'jpeg'
	out.data   = buf.tobytes()

	self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelay()
    try:
	rclpy.spin(node)
    except KeyboardInterrupt:
	pass
    finally:
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
