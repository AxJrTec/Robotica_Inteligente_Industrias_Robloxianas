import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from threading import Lock

class SemaforoROI(Node):
    def __init__(self):
        super().__init__('semaforo_roi_node')
        
        # 1. Candado para asegurar que solo se procese un frame a la vez
        self.lock = Lock()
        
        # 2. Nombre de ventana fijo y creación única
        self.window_name = "Deteccion_Semaforo_ROI"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        self.config_hsv = {
            'rojo': [((0, 100, 100), (10, 255, 255)),
                     ((160, 100, 100), (180, 255, 255))],
            'amarillo': [((10, 100, 100), (45, 255, 255))],
            'verde': [((46, 100, 100), (80, 255, 255))]
        }
        
        self.p_ancho = 0.45 
        self.p_alto = 0.95   
        self.MIN_AREA = 170
        self.MAX_AREA = 1000
        
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, 'estado_semaforo', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Nodo de Semáforo optimizado iniciado.')

    def obtener_roi(self, frame):
        alto, ancho = frame.shape[:2]
        centro_x, centro_y = ancho // 2, alto // 2
        ancho_roi, alto_roi = int(ancho * self.p_ancho), int(alto * self.p_alto)
        
        x1, y1 = max(0, centro_x - (ancho_roi // 2)), max(0, centro_y - (alto_roi // 2))
        x2, y2 = min(ancho, centro_x + (ancho_roi // 2)), min(alto, centro_y + (alto_roi // 2))
        
        roi = frame[y1:y2, x1:x2]
        frame_visual = frame.copy()
        cv2.rectangle(frame_visual, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        return roi, frame_visual, (x1, y1)

    def obtener_contorno_valido(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        
        if self.MIN_AREA < area < self.MAX_AREA:
            return c
        return None

    def image_callback(self, msg):
        if not self.lock.acquire(blocking=False):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (320, 240))
            
            roi, frame_visual, (x1, y1) = self.obtener_roi(frame)
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            estado = "NINGUNO"
            c_detectado = None

            for color, rangos in self.config_hsv.items():
                mask_final = np.zeros(hsv_roi.shape[:2], dtype=np.uint8)
                for lower, upper in rangos:
                    mask = cv2.inRange(hsv_roi, np.array(lower), np.array(upper))
                    mask_final = cv2.add(mask_final, mask)
                
                contorno = self.obtener_contorno_valido(mask_final)
                if contorno is not None:
                    estado = color.upper()
                    c_detectado = contorno
                    break 

            if c_detectado is not None:
                (x, y), radio = cv2.minEnclosingCircle(c_detectado)
                center = (int(x + x1), int(y + y1))
                radius = int(radio)
                cv2.circle(frame_visual, center, radius, (0, 255, 255), 2)

            # Publicación de estado
            msg_pub = String()
            msg_pub.data = estado
            self.publisher_.publish(msg_pub)

            # UI
            cv2.putText(frame_visual, f"Estado: {estado}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(self.window_name, frame_visual)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error en callback: {e}')
        
        finally:
            self.lock.release()

def main(args=None):
    rclpy.init(args=args)
    node = SemaforoROI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpieza correcta
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
