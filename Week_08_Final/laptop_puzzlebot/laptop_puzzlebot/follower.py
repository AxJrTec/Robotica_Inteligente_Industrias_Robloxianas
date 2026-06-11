import time
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import YOLO con guard
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


SEMAFORO_MAP = {
    'green':  'VERDE',
    'red':    'ROJO',
    'yellow': 'AMARILLO',
}

SENALES_VALIDAS = {
    'construccion', 'forward', 'giveway', 'roundabout',
    'stop', 'turnleftahead', 'turnrightahead',
}


class Follower(Node):
    def __init__(self):
        super().__init__('laptop_puzzlebot')

        # Parametros generales
        self.declare_parameter('signals_model',  '/home/edwin/puzzle_ws/src/laptop_puzzlebot/laptop_puzzlebot/signals.pt')
        self.declare_parameter('traffic_model',  '/home/edwin/puzzle_ws/src/laptop_puzzlebot/laptop_puzzlebot/semaforo.pt')
        self.declare_parameter('conf_threshold', 0.7)
        self.declare_parameter('input_topic',    '/camera/compressed')
        self.declare_parameter('device',         'cuda')
        self.declare_parameter('show_windows',   True)

        # Parametros del seguidor
        self.declare_parameter('roi_top_frac',   0.60)
        self.declare_parameter('roi_bot_frac',   0.95)
        self.declare_parameter('roi_side_frac',  0.15)
        self.declare_parameter('use_adaptive',   True)
        self.declare_parameter('bin_threshold',  80)
        self.declare_parameter('min_area',       300)
        self.declare_parameter('hold_frames',    5)

        # Limites de error
        self.declare_parameter('max_error_jump', 60.0)
        self.declare_parameter('max_error_abs',  120.0)

        # Tamaño minimo de bounding box para YOLO
        self.declare_parameter('min_signal_w',   70)
        self.declare_parameter('min_signal_h',   70)
        self.declare_parameter('min_semaforo_w', 20)
        self.declare_parameter('min_semaforo_h', 60)

        signals_path        = self.get_parameter('signals_model').value
        traffic_path        = self.get_parameter('traffic_model').value
        self.conf_thr       = self.get_parameter('conf_threshold').value
        input_topic         = self.get_parameter('input_topic').value
        self.device         = self.get_parameter('device').value
        self.show_win       = self.get_parameter('show_windows').value
        self.roi_top_f      = self.get_parameter('roi_top_frac').value
        self.roi_bot_f      = self.get_parameter('roi_bot_frac').value
        self.roi_side_f     = self.get_parameter('roi_side_frac').value
        self.use_adapt      = self.get_parameter('use_adaptive').value
        self.bin_thr        = self.get_parameter('bin_threshold').value
        self.min_area       = self.get_parameter('min_area').value
        self.hold_frames    = self.get_parameter('hold_frames').value
        self.max_err_jump   = self.get_parameter('max_error_jump').value
        self.max_err_abs    = self.get_parameter('max_error_abs').value
        self.min_signal_w   = self.get_parameter('min_signal_w').value
        self.min_signal_h   = self.get_parameter('min_signal_h').value
        self.min_semaforo_w = self.get_parameter('min_semaforo_w').value
        self.min_semaforo_h = self.get_parameter('min_semaforo_h').value

        # Modelos YOLO
        self.model_signals = None
        self.model_traffic = None

        if YOLO_AVAILABLE:
            if signals_path:
                try:
                    self.model_signals = YOLO(signals_path)
                    self.model_signals.to(self.device)
                    self.get_logger().info(f'Modelo senales cargado: {signals_path}')
                except Exception as e:
                    self.get_logger().error(f'No se pudo cargar modelo senales: {e}')
            else:
                self.get_logger().warn('signals_model no especificado')

            if traffic_path:
                try:
                    self.model_traffic = YOLO(traffic_path)
                    self.model_traffic.to(self.device)
                    self.get_logger().info(f'Modelo semaforo cargado: {traffic_path}')
                except Exception as e:
                    self.get_logger().error(f'No se pudo cargar modelo semaforo: {e}')
            else:
                self.get_logger().warn('traffic_model no especificado')
        else:
            self.get_logger().error('ultralytics no instalado. Instalar con: pip install ultralytics')

        # Kernels OpenCV
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        # Persistencia YOLO
        self.PERSIST_FRAMES = 5
        self._sem_count     = 0
        self._sen_count     = 0
        self._last_semaforo = 'NINGUNO'
        self._last_senal    = 'NINGUNO'

        # Memoria del seguidor
        self._last_error    = 0.0
        self._lost_count    = 0

        # Subsampling YOLO
        self.YOLO_EVERY_N   = 1
        self._frame_counter = 0

        if self.show_win:
            cv2.namedWindow('Seguidor (centroide)', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Binaria',              cv2.WINDOW_NORMAL)
            cv2.namedWindow('Detecciones',          cv2.WINDOW_NORMAL)

        self.sub_video    = self.create_subscription(CompressedImage, input_topic, self.callback_video, 10)
        self.pub_error    = self.create_publisher(Float32, 'error',           10)
        self.pub_linea    = self.create_publisher(Bool,    'line_detected',   10)
        self.pub_semaforo = self.create_publisher(String,  'estado_semaforo', 10)
        self.pub_senal    = self.create_publisher(String,  'senal_detectada', 10)
        self.pub_debug    = self.create_publisher(Image,   'follower/debug',  10)

        self.get_logger().info(
            f'Follower iniciado | {input_topic} | device={self.device} | '
            f'max_jump={self.max_err_jump}px | max_abs={self.max_err_abs}px')

    # YOLO
    def _run_yolo(self, frame: np.ndarray):
        semaforo_det = None
        senal_det    = None

        if self.model_traffic is not None:
            results   = self.model_traffic.predict(frame, conf=self.conf_thr, verbose=False, device=self.device)
            best_conf = 0.0
            for r in results:
                for box in r.boxes:
                    cls_name = r.names[int(box.cls)].lower()
                    conf     = float(box.conf)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if (x2-x1) < self.min_semaforo_w or (y2-y1) < self.min_semaforo_h:
                        continue
                    if cls_name in SEMAFORO_MAP and conf > best_conf:
                        best_conf    = conf
                        semaforo_det = SEMAFORO_MAP[cls_name]
                    color = {'ROJO': (0,0,255), 'AMARILLO': (0,255,255), 'VERDE': (0,255,0)}.get(
                        SEMAFORO_MAP.get(cls_name, ''), (180,180,180))
                    cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                    cv2.putText(frame, f'{cls_name} {conf:.2f}', (x1, y1-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if self.model_signals is not None:
            results   = self.model_signals.predict(frame, conf=self.conf_thr, verbose=False, device=self.device)
            best_conf = 0.0
            for r in results:
                for box in r.boxes:
                    cls_name = r.names[int(box.cls)].lower()
                    conf     = float(box.conf)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if (x2-x1) < self.min_signal_w or (y2-y1) < self.min_signal_h:
                        continue
                    if cls_name in SENALES_VALIDAS and conf > best_conf:
                        best_conf = conf
                        senal_det = cls_name
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (255,140,0), 2)
                    cv2.putText(frame, f'{cls_name} {conf:.2f}', (x1, y1-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,140,0), 1)

        if semaforo_det:
            self._last_semaforo = semaforo_det; self._sem_count = 0
        else:
            self._sem_count += 1
            if self._sem_count >= self.PERSIST_FRAMES:
                self._last_semaforo = 'NINGUNO'

        if senal_det:
            self._last_senal = senal_det; self._sen_count = 0
        else:
            self._sen_count += 1
            if self._sen_count >= self.PERSIST_FRAMES:
                self._last_senal = 'NINGUNO'

        return self._last_semaforo, self._last_senal, frame

    # Seguidor por centroide
    def _seguir_por_centroide(self, roi_bgr: np.ndarray):
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)

        if self.use_adapt:
            binary = cv2.adaptiveThreshold(
                blur, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                blockSize=51, C=10)
        else:
            _, binary = cv2.threshold(blur, self.bin_thr, 255, cv2.THRESH_BINARY_INV)

        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  self.kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, binary, None

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.min_area:
            return None, binary, None

        M = cv2.moments(c)
        if M['m00'] == 0:
            return None, binary, None
        cx = int(M['m10'] / M['m00'])
        return cx, binary, c

    # Validacion del error: limita saltos bruscos
    def _validar_error(self, error_raw: float) -> tuple[float, bool]:
        # Filtro 1: valor absoluto demasiado grande
        if abs(error_raw) > self.max_err_abs:
            self.get_logger().warn(
                f'Error absoluto {error_raw:.1f}px > limite {self.max_err_abs}px -> descartado')
            return self._last_error, False

        # Filtro 2: salto brusco respecto al frame anterior
        jump = abs(error_raw - self._last_error)
        if jump > self.max_err_jump:
            self.get_logger().warn(
                f'Salto de error {jump:.1f}px > limite {self.max_err_jump}px -> descartado')
            return self._last_error, False

        return error_raw, True

    # Callback principal
    def callback_video(self, msg: CompressedImage):
        t_start = time.perf_counter()

        np_arr = np.frombuffer(msg.data, np.uint8)
        img    = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn('Frame vacio recibido, descartando')
            return

        img    = cv2.rotate(img, cv2.ROTATE_180)
        full_h, full_w = img.shape[:2]

        self._frame_counter += 1
        if self._frame_counter % self.YOLO_EVERY_N == 0:
            semaforo_str, senal_str, img = self._run_yolo(img)
        else:
            semaforo_str = self._last_semaforo
            senal_str    = self._last_senal

        img_boxes = img.copy()

        sem_msg = String(); sem_msg.data = semaforo_str
        sen_msg = String(); sen_msg.data = senal_str
        self.pub_semaforo.publish(sem_msg)
        self.pub_senal.publish(sen_msg)

        # ROI
        y_top = int(full_h * self.roi_top_f)
        y_bot = int(full_h * self.roi_bot_f)
        x_off = int(full_w * self.roi_side_f)
        x_end = full_w - x_off

        roi = img[y_top:y_bot, x_off:x_end]
        roi_h, roi_w = roi.shape[:2]

        cx_roi, binaria, contorno = self._seguir_por_centroide(roi)

        centro_imagen = roi_w // 2

        if cx_roi is not None:
            error_raw = float(centro_imagen - cx_roi)

            # Aplicar limites de error antes de publicar
            error, valido = self._validar_error(error_raw)

            if valido:
                self._last_error = error
                self._lost_count = 0
                linea_detectada  = True
            else:
                # El centroide existia pero el error fue rechazado:
                # mantenemos el ultimo error valido como si fuera hold_frames
                error           = self._last_error
                linea_detectada = True
                self._lost_count += 1
                if self._lost_count > self.hold_frames:
                    error           = 0.0
                    linea_detectada = False
        else:
            # Sin contorno detectado
            self._lost_count += 1
            if self._lost_count <= self.hold_frames:
                error           = self._last_error
                linea_detectada = True
            else:
                error           = 0.0
                linea_detectada = False

        err_msg   = Float32(); err_msg.data   = error
        linea_msg = Bool();    linea_msg.data = linea_detectada
        self.pub_error.publish(err_msg)
        self.pub_linea.publish(linea_msg)

        # Debug visual
        if contorno is not None:
            cv2.drawContours(roi, [contorno], -1, (0, 255, 0), 2)
        if cx_roi is not None:
            cv2.circle(roi, (cx_roi, roi_h // 2), 6, (0, 255, 255), -1)
        cv2.line(roi, (centro_imagen, 0), (centro_imagen, roi_h), (0, 0, 255), 2)
        if cx_roi is not None:
            cv2.line(roi, (cx_roi, 0), (cx_roi, roi_h), (0, 255, 255), 1)

        det_label = ''
        if semaforo_str != 'NINGUNO':
            det_label += f'SEM:{semaforo_str} '
        if senal_str != 'NINGUNO':
            det_label += f'SENAL:{senal_str} '
        if not linea_detectada:
            det_label += 'SIN LINEA'
        else:
            det_label += f'err:{error:+.1f}px'
        cv2.putText(roi, det_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        img[y_top:y_bot, x_off:x_end] = roi
        cv2.rectangle(img, (x_off, y_top), (x_end, y_bot), (255, 255, 0), 1)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

        t_proc = time.perf_counter() - t_start

        if self.show_win:
            cv2.imshow('Seguidor (centroide)', roi)
            cv2.imshow('Binaria',              binaria)
            cv2.imshow('Detecciones',          img_boxes)
            cv2.waitKey(1)

        self.get_logger().info(
            f'Error: {error:+8.2f} px | Linea: {linea_detectada} | '
            f'Tiempo callback: {t_proc * 1000:6.2f} ms')


def main(args=None):
    rclpy.init(args=args)
    nodo = Follower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
