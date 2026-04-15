import rclpy
from rclpy.node import Node
import math
from robloxianas_msg.msg import PosePose

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        
        # 1. Declarar parámetros con valores por defecto vacíos
        self.declare_parameter('puntos_x', [])
        self.declare_parameter('puntos_y', [])
        self.declare_parameter('tiempos', [])
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('w_max', 4.3)

        self.pub = self.create_publisher(PosePose, 'trayectoria_completa', 10)
        
        # 2. Decidir de dónde vienen los datos
        px = self.get_parameter('puntos_x').value
        
        if len(px) > 0:
            self.get_logger().info('Cargando trayectoria desde archivo YAML...')
            self.process_trajectory(px)
        else:
            self.get_logger().info('No se detectó YAML. Iniciando entrada manual por terminal...')
            self.get_manual_input()

    def get_manual_input(self):
        try:
            n = int(input("\n¿Cuántos puntos deseas ingresar?: "))
            px, py, pt = [], [], []
            for i in range(n):
                print(f"--- Punto {i+1} ---")
                px.append(float(input("  X: ")))
                py.append(float(input("  Y: ")))
                pt.append(float(input("  Tiempo: ")))
            self.process_trajectory(px, py, pt)
        except ValueError:
            self.get_logger().error('Entrada inválida. Usa números.')

    def process_trajectory(self, px, py=None, pt=None):
        # Si py y pt son None, significa que vienen de los parámetros
        if py is None: py = self.get_parameter('puntos_y').value
        if pt is None: pt = self.get_parameter('tiempos').value
        
        v_limit = self.get_parameter('v_max').value
        w_limit = self.get_parameter('w_max').value

        msg = PosePose()
        msg.cantidad_puntos = len(px)
        curr_x, curr_y, curr_theta = 0.0, 0.0, 0.0

        for i in range(len(px)):
            dx = px[i] - curr_x
            dy = py[i] - curr_y
            dist = math.hypot(dx, dy)
            target_theta = math.atan2(dy, dx)
            dtheta = math.atan2(math.sin(target_theta - curr_theta), math.cos(target_theta - curr_theta))

            # Cálculo de tiempo mínimo para ROBUSTEZ
            t_min = (abs(dtheta) / w_limit) + (dist / v_limit)
            t_req = pt[i]

            if t_req < t_min:
                self.get_logger().warn(f'Punto {i+1} inalcanzable. Ajustando tiempo a {t_min:.2f}s')
                t_req = t_min

            # Distribución de tiempos y velocidades
            t_turn = t_req * ((abs(dtheta)/w_limit) / t_min) if t_min > 0 else 0.0
            t_fwd = t_req - t_turn
            
            v = dist / t_fwd if t_fwd > 0 else 0.0
            w = dtheta / t_turn if t_turn > 0 else 0.0

            msg.x_puntos.append(px[i])
            msg.y_puntos.append(py[i])
            msg.vel_lineal.append(v)
            msg.vel_angular.append(w)
            msg.t_turn.append(t_turn)
            msg.t_fwd.append(t_fwd)

            curr_x, curr_y, curr_theta = px[i], py[i], target_theta

        self.pub.publish(msg)
        self.get_logger().info('¡Trayectoria publicada exitosamente!')

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    # No usamos rclpy.spin porque el nodo hace su trabajo y debe morir
    node.destroy_node()
    rclpy.shutdown()