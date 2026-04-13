import rclpy
from rclpy.node import Node
import math
from custom_msgs.msg import CustomPose

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        self.min_cmd_v = 0.01
        self.max_cmd_v = 0.4
        self.min_cmd_w = 0.2
        self.max_cmd_w = 4.3

        self.targets = []
        self.pub = self.create_publisher(CustomPose, 'pose', 10)

        self.wait_for_ros_time()
        self.generate_path()
        self.publish_path()

    # Definir y Generar Trayectoria
    def generate_path(self):
        print("\n" + "="*20)
        print("GENERADOR DE TRAYECTORIA")
        print("="*20)

        while True:
            try:
                n = int(input("Numero de puntos: "))
                if n > 0:
                    break
                else:
                    print("Defina almenos un punto")
            except ValueError:
                print("Error: Ingrese un numero entero valido.")

        current_x, current_y, current_theta = 0.0, 0.0, 0.0

        for i in range(n):
            print(f"\n--- Punto {i+1} ---")
            while True:
                try:
                    x = float(input("Coordenada X: ")) #Escala en metros
                    y = float(input("Coordenada Y: ")) #Escala en metros
                    t_user = float(input("Tiempo (s): "))

                    # Obtenemos distancia al siguiente punto
                    dx = x - current_x
                    dy = y - current_y
                    dist = math.hypot(dx, dy)

                    # Obtenemos el angulo entre puntos
                    target_theta = math.atan2(dy, dx)
                    dtheta = target_theta - current_theta
                    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

                    # Calculamos el tiempo minimo para alcanzar el punto
                    t_turn_min = abs(dtheta) / self.max_cmd_w
                    t_fwd_min = dist / self.max_cmd_v
                    t_min = t_turn_min + t_fwd_min

                    # Validamos si es posible alcanzarse 
                    if t_user < t_min:
                        print(f"\n[ERROR] No alcanzable en {t_user:.2f}s")
                        print(f"Tiempo mínimo: {t_min:.2f}s")
                    else:
                        # Dividimos el tiempo entre la velocidad angular y lineal
                        if t_min == 0:
                            t_turn = 0
                            t_fwd = t_user
                        else:
                            t_turn = t_user * (t_turn_min / t_min)
                            t_fwd = t_user * (t_fwd_min / t_min)

                        w = dtheta / t_turn if t_turn > 0 else 0.0
                        v = dist / t_fwd if t_fwd > 0 else 0.0

                        self.targets.append({
                            'x': x,
                            'y': y,
                            'theta': target_theta,
                            'cmd_v': v,
                            'cmd_w': w,
                            't_turn': t_turn,
                            't_fwd': t_fwd
                        })
                        print(f"Punto Alcanzable v={v:.2f}, w={w:.2f}")
                        current_x, current_y = x, y
                        current_theta = target_theta
                        break
                    
                except ValueError:
                    print("Entrada invalida")

    # Publicar trayectoria
    def publish_path(self):
        self.get_logger().info("Publicando trayectoria...")
        for i, p in enumerate(self.targets):
            msg = CustomPose()
            
            # Posicion
            msg.pose.position.x = float(p['x'])
            msg.pose.position.y = float(p['y'])

            # Velocidades y Tiempos
            msg.req_v = float(p['cmd_v'])
            msg.req_w = float(p['cmd_w'])
            msg.t_turn = float(p['t_turn'])
            msg.t_fwd = float(p['t_fwd'])

            self.pub.publish(msg)

            self.get_logger().info(
                f"P{i+1}: v={p['cmd_v']:.2f}, w={p['cmd_w']:.2f}, "
                f"t_turn={p['t_turn']:.2f}, t_fwd={p['t_fwd']:.2f}"
            )

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()