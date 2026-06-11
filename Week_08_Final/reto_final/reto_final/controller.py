import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String

# ── Máquina de estados ────────────────────────────────────────────────────────
TRACKING  = 0   # siguiendo la línea normalmente
SEARCHING = 1   # buscando la línea (perdida)
SIGN      = 2   # ejecutando comportamiento de señal

# ── Señales de tráfico reconocidas ────────────────────────────────────────────
KNOWN_SIGNS = {
    'stop',
    'roundabout',
    'forward',
    'giveway',
    'construction',
    'turnleftahead',
    'turnrightahead',
}

# ── Semáforos reconocidos ─────────────────────────────────────────────────────
KNOWN_TRAFFIC = {'ROJO', 'AMARILLO', 'VERDE'}

class Controller(Node):
    def __init__(self):
        super().__init__('lazo_cerrado_node')

        # ── Subscribers ──────────────────────────────────────────────────────
        self.odom_sub     = self.create_subscription(Odometry, 'odom',             self.odom_callback,     10)
        self.error_sub    = self.create_subscription(Float32,  'error',            self.error_callback,    10)
        self.line_sub     = self.create_subscription(Bool,     'line_detected',    self.line_cb,           10)
        self.semaforo_sub = self.create_subscription(String,   'estado_semaforo',  self.semaforo_callback, 10)
        self.senal_sub    = self.create_subscription(String,   'senal_detectada',  self.senal_callback,    10)
        self.cmd_pub      = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── Odometría ────────────────────────────────────────────────────────
        self.v_real = 0.0
        self.w_real = 0.0

        # ── Seguimiento de línea ─────────────────────────────────────────────
        self.v_ref           = 0.1
        self.Kp_vision       = 0.006
        self.error_pixeles   = 0.0
        self.last_error_time = self.get_clock().now()
        self.ERROR_TIMEOUT   = 1.0
        self.line_detected   = False

        # ── PI ───────────────────────────────────────────────────────────────
        self.dt      = 0.02
        self.Kp_v    = 1.1
        self.Ki_v    = 0.35
        self.Kp_w    = 1.3
        self.Ki_w    = 0.35
        self.e_v_int = 0.0
        self.e_w_int = 0.0

        # ── Recuperación de carril ───────────────────────────────────────────
        self.state            = TRACKING
        self.last_known_error = 0.0

        self.BACKUP_DURATION = 0.4
        self.v_backup        = -0.06
        self.w_search        = 0.5
        self.v_search        = 0.03
        self.search_period   = 1.2
        self.MAX_SEARCH_T    = 8.0

        self.search_phase = 'backup'
        self.search_timer = 0.0
        self.search_dir   = 1
        self.total_search = 0.0

        # ── Semáforo ─────────────────────────────────────────────────────────
        self.semaforo_estado = 'NINGUNO'

        # ── Señales de tránsito ──────────────────────────────────────────────
        self.current_sign = None
        self.sign_timer   = 0.0
        self.prev_state   = TRACKING

        # Duración del comportamiento por señal (s)
        self.sign_duration = {
            'stop':           3.0,
            'roundabout':     3.0,
            'forward':        5.0,
            'giveway':        3.0,
            'construction':   3.0,
            'turnleftahead':  4.0,
            'turnrightahead': 4.0,
        }

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Controlador iniciado')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.v_real = msg.twist.twist.linear.x
        self.w_real = msg.twist.twist.angular.z

    def error_callback(self, msg):
        self.error_pixeles   = msg.data
        self.last_error_time = self.get_clock().now()

    def line_cb(self, msg):
        self.line_detected = msg.data

    def semaforo_callback(self, msg: String):
        estado = msg.data
        if estado != self.semaforo_estado:
            self.get_logger().info(f'Semáforo: {self.semaforo_estado} → {estado}')
            self.semaforo_estado = estado

    def senal_callback(self, msg: String):
        senal = msg.data
        if senal == 'NINGUNO' or senal not in KNOWN_SIGNS:
            return
        if senal != self.current_sign and self.state != SIGN:
            self.get_logger().info(f'Señal detectada: {senal}')
            self.prev_state   = self.state
            self.current_sign = senal
            self.sign_timer   = 0.0
            self.state        = SIGN

    # ── Control PI ───────────────────────────────────────────────────────────
    def control_linear(self, v_ref):
        if v_ref == 0.0:
            self.e_v_int = 0.0
            return 0.0
        e             = v_ref - self.v_real
        self.e_v_int += e * self.dt
        self.e_v_int  = np.clip(self.e_v_int, -0.5, 0.5)
        return float(np.clip(self.Kp_v * e + self.Ki_v * self.e_v_int, -0.3, 0.3))

    def control_angular(self):
        w_ref = np.clip(self.error_pixeles * self.Kp_vision, -2.5, 2.5)
        if abs(w_ref) < 0.01:
            self.e_w_int = 0.0
        e_w           = w_ref - self.w_real
        self.e_w_int += e_w * self.dt
        self.e_w_int  = np.clip(self.e_w_int, -0.5, 0.5)
        return float(np.clip(self.Kp_w * e_w + self.Ki_w * self.e_w_int, -3.0, 3.0))

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _v_ref_semaforo(self):
        if self.semaforo_estado == 'ROJO':
            return 0.0
        elif self.semaforo_estado == 'AMARILLO':
            return self.v_ref * 0.5
        return self.v_ref

    def _reset_search(self):
        self.search_phase = 'backup'
        self.search_timer = 0.0
        self.total_search = 0.0
        self.search_dir   = 1 if self.last_known_error >= 0 else -1

    def _sign_finished(self):
        self.get_logger().info(f'Señal {self.current_sign} finalizada → reanudando')
        self.current_sign = None
        self.e_v_int      = 0.0
        self.e_w_int      = 0.0
        self.state        = self.prev_state

    # ── Comportamientos de señal ──────────────────────────────────────────────
    def _execute_sign(self, cmd) -> bool:
        self.sign_timer += self.dt
        duration = self.sign_duration.get(self.current_sign, 0.0)

        if self.current_sign == 'stop':
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        elif self.current_sign == 'giveway':
            cmd.linear.x  = 0.03
            cmd.angular.z = 0.0

        elif self.current_sign == 'turnleftahead':
            cmd.linear.x  = 0.09
            cmd.angular.z = 0.36

        elif self.current_sign == 'turnrightahead':
            cmd.linear.x  = 0.09
            cmd.angular.z = -0.36

        elif self.current_sign == 'roundabout':
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        elif self.current_sign == 'forward':
            cmd.linear.x  = self.v_ref
            cmd.angular.z = 0.0

        elif self.current_sign == 'construction':
            cmd.linear.x  = 0.03
            cmd.angular.z = 0.0

        if duration <= 0.0:
            return False
        return self.sign_timer < duration

    # ── Loop principal ────────────────────────────────────────────────────────
    def control_loop(self):
        cmd = Twist()

        if self.line_detected:
            self.last_known_error = self.error_pixeles

        # ESTADO: SIGN
        if self.state == SIGN:
            if not self._execute_sign(cmd):
                self._sign_finished()
            self.cmd_pub.publish(cmd)
            self.get_logger().debug(
                f'[SIGN:{self.current_sign}] t={self.sign_timer:.2f}s '
                f'v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}'
            )
            return

        # TRANSICIÓN TRACKING ↔ SEARCHING
        if self.line_detected:
            if self.state == SEARCHING:
                self.get_logger().info('Línea recuperada → TRACKING')
                self.e_v_int = 0.0
                self.e_w_int = 0.0
            self.state = TRACKING
        else:
            if self.state == TRACKING:
                self.get_logger().info('Línea perdida → SEARCHING')
                self._reset_search()
            self.state = SEARCHING

        # v_ref según semáforo (aplica a TRACKING y SEARCHING)
        v_ref = self._v_ref_semaforo()

        # ESTADO: TRACKING
        if self.state == TRACKING:
            elapsed = (self.get_clock().now() - self.last_error_time).nanoseconds * 1e-9
            if elapsed > self.ERROR_TIMEOUT:
                self.error_pixeles = 0.0
            cmd.linear.x  = self.control_linear(v_ref)
            cmd.angular.z = self.control_angular()

        # ESTADO: SEARCHING  (backup → turn → creep → turn → …)
        else:
            self.search_timer += self.dt
            self.total_search += self.dt

            if self.total_search >= self.MAX_SEARCH_T:
                self.get_logger().warn('Timeout de búsqueda: robot detenido.')
                self.cmd_pub.publish(cmd)
                return

            # Si el semáforo está en rojo, no moverse aunque busquemos
            if v_ref == 0.0:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0

            elif self.search_phase == 'backup':
                cmd.linear.x  = self.v_backup
                cmd.angular.z = 0.0
                if self.search_timer >= self.BACKUP_DURATION:
                    self.get_logger().info('Búsqueda: retroceso listo → girando')
                    self.search_phase = 'turn'
                    self.search_timer = 0.0

            elif self.search_phase == 'turn':
                cmd.linear.x  = 0.0
                cmd.angular.z = self.search_dir * self.w_search
                if self.search_timer >= self.search_period:
                    self.search_dir  *= -1
                    self.search_phase = 'creep'
                    self.search_timer = 0.0
                    self.get_logger().info(
                        f'Búsqueda: avanzando hacia '
                        f'{"izquierda" if self.search_dir > 0 else "derecha"}'
                    )

            elif self.search_phase == 'creep':
                cmd.linear.x  = self.v_search
                cmd.angular.z = self.search_dir * (self.w_search * 0.5)
                if self.search_timer >= self.search_period:
                    self.search_phase = 'turn'
                    self.search_timer = 0.0
                    self.get_logger().info('Búsqueda: sin línea → volviendo a girar')

        self.cmd_pub.publish(cmd)
        self.get_logger().debug(
            f'[{("TRACKING","SEARCHING","SIGN")[self.state]}] '
            f'sem={self.semaforo_estado} '
            f'phase={self.search_phase if self.state == SEARCHING else "-"} '
            f'v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
