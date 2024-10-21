import rclpy
import numpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition
from geometry_msgs.msg import Quaternion, TwistStamped
# from window_detection import WindowDetection
from nav_msgs.msg import Path

class TrajectroyFollower(Node):
    def __init__(self):
        super().__init__("trajectory_follower")
        self.get_logger().info("Started Trajectory Follower Node ...")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # self.window_sub = self.create_subscription(WindowDetection, "/window_detections", self.window_callback, 10)
        # self.path_sub = self.create_subscription(Path, "/path_plan", self.path_callback, 10)
        self.local_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position, qos_profile)
        self.globa_sub = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_position, qos_profile)

        self.vel_pub = self.create_publisher(TwistStamped, "/px4_driver/cmd_vel/Path", 10)
        
        self.position = VehicleGlobalPosition()

        # X variables
        self.px_gain = 0.3
        self.dx_gain = 0.03
        self.nx_filter = 0.03
        self.x = 0.0 # current x
        self.x_goal = 100.0 # next x
        self.x_error = 0.0
        self.x_error_1 = 0.0
        self.x_1 = 0.0 # last x
        self.x_output = 0.0 # current x out
        self.x_output_1 = 0.0 # last x out

        # Y variables
        self.py_gain = 0.3
        self.dy_gain = 0.03
        self.ny_filter = 0.03
        self.y = 0.0 # current y
        self.y_goal = 100.0 # next y
        self.y_error = 0.0
        self.y_error_1 = 0.0
        self.y_1 = 0.0 # last y
        self.y_output = 0.0 # current y out
        self.y_output_1 = 0.0 # last y out

        # Yaw variables
        self.pyaw_gain = 0.01
        self.dyaw_gain = 0.001
        self.nyaw_filter = 0.001
        self.yaw = 0.0 # current yaw
        self.yaw_goal = 0.0 # next yaw
        self.yaw_error = 0.0
        self.yaw_error_1 = 0.0
        self.yaw_1 = 0.0 # last yaw
        self.yaw_output = 0.0 # current yaw out
        self.yaw_output_1 = 0.0 # last yaw out

        self.fact_conv = 111139

        # limits for window detection
        self.lim_rad = 1
        self.lim_yaw = 5
        self.lim_area = 500 # no se que limite poner 
        self.window_flag = 0
        
        self.q1 = Quaternion()
        self.q2 = Quaternion()

        self.ts = 0.064
        self.heartbeat_timer = self.create_timer(self.ts, self.control)

    """
    def window_callback(self, msg):
        window_index = 0
        if len(msg.windows) > 0: 
            for i in range(0, len(msg.windows)):
                if msg.windows[i].area >= self.lim_area:
                    window_index = i
                    self.get_logger().info(f"Window area: {msg.windows[window_index].area}")
                    self.window_flag = 1
                    break
                else:
                    self.window_flag = 0
    
    def path_callback(self, msg):
        self.x_goal = msg.poses.position.x*self.fact_conv # lat
        self.y_goal = msg.poses.position.y*self.fact_conv # long
        self.q2 = msg.pose.orientation
        _, _, angle = self.euler_from_quaternion(self.q2)
        angle = math.degrees(angle)
        self.yaw_goal = angle
    """

    def vehicle_global_position(self, msg):
        # self.get_logger().info(f"Latitud recibida: {msg.lat}, tipo: {type(msg.lat)}")
        # self.get_logger().info(f"Factor de conversión: {self.fact_conv}")
        # latitud y longitud actual
        self.x = msg.lat*self.fact_conv
        self.y = msg.lon*self.fact_conv
    
    def vehicle_local_position(self, msg):
        # orientacion actual
        self.yaw = msg.heading

    def control(self):
        msg = TwistStamped()

        self.get_logger().info("Entering Trajectory control")
        self.x_error = self.x_goal - self.x
        self.y_error = self.y_goal - self.y
        self.yaw_error = self.yaw_goal - self.yaw

        vel_x = 1.0
        vel_y = 1.0
        vel_yaw = 1.0

        self.get_logger().info(f"Erores: x={self.x_error}, y={self.y_error}, yaw={self.yaw_error}")

        # Control PD en X y Y 
        if (abs(self.x_error) < self.lim_rad):
            px_action = self.x_error * self.px_gain
            # ix_action = self.x_output_1 + self.x_error * self.ix_gain * self.ts
            dx_action = self.x_output_1 * (self.nx_filter * self.dx_gain * (self.x_error - self.x_error_1)) / (1 + self.nx_filter * self.ts)
            # self.x_output = float(px_action + ix_action + dx_action)
            vel_x = float(px_action + dx_action)

        else:
            vel_x = 0
            self.x_output = 0


        if (abs(self.y_error) < self.lim_rad):
            py_action = self.y_error * self.py_gain
            # iy_action = self.y_output_1 + self.y_error * self.iy_gain * self.ts
            dy_action = self.y_output_1 * (self.ny_filter * self.dy_gain * (self.y_error - self.y_error_1)) / (1 + self.ny_filter * self.ts)
            # self.y_output = float(py_action + iy_action + dy_action)
            vel_y = float(py_action + dy_action)

        else:
            self.y_output = 0


        if abs(self.yaw_error) > self.lim_yaw:
            # Control PD en Yaw
            pyaw_action = self.yaw_error * self.pyaw_gain
            # iyaw_action = self.yaw_output_1 + self.yaw_error * self.iyaw_gain * self.ts
            dyaw_action = self.yaw_output_1 * (self.nyaw_filter * self.dyaw_gain * (self.yaw_error - self.yaw_error_1)) / (1 + self.nyaw_filter * self.ts)
            vel_yaw = float(pyaw_action + dyaw_action)

        else:
            self.yaw_output = 0

        if abs(self.x_error) <= self.lim_rad and abs(self.y_error) <= self.lim_rad and abs(self.yaw_error) <= self.lim_yaw:
            self.get_logger().info("Vehicle is near to the next point")
            self.x_output = 0.0
            self.y_output = 0.0
            self.yaw_output = 0.0
            
        else:
            norm_vel = math.sqrt((vel_x**2) + (vel_y**2))
            self.x_output = vel_x / norm_vel
            self.y_output = vel_y / norm_vel
            self.yaw_output = vel_yaw 

            self.x_1 = self.x # last x
            self.x_output_1 = self.x_output # last x out

            self.y_1 = self.y # last y
            self.y_output_1 = self.y_output # last y out

            self.yaw_1 = self.yaw # last yaw
            self.yaw_output_1 = self.yaw_output # last yaw out

        msg.twist.linear.x = self.x_output
        msg.twist.linear.y = self.y_output
        msg.twist.angular.z = self.yaw_output
        self.get_logger().info(f"Velocity published: x={self.x_output}, y={self.y_output}, yaw={self.yaw_output}")

        self.vel_pub.publish(msg)

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
        

def main(args=None):
    rclpy.init(args=args)
    node = TrajectroyFollower()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)