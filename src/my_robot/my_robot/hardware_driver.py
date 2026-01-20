#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial, struct, math, smbus, time

# --- ROBOT CONSTANTS ---
WHEEL_DIA = 0.165
TICKS_PER_REV = 2922
METERS_PER_TICK = (math.pi * WHEEL_DIA) / TICKS_PER_REV
WHEEL_BASE = 0.40 

class HardwareDriver(Node):
    def __init__(self):
        super().__init__('hardware_driver')
        
        # --- 1. MOTORS ---
        # Using your FTDI path
        self.motor_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10MMVWH-if00-port0'
        self.get_logger().info(f"Connecting to Motors on {self.motor_port}")
        try:
            self.ser = serial.Serial(self.motor_port, 115200, timeout=0.01)
            self.get_logger().info("Serial Port Opened Successfully!")
        except Exception as e:
            self.get_logger().error(f"FATAL: Could not open serial port: {e}")

        # --- 2. IMU ---
        self.z_bias = 0.0
        try:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(0x68, 0x6B, 0)
            self.calibrate_imu()
        except:
            self.get_logger().warn("IMU not found (Running without it)")

        # --- 3. ROS ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.02, self.update_loop)

        # Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.enc_l = 0
        self.enc_r = 0
        self.prev_l = None
        self.prev_r = None
        self.buffer = bytearray()
        self.last_time = self.get_clock().now()

        # Debug Counters
        self.debug_counter = 0

    def calibrate_imu(self):
        total = 0
        for _ in range(50):
            total += self.read_imu_raw()
            time.sleep(0.01)
        self.z_bias = total / 50.0

    def read_imu_raw(self):
        try:
            h = self.bus.read_byte_data(0x68, 0x47)
            l = self.bus.read_byte_data(0x68, 0x48)
            v = (h << 8) + l
            return v - 65536 if v > 32768 else v
        except: return 0

    def get_signed_delta(self, current, previous):
        if previous is None: return 0
        delta = (current - previous) & 0xFFFF
        if delta > 0x7FFF: delta -= 0x10000
        return delta

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x * 200 
        angular = msg.angular.z * 100
        left = int(linear - angular)
        right = int(linear + angular)
        self.send_motor_cmd(left, right)

    def send_motor_cmd(self, v1, v2):
        v1 = max(min(v1, 800), -800)
        v2 = max(min(v2, 800), -800)
        frame = bytearray([0xAA, 0x55, 0x01, 0x03, 0x04,
                           (v1>>8)&0xFF, v1&0xFF, (v2>>8)&0xFF, v2&0xFF, 0x02, 0x00])
        chk = sum(frame) % 256
        frame += bytes([chk, 0xCC])
        try: self.ser.write(frame)
        except: pass

    def check_feedback(self):
        # 1. READ
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(data)

        # 2. PARSE (EXACT FINAL.PY LOGIC)
        while len(self.buffer) >= 36:
            # Check for Header exactly like final.py does
            if self.buffer[0] == 0xAA and self.buffer[1] == 0x55:
                if self.buffer[3] == 0x81:
                    frame = self.buffer[:36]
                    try:
                        new_l = struct.unpack('>h', frame[5:7])[0]
                        new_r = struct.unpack('>h', frame[7:9])[0]
                        self.enc_l = new_l
                        self.enc_r = new_r
                    except Exception as e:
                        self.get_logger().warn(f"Parse Fail: {e}")
                    
                    self.buffer = self.buffer[36:]
                else:
                    # Header matches, but not Feedback packet (skip header)
                    self.buffer.pop(0)
            else:
                # No header, pop 1 byte (Slow but matches final.py)
                self.buffer.pop(0)

    def euler_to_quat(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def update_loop(self):
        self.check_feedback()
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- 1. ODOMETRY CALCULATION ---
        if self.prev_l is not None:
            dL = self.get_signed_delta(self.enc_l, self.prev_l)
            dR = self.get_signed_delta(self.enc_r, self.prev_r)

            dist_l = dL * METERS_PER_TICK
            dist_r = dR * METERS_PER_TICK
            dist = (dist_l + dist_r) / 2.0
            
            # --- Fix Calculate Theta from Encoders instead of IMU ---
            # This uses the difference between wheels to determine turning
            delta_th = (dist_r - dist_l) / WHEEL_BASE
            
            self.th += delta_th
            self.x += dist * math.cos(self.th)
            self.y += dist * math.sin(self.th)
            
            # Normalize Theta to -pi to +pi range
            self.th = (self.th + math.pi) % (2 * math.pi) - math.pi
            
            vx = dist / dt if dt > 0 else 0.0
            vth = delta_th / dt if dt > 0 else 0.0
        else:
            vx = 0.0
            vth = 0.0

        self.prev_l = self.enc_l
        self.prev_r = self.enc_r

        # --- 2. PUBLISH TF ---
        q = self.euler_to_quat(0, 0, self.th)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # --- 3. PUBLISH ODOM TOPIC ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)
def main(args=None):
    rclpy.init(args=args)
    node = HardwareDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
