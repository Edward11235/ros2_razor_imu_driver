#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import sys
import time
import yaml
import math

from sensor_msgs.msg import Imu, MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from transforms3d.euler import euler2quat as quaternion_from_euler

from ros2_razor_imu.lib.serial_commands import (
    STOP_DATASTREAM,
    START_DATASTREAM,
    SET_TEXT_EXTENDED_FORMAT_NO_MAG,
    SET_TEXT_EXTENDED_FORMAT_WITH_MAG,
    GET_CALIBRATION_VALUES,
    SET_CALIB_ACC_X_MIN,
    SET_CALIB_ACC_X_MAX,
    SET_CALIB_ACC_Y_MIN,
    SET_CALIB_ACC_Y_MAX,
    SET_CALIB_ACC_Z_MIN,
    SET_CALIB_ACC_Z_MAX,
    SET_MAG_X_MIN,
    SET_MAG_X_MAX,
    SET_MAG_Y_MIN,
    SET_MAG_Y_MAX,
    SET_MAG_Z_MIN,
    SET_MAG_Z_MAX,
    SET_GYRO_AVERAGE_OFFSET_X,
    SET_GYRO_AVERAGE_OFFSET_Y,
    SET_GYRO_AVERAGE_OFFSET_Z,
)

LINE_START_NO_MAG = "#YPRAG="
LINE_START_WITH_MAG = "#YPRAGM="

class RazorImuDriver(Node):

    def __init__(self):
        super().__init__('razor_imu_node')

        #--- parameters ---
        self.frame_id = self.declare_parameter('frame_header', 'base_imu_link').value
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        publish_mag = self.declare_parameter('publish_magnetometer', False).value
        timer_period = self.declare_parameter('timer_period', 0.01).value  # seconds

        # calibration parameters
        self.calib = {
            'accel_x_min': self.declare_parameter('accel_x_min', -250.0).value,
            'accel_x_max': self.declare_parameter('accel_x_max',  250.0).value,
            'accel_y_min': self.declare_parameter('accel_y_min', -250.0).value,
            'accel_y_max': self.declare_parameter('accel_y_max',  250.0).value,
            'accel_z_min': self.declare_parameter('accel_z_min', -250.0).value,
            'accel_z_max': self.declare_parameter('accel_z_max',  250.0).value,
            'magn_x_min': self.declare_parameter('magn_x_min', -600.0).value,
            'magn_x_max': self.declare_parameter('magn_x_max',  600.0).value,
            'magn_y_min': self.declare_parameter('magn_y_min', -600.0).value,
            'magn_y_max': self.declare_parameter('magn_y_max',  600.0).value,
            'magn_z_min': self.declare_parameter('magn_z_min', -600.0).value,
            'magn_z_max': self.declare_parameter('magn_z_max',  600.0).value,
            'gyro_average_offset_x': self.declare_parameter('gyro_average_offset_x', 0.0).value,
            'gyro_average_offset_y': self.declare_parameter('gyro_average_offset_y', 0.0).value,
            'gyro_average_offset_z': self.declare_parameter('gyro_average_offset_z', 0.0).value,
        }
        self.imu_yaw_calibration = self.declare_parameter('imu_yaw_calibration', 0.0).value

        #--- publishers ---
        self.pub_imu  = self.create_publisher(Imu,             'imu',         1)
        self.pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics', 1)
        if publish_mag:
            self.pub_mag = self.create_publisher(MagneticField, 'mag', 1)
            self.mag_msg = MagneticField()
            self.mag_msg.magnetic_field_covariance = [0.0]*9
            self.line_start = LINE_START_WITH_MAG
        else:
            self.line_start = LINE_START_NO_MAG

        #--- prepare IMU message template ---
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id
        self.imu_msg.orientation_covariance = [0.0025, 0.0, 0.0,
                                               0.0, 0.0025, 0.0,
                                               0.0, 0.0, 0.0025]
        self.imu_msg.angular_velocity_covariance = [0.02, 0.0, 0.0,
                                                    0.0, 0.02, 0.0,
                                                    0.0, 0.0, 0.02]
        self.imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0,
                                                       0.0, 0.04, 0.0,
                                                       0.0, 0.0, 0.04]

        #--- open serial port ---
        self.get_logger().info(f'Opening serial {self.port} @115200...')
        attempts = 5
        for i in range(attempts):
            try:
                self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
                break
            except serial.serialutil.SerialException:
                self.get_logger().warn(
                    f'Cannot open {self.port}, retrying ({i+1}/{attempts})...')
                time.sleep(1)
        else:
            self.get_logger().error('Serial port failed, exiting.')
            sys.exit(1)

        # give board 5s to boot
        self.get_logger().info('Sleeping 2 seconds for IMU boot...')
        time.sleep(2)
        self.get_logger().info('Finish sleep')

        #--- configure board exactly like ROS1 ---
        self.ser.write(('#o0').encode('utf-8'))
        discard = self.ser.readlines()
        self.ser.write(('#ox').encode('utf-8'))

        self.get_logger().info('Writing calibration values...')
        self.ser.write(('#caxm' + str(self.calib['accel_x_min'])).encode('utf-8'))
        self.ser.write(('#caxM' + str(self.calib['accel_x_max'])).encode('utf-8'))
        self.ser.write(('#caym' + str(self.calib['accel_y_min'])).encode('utf-8'))
        self.ser.write(('#cayM' + str(self.calib['accel_y_max'])).encode('utf-8'))
        self.ser.write(('#cazm' + str(self.calib['accel_z_min'])).encode('utf-8'))
        self.ser.write(('#cazM' + str(self.calib['accel_z_max'])).encode('utf-8'))

        self.ser.write(('#cmxm' + str(self.calib['magn_x_min'])).encode('utf-8'))
        self.ser.write(('#cmxM' + str(self.calib['magn_x_max'])).encode('utf-8'))
        self.ser.write(('#cmym' + str(self.calib['magn_y_min'])).encode('utf-8'))
        self.ser.write(('#cmyM' + str(self.calib['magn_y_max'])).encode('utf-8'))
        self.ser.write(('#cmzm' + str(self.calib['magn_z_min'])).encode('utf-8'))
        self.ser.write(('#cmzM' + str(self.calib['magn_z_max'])).encode('utf-8'))

        self.ser.write(('#cgx' + str(self.calib['gyro_average_offset_x'])).encode('utf-8'))
        self.ser.write(('#cgy' + str(self.calib['gyro_average_offset_y'])).encode('utf-8'))
        self.ser.write(('#cgz' + str(self.calib['gyro_average_offset_z'])).encode('utf-8'))

        self.ser.flushInput()
        self.ser.write(('#p').encode('utf-8'))
        calib_data = self.ser.readlines()
        text = 'Calibration values from IMU:\n' + ''.join(r.decode('utf-8') for r in calib_data)
        self.get_logger().info(text)

        self.ser.write(('#o1').encode('utf-8'))
        self.get_logger().info('Flushing first 200 IMU entries...')
        for _ in range(200):
            self.ser.readline()

        #--- start periodic read/publish ---
        self.next_diag_time = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Razor IMU node started.')




    def timer_callback(self):
        # read a raw frame, decode safely and strip whitespace
        raw = self.ser.readline()
        line = raw.decode('utf-8', errors='ignore').strip()
        # only proceed if it really is a YPR frame
        if not line.startswith(self.line_start):
            return

        words = line[len(self.line_start):].split(',')
        # when publishing magnetometer, words has 12 entries, else 9
        if len(words) < 9:
            self.get_logger().warn('Unexpected field count')
            return

        # parse YPR
        yaw_deg  = -float(words[0]) + self.imu_yaw_calibration
        yaw_deg  = (yaw_deg + 180) % 360 - 180
        yaw      = yaw_deg * math.pi/180.0
        pitch    = -float(words[1]) * math.pi/180.0
        roll     =  float(words[2]) * math.pi/180.0

        # accelerometer → m/s²
        accel_factor = 9.806/256.0
        self.imu_msg.linear_acceleration.x = -float(words[3]) * accel_factor
        self.imu_msg.linear_acceleration.y =  float(words[4]) * accel_factor
        self.imu_msg.linear_acceleration.z =  float(words[5]) * accel_factor

        # gyro → rad/s (already in deg/s)
        self.imu_msg.angular_velocity.x =  float(words[6]) * math.pi/180.0
        self.imu_msg.angular_velocity.y = -float(words[7]) * math.pi/180.0
        self.imu_msg.angular_velocity.z = -float(words[8]) * math.pi/180.0

        # optional magnetometer
        if hasattr(self, 'pub_mag') and len(words) >= 12:
            # convert mGauss → Tesla
            self.mag_msg.magnetic_field.x =  float(words[9]) * 1e-7
            self.mag_msg.magnetic_field.y = -float(words[10])* 1e-7
            self.mag_msg.magnetic_field.z = -float(words[11])* 1e-7
            self.mag_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_mag.publish(self.mag_msg)

        # orientation quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]

        # header
        now = self.get_clock().now().to_msg()
        self.imu_msg.header.stamp = now
        self.pub_imu.publish(self.imu_msg)

        # once-a-second diagnostics
        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= self.next_diag_time:
            diag = DiagnosticArray()
            diag.header.stamp = now
            status = DiagnosticStatus()
            status.name  = 'Razor_Imu'
            status.level = DiagnosticStatus.OK
            status.message = 'AHRS measurement'
            for key, val in [('roll (deg)', roll*180/math.pi),
                             ('pitch(deg)', pitch*180/math.pi),
                             ('yaw  (deg)', yaw*180/math.pi)]:
                kv = KeyValue(key=key, value=f'{val:.2f}')
                status.values.append(kv)
            diag.status.append(status)
            self.pub_diag.publish(diag)
            # schedule next
            self.next_diag_time = now_ns + int(1e9)

    def send_command(self, command, value=None):
        cmd = command if value is None else f'{command}{value}'
        cmd += '\r'
        self.get_logger().debug(f'Serial → {cmd.strip()}')
        written = self.ser.write(cmd.encode())
        if written != len(cmd):
            self.get_logger().error('Serial write length mismatch')
        time.sleep(0.05)

    def write_and_check_config(self):
        # send accel & gyro cal
        self.send_command(SET_CALIB_ACC_X_MIN, self.calib['accel_x_min'])
        self.send_command(SET_CALIB_ACC_X_MAX, self.calib['accel_x_max'])
        self.send_command(SET_CALIB_ACC_Y_MIN, self.calib['accel_y_min'])
        self.send_command(SET_CALIB_ACC_Y_MAX, self.calib['accel_y_max'])
        self.send_command(SET_CALIB_ACC_Z_MIN, self.calib['accel_z_min'])
        self.send_command(SET_CALIB_ACC_Z_MAX, self.calib['accel_z_max'])
        # mag
        self.send_command(SET_MAG_X_MIN, self.calib['magn_x_min'])
        self.send_command(SET_MAG_X_MAX, self.calib['magn_x_max'])
        self.send_command(SET_MAG_Y_MIN, self.calib['magn_y_min'])
        self.send_command(SET_MAG_Y_MAX, self.calib['magn_y_max'])
        self.send_command(SET_MAG_Z_MIN, self.calib['magn_z_min'])
        self.send_command(SET_MAG_Z_MAX, self.calib['magn_z_max'])
        # gyro offsets
        self.send_command(SET_GYRO_AVERAGE_OFFSET_X, self.calib['gyro_average_offset_x'])
        self.send_command(SET_GYRO_AVERAGE_OFFSET_Y, self.calib['gyro_average_offset_y'])
        self.send_command(SET_GYRO_AVERAGE_OFFSET_Z, self.calib['gyro_average_offset_z'])

        # verify
        self.send_command(GET_CALIBRATION_VALUES)
        buf = ''
        for _ in range(21):
            buf += self.ser.readline().decode('utf-8').lower().replace(':', ': ')
        parsed = yaml.safe_load(buf) or {}

        for k,v in self.calib.items():
            if parsed.get(k) != v:
                self.get_logger().warn(f'Calib mismatch for {k}: expected={v}, got={parsed.get(k)}')

def main(args=None):
    rclpy.init(args=args)
    node = RazorImuDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
