#!/usr/bin/env python3

import serial
import threading
import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, EmptyResponse
import signal
import sys


class SensorDriver(object):
    def __init__(self):
        # It can't disconnect serial connectivity Sometimes,
        # so we force to disconnect when keyboard interrupted
        signal.signal(signal.SIGINT, self.signal_handler)
        # ROS parameters
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud = rospy.get_param('~baud', 230400)
        self.calib_sample_num = rospy.get_param('~offset_calib_sample_number', 300)
        self.sensor_frame_id = rospy.get_param('~sensor_frame_id', 'world')
        self.debug = rospy.get_param('~debug', True)
        rospy.loginfo('port=%s, baud=%s, debug=%s', self.port, self.baud, self.debug)

        # Serial port initialization
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
        )

        # Determine packet length
        self.packet_length = self.determine_packet_length()
        if self.packet_length is None:
            rospy.logerr("Failed to determine packet length. Shutting down.")
            rospy.signal_shutdown("Invalid sensor response")
            return
        if self.debug:
            rospy.loginfo('Detected packet length: %d bytes', self.packet_length)

        # Scaling factors
        if self.packet_length == 34:
            self.scale_factors = {'Fx': 0.01, 'Fy': 0.01, 'Fz': 0.02}
        else:
            self.scale_factors = {'Fx': 0.005, 'Fy': 0.005, 'Fz': 0.01}
        if self.packet_length == 34:
            self.scale_factors.update({'Mx': 0.05, 'My': 0.05, 'Mz': 0.05})

        # State variables
        self.offsets = {'Fx': 0x0800, 'Fy': 0x0800, 'Fz': 0x0800}
        if self.packet_length == 34:
            self.offsets.update({'Mx': 0x0800, 'My': 0x0800, 'Mz': 0x0800})
        self.last_raw_vals = None
        self.last_stamp = None
        self.lock = threading.Lock()

        # ROS services and publishers
        self.calib_srv = rospy.Service('~calibrate_offset', Empty, self.calib_offset)
        self.wrench_pub = rospy.Publisher('~output/wrench', WrenchStamped, queue_size=1)

        # Perform initial calibration
        self.offsets = self.calibrate_offsets()

    def determine_packet_length(self):
        """Send 020201 and determine packet length (34 or 22 bytes)"""
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'020201')  # Request single packet
            if self.debug:
                rospy.loginfo("Sent single packet request: 020201")

            # Read up to 34 bytes with a timeout
            data = self.ser.read(34)
            if len(data) == 34:
                # Verify CR/LF for 34-byte packet
                cr, lf = data[32:34]
                if cr == 0x0D and lf == 0x0A:
                    return 34
                else:
                    rospy.logerr("Invalid CR/LF for 34-byte packet")
                    return None
            elif len(data) == 22:
                return 22
            else:
                rospy.logerr(f"Received invalid packet length: {len(data)} bytes")
                return None
        except Exception as e:
            rospy.logerr(f"Error determining packet length: {e}")
            return None
        finally:
            self.ser.reset_input_buffer()

    def parse_data(self, data, calibration_mode=False):
        """Parse byte data and calculate forces/torques or return raw values"""
        try:
            if len(data) != self.packet_length:
                rospy.logerr(f"Invalid data length: {len(data)} bytes, expected {self.packet_length} bytes")
                return None

            data_string = data.decode()
            # Address: 2 bytes
            address = data_string[0:2]

            # Binary Counter: 2 bytes, decode to hex integer
            bc_str = data_string[2:4]
            bc = int(bc_str, 16)
            if not (bc != 14 or bc != 8):
                rospy.logerr(f"Binary Counter error: {bc_str} (decimal {bc})")
                return None

            # Extract raw force values
            fx = int(data_string[4:8], 16)
            fy = int(data_string[8:12], 16)
            fz = int(data_string[12:16], 16)
            raw_vals = {'Fx': fx, 'Fy': fy, 'Fz': fz}

            if self.packet_length == 34:
                # 34-byte format: includes torque and CR/LF
                raw_vals.update({
                    'Mx': int(data_string[16:20], 16),
                    'My': int(data_string[20:24], 16),
                    'Mz': int(data_string[24:28], 16)
                })
                t_hex = data_string[28:32]
                t = int(t_hex, 16)  # Convert T to decimal
                cr, lf = data[32:34]
                if cr != 0x0D or lf != 0x0A:
                    rospy.logerr(f"CR/LF error: {cr} {lf}")
                    return None
            else:
                # 22-byte format: no torque, no CR/LF
                t_hex = data_string[16:22]
                t = int(t_hex, 16)  # Convert T to decimal

            if calibration_mode:
                return raw_vals

            # Apply offsets and scaling
            forces = {
                'Fx': (fx - self.offsets['Fx']) * self.scale_factors['Fx'],
                'Fy': (fy - self.offsets['Fy']) * self.scale_factors['Fy'],
                'Fz': (fz - self.offsets['Fz']) * self.scale_factors['Fz']
            }
            if self.packet_length == 34:
                forces.update({
                    'Mx': (raw_vals['Mx'] - self.offsets['Mx']) * self.scale_factors['Mx'],
                    'My': (raw_vals['My'] - self.offsets['My']) * self.scale_factors['My'],
                    'Mz': (raw_vals['Mz'] - self.offsets['Mz']) * self.scale_factors['Mz']
                })

            # Debug output (controlled by ~debug param)
            if self.debug:
                rospy.loginfo(f"Address: {address}")
                rospy.loginfo(f"Binary Counter: {bc_str} (decimal: {bc})")
                rospy.loginfo(f"Fx: {fx} -> {forces['Fx']:.2f} N")
                rospy.loginfo(f"Fy: {fy} -> {forces['Fy']:.2f} N")
                rospy.loginfo(f"Fz: {fz} -> {forces['Fz']:.2f} N")
                if self.packet_length == 34:
                    rospy.loginfo(f"Mx: {raw_vals['Mx']} -> {forces['Mx']:.2f} Nmm")
                    rospy.loginfo(f"My: {raw_vals['My']} -> {forces['My']:.2f} Nmm")
                    rospy.loginfo(f"Mz: {raw_vals['Mz']} -> {forces['Mz']:.2f} Nmm")
                rospy.loginfo(f"T: {t_hex} (decimal: {t})")
                rospy.loginfo("-" * 50)

            return forces

        except Exception as e:
            rospy.logerr(f"Data parsing error: {e}")
            return None

    def calibrate_offsets(self, dataStream=True):
        """Calibrate offsets by collecting samples from last_raw_vals"""
        rospy.loginfo("Starting calibration: collecting %d samples", self.calib_sample_num)
        raw_values = {key: [] for key in self.offsets}
        samples_collected = 0
        
        # Wait for initial data to be available
        if dataStream:
            self.ser.write(b'020202')
        else:
            while self.last_stamp is None and not rospy.is_shutdown():
                if self.debug:
                    rospy.loginfo("Waiting for initial sensor data...")
                rospy.sleep(0.1)

        # Collect samples
        last_stamp = None
        buffer = b""

        while samples_collected < self.calib_sample_num and not rospy.is_shutdown():
            if dataStream:
                data = self.ser.read(self.packet_length)
                if data:
                    buffer += data
                    while len(buffer) >= self.packet_length:
                        raw_data = self.parse_data(buffer[:self.packet_length], calibration_mode=True)
                        if raw_data:
                            for key in raw_values:
                                raw_values[key].append(raw_data[key])
                            samples_collected += 1
                        buffer = buffer[self.packet_length:]
            else:
                with self.lock:
                    if self.last_stamp != last_stamp and self.last_raw_vals is not None:
                        for key in raw_values:
                            raw_values[key].append(self.last_raw_vals[key])
                        last_stamp = self.last_stamp
                        samples_collected += 1
            rospy.sleep(0.01)  # Short sleep to avoid busy-waiting

        if self.debug:
            rospy.loginfo("Collected sample %d/%d", samples_collected, self.calib_sample_num)

        if dataStream:
            self.ser.write(b'020200')

        # Calculate averages
        new_offsets = {}
        for key in raw_values:
            if raw_values[key]:
                avg = sum(raw_values[key]) / len(raw_values[key])
                new_offsets[key] = int(avg)
            else:
                rospy.logwarn(f"No data for {key}, using default offset 0x0800")
                new_offsets[key] = 0x0800

        if self.debug:
            rospy.loginfo("Calibration complete. Offsets: %s", new_offsets)
        return new_offsets

    def calib_offset(self, req):
        """Service handler for offset calibration"""
        rospy.loginfo('[%s] Offset calibration started...', rospy.get_name())
        try:
            new_offsets = self.calibrate_offsets(False)
            with self.lock:
                self.offsets.update(new_offsets)
            rospy.loginfo('[%s] Offset calibration finished. Offsets: %s', rospy.get_name(), self.offsets)
            return EmptyResponse()
        except Exception as e:
            rospy.logerr(f"Calibration service failed: {e}")
            return EmptyResponse()

    def run(self):
        # Clear initial buffer
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Start continuous output
        self.ser.write(b'020202')
        if self.debug:
            rospy.loginfo("Sent continuous output command: 020202")

        try:
            while not rospy.is_shutdown():
                data = self.ser.read(self.packet_length)
                if data:
                    forces = self.parse_data(data)
                    if forces:
                        with self.lock:
                            self.last_raw_vals = self.parse_data(data, calibration_mode=True)
                            self.last_stamp = rospy.Time.now()

                            # Publish WrenchStamped
                            wrench_msg = WrenchStamped()
                            wrench_msg.header.stamp = self.last_stamp
                            wrench_msg.header.frame_id = self.sensor_frame_id
                            wrench_msg.wrench.force.x = forces['Fx']
                            wrench_msg.wrench.force.y = forces['Fy']
                            wrench_msg.wrench.force.z = forces['Fz']
                            if self.packet_length == 34:
                                wrench_msg.wrench.torque.x = forces['Mx'] / 1000.0  # Nmm -> Nm
                                wrench_msg.wrench.torque.y = forces['My'] / 1000.0  # Nmm -> Nm
                                wrench_msg.wrench.torque.z = forces['Mz'] / 1000.0  # Nmm -> Nm
                            self.wrench_pub.publish(wrench_msg)
                else:
                    rospy.logwarn_throttle(1, "No data received")
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
        except Exception as e:
            rospy.logerr(f"Error occurred: {e}")
        finally:
            try:
                self.ser.write(b'020200')  # Stop transmission
            except Exception:
                pass
            if self.ser.is_open:
                self.ser.close()
                rospy.loginfo("Serial port closed")

    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) to gracefully shut down"""
        rospy.loginfo("Received SIGINT, shutting down...")
        try:
            if self.ser.is_open:
                self.ser.write(b'020200')  # Stop transmission
                self.ser.close()
                rospy.loginfo("Serial port closed")
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")
        rospy.signal_shutdown("User interrupted")
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('force_torque_sensor_driver')
    app = SensorDriver()
    app.run()
