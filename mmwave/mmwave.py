#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32, Bool, Int32, String, Int32MultiArray
from ros_healthcare_msg.msg import HR, RR, HRheader, RRheader
from geometry_msgs.msg import Point

from .driver.commands import commands
from .driver.util import checksum, check_pkt, fill_checksum
import struct
import select

class MMWaveNode(Node):
    def __init__(self, ser):
        super().__init__('mmwave_node')

        # init active filters
        self.active_filters = set()  # null set means all topics are active

        # create publishers
        self._publishers = {
            'heartrate': self.create_publisher(HR, 'mmwave/heartrate', 10),
            'heartrate_waveform': self.create_publisher(Int32MultiArray, 'mmwave/heartrate_waveform', 10),
            'presence': self.create_publisher(Bool, 'mmwave/presence', 10),
            'distance': self.create_publisher(Int32, 'mmwave/distance', 10),
            'position': self.create_publisher(Point, 'mmwave/position', 10),
            'movement': self.create_publisher(Int32, 'mmwave/movement', 10),
            'activity': self.create_publisher(String, 'mmwave/activity', 10),
            'breathing': self.create_publisher(String, 'mmwave/breathing', 10),
            'respiratory_rate': self.create_publisher(RR, 'mmwave/respiratory_rate', 10),
            'respiratory_waveform': self.create_publisher(Int32MultiArray, 'mmwave/respiratory_waveform', 10),
            'bed_status': self.create_publisher(String, 'mmwave/bed_status', 10),
            'sleep_state': self.create_publisher(String, 'mmwave/sleep_state', 10),
            'sleep_status': self.create_publisher(String, 'mmwave/sleep_status', 10),
            'sleep_analysis': self.create_publisher(String, 'mmwave/sleep_analysis', 10),
            'sleep_quality': self.create_publisher(String, 'mmwave/sleep_quality', 10),
            'struggling': self.create_publisher(String, 'mmwave/struggling', 10),
            'sleep_abnormality': self.create_publisher(String, 'mmwave/sleep_abnormality', 10),
        }

        # create parameter to set filters
        self.declare_parameter('filters', '')

        self.add_on_set_parameters_callback(self.check_filter_parameter)

        self.cmd_dict = {}
        self.cmds = {}
        self.dev = None
        self.con = None
        self.callback = None
        self._filter = []
        self.stack = {}
        for command in commands:
            if not command.control in self.cmd_dict:
                self.cmd_dict[command.control] = {}
            self.cmd_dict[command.control][command.command] = command()
            self.cmds[command.__name__] = self.cmd_dict[command.control][command.command]


        # serial port
        self.ser = ser
        
        self.hr_seq = 0
        self.rr_seq = 0

    def check_filter_parameter(self):
        """check if the filter parameter has been updated"""
        filters_str = self.get_parameter('filters').value
        if filters_str:
            new_filters = set(filters_str.split(','))
            if new_filters != self.active_filters:
                self.active_filters = new_filters
                self.get_logger().info(f'Updated filters: {self.active_filters}')
        return SetPArametersResult(successful=True)

    def is_topic_active(self, topic_name: str) -> bool:
        """check if a topic is active based on the current filters"""
        # if no filters are active, all topics are active
        if not self.active_filters or FilterCategories.ALL in self.active_filters:
            return True

        # check if the topic is in any of the active filter categories
        for filter_category in self.active_filters:
            if filter_category in TOPIC_GROUPS and topic_name in TOPIC_GROUPS[filter_category]:
                return True
        return False

    def set_callback(self, callback):
        self.callback = callback

    def interpret_package(self, pkt):
        try:
            cmd = self.cmd_dict[pkt[2].to_bytes(1, "little")][pkt[3].to_bytes(1, "little")]
            response = None
            if cmd.name() in self.stack:
                self.stack[cmd.name()].set()
                self.stack[cmd.name()] = cmd.process(pkt)
            else:
                if hasattr(cmd, "process"):
                    response = cmd.process(pkt)
                else:
                    response = repr(pkt)
            if self.callback:
                if self._filter:
                    if type(cmd) in self._filter:
                        if response:
                            self.callback(cmd, response)
                else:
                    if response:
                        self.callback(cmd, response)
        except:
            print(pkt)
            raise

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

    def command_callback(self, cmd, response):
        #print(cmd, response)
        if type(cmd) == type(self.cmds["REPORT_HUMAN_PRESENCE"]):
            self._publish_bool('presence', response["presence"])
        elif type(cmd) == type(self.cmds["REPORT_HUMAN_SPORTS_INFORMATION"]):
            self._publish_string('activity', response["activity"])
        elif type(cmd) == type(self.cmds["REPORT_HUMAN_BODY_MOVEMENT"]):
            self._publish_int('movement', response["movement"])
        elif type(cmd) == type(self.cmds["REPORT_HUMAN_DISTANCE"]):
            self._publish_int('distance', response["distance"])
        elif type(cmd) == type(self.cmds["REPORT_HUMAN_POSITION"]):
            self._publish_position(response["x"], response["y"], response["z"])
        elif type(cmd) == type(self.cmds["REPORT_BREATHING_INFORMATION"]):
            self._publish_string('breathing', response["breathing"])
        elif type(cmd) == type(self.cmds["REPORT_RESPIRATORY_RATE"]):
            self._publish_rr('respiratory_rate', response["respiratoryrate"])
        elif type(cmd) == type(self.cmds["REPORT_RESPIRATORY_WAVEFORM"]):
            self._publish_array('respiratory_waveform', response["waveform"])
        elif type(cmd) == type(self.cmds["REPORT_BED_OCCUPATION"]):
            self._publish_string('bed_status', response["occupation"])
        elif type(cmd) == type(self.cmds["REPORT_ABNORMAL_SLEEP"]):
            self._publish_string('sleep_abnormality', response["abnormality"])
        elif type(cmd) == type(self.cmds["REPORT_SLEEP_QUALITY_RATING"]):
            self._publish_string('sleep_quality', response["quality"])
        elif type(cmd) == type(self.cmds["REPORT_ABNORMAL_STRUGGLING"]):
            self._publish_string('struggling', response["struggling"])
        elif type(cmd) == type(self.cmds["REPORT_HEARTRATE"]):
            self._publish_hr('heartrate', response["heartrate"])
        elif type(cmd) == type(self.cmds["REPORT_HEARTRATE_WAVEFORM"]):
            self._publish_array('heartrate_waveform', response["waveform"])
        elif type(cmd) == type(self.cmds["REPORT_SLEEP_STATE"]):
            self._publish_string('sleep_state', response["state"])
        elif type(cmd) == type(self.cmds["REPORT_AWAKE_TIME"]):
            self._publish_int('awake_time', response["length"])
        elif type(cmd) == type(self.cmds["REPORT_SLEEP_STATUS"]):
            self._publish_sleep_status(response)
        elif type(cmd) == type(self.cmds["REPORT_SLEEP_QUALITY_ANALYSIS"]):
            self._publish_sleep_analysis(response)
        elif type(cmd) == type(self.cmds["REPORT_ABNORMAL_SLEEP"]):
            self._publish_string('sleep_abnormality', response["abnormality"])

    # Helper methods for publishing
    def _publish_bool(self, topic, value):
        if topic in self._publishers and topic not in self.active_filters:
            msg = Bool()
            msg.data = value
            self._publishers[topic].publish(msg)

    def _publish_int(self, topic, value):
        if topic in self._publishers and topic not in self.active_filters:
            msg = Int32()
            msg.data = value
            self._publishers[topic].publish(msg)

    def _publish_hr(self, topic, value):
        if topic in self._publishers and topic not in self.active_filters:
            self.hr_seq += 1
            msg = HR()
            msg.hr = float(value)
            msg.hr_quality = 0
            header = HRheader()
            header.device_serial_number = "0"
            header.unit = "bpm"
            header.sampling_frequency = 0.33
            header.resolution = 1
            header.accuracy = 0.85
            header.max_range = 100
            header.min_range = 0
            header.stride_length = 0
            header.window_size = 0
            header.single_refresh_rate = 0.33
            header.header.seq = self.hr_seq
            header.header.stamp = self.get_clock().now().to_msg()
            msg.header = header
            self._publishers[topic].publish(msg)

    def _publish_rr(self, topic, value):
        if topic in self._publishers and topic not in self.active_filters:
            self.rr_seq += 1
            msg = RR()
            msg.rr = float(value)
            msg.rr_quality = 0
            header = RRheader()
            header.device_serial_number = "0"
            header.unit = "bpm"
            header.sampling_frequency = 0.33
            header.resolution = 1
            header.accuracy = 0.9
            header.max_range = 25
            header.min_range = 0
            header.stride_length = 0
            header.window_size = 0
            header.single_refresh_rate = 0.33
            header.header.seq = self.rr_seq
            header.header.stamp = self.get_clock().now().to_msg()
            msg.header = header
            self._publishers[topic].publish(msg)

    def _publish_float(self, topic, value):
        if topic in self._publishers and topic not in self.active_filters:
            msg = Float32()
            msg.data = value
            self._publishers[topic].publish(msg)

    def _publish_string(self, topic, value):
        if topic in self._publishers and topic not in self.active_filters:
            msg = String()
            msg.data = value
            self._publishers[topic].publish(msg)

    def _publish_array(self, topic, values):
        if topic in self._publishers and topic not in self.active_filters:
            msg = Int32MultiArray()
            msg.data = values
            self._publishers[topic].publish(msg)

    def _publish_position(self, x, y, z):
        if 'position' in self._publishers and 'position' not in self.active_filters:
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            self._publishers['position'].publish(msg)

    def _publish_sleep_status(self, data):
        if 'sleep_status' in self._publishers and 'sleep_status' not in self.active_filters:
            self._publish_string('sleep_status', str(data))

    def _publish_sleep_analysis(self, data):
        if 'sleep_analysis' in self._publishers and 'sleep_analysis' not in self.active_filters:
            self._publish_string('sleep_analysis', str(data))


def main(args=None):
    rclpy.init(args=args)

    ser = serial.Serial(
        port='/dev/ttyACM2',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

    node = MMWaveNode(ser)
    node.set_callback(node.command_callback)
    ser.write(b"0")
    while True:
        rclpy.spin_once(node, timeout_sec=0)
        r, w, x = select.select((ser,),(),(), 0.1)
        if ser in r:
            b = ser.read(6)
            if b[0:2] == b'\x53\x59':
                m_len = struct.unpack(">h", b[4:6])[0] + 1
                b += ser.read(m_len + 2)
                if check_pkt(b):
                    node.interpret_package(b)
            else:
                b = ser.read_until(b"\x54\x43")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
