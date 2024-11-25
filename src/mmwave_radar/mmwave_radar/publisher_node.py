#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32, Bool, Int32, String, Float32MultiArray
from geometry_msgs.msg import Point
from .mmwave_decoder import MMWaveDecoder
from .mmwave_defs import FilterCategories, TOPIC_GROUPS

class MMWaveNode(Node):
    def __init__(self):
        super().__init__('mmwave_node')
        
        # init active filters
        self.active_filters = set()  # null set means all topics are active
        
        # create publishers
        self._publishers = {
            'heartrate': self.create_publisher(Float32, 'mmwave/heartrate', 10),
            'heartrate_waveform': self.create_publisher(Float32MultiArray, 'mmwave/heartrate_waveform', 10),
            'presence': self.create_publisher(Bool, 'mmwave/presence', 10),
            'distance': self.create_publisher(Int32, 'mmwave/distance', 10),
            'position': self.create_publisher(Point, 'mmwave/position', 10),
            'movement': self.create_publisher(Int32, 'mmwave/movement', 10),
            'activity': self.create_publisher(String, 'mmwave/activity', 10),
            'breathing': self.create_publisher(String, 'mmwave/breathing', 10),
            'respiratory_rate': self.create_publisher(Int32, 'mmwave/respiratory_rate', 10),
            'respiratory_waveform': self.create_publisher(Float32MultiArray, 'mmwave/respiratory_waveform', 10),
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
        
        # create timer to check filter parameter
        self.create_timer(1.0, self.check_filter_parameter)
        
        # init decoder, pass publishers and filter check function
        self.decoder = MMWaveDecoder(self._publishers, self.is_topic_active)
        
        # init serial port
        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.buffer = b''

    def check_filter_parameter(self):
        """check if the filter parameter has been updated"""
        filters_str = self.get_parameter('filters').value
        if filters_str:
            new_filters = set(filters_str.split(','))
            if new_filters != self.active_filters:
                self.active_filters = new_filters
                self.get_logger().info(f'Updated filters: {self.active_filters}')

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

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)
            
            while True:
                packet, self.buffer = self.decoder.find_complete_packet(self.buffer)
                if not packet:
                    break
                    
                self.decoder.decode_packet(packet)

    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = MMWaveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()