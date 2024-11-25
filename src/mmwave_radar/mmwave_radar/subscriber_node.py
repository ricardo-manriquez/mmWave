#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32, String, Float32MultiArray
from geometry_msgs.msg import Point
import sys

class MMWaveSubscriber(Node):
    def __init__(self):
        super().__init__('mmwave_subscriber')
        
        # create all subscribers
        self._subscribers = {
            'heartrate': self.create_subscription(
                Float32, 'mmwave/heartrate', self.heartrate_callback, 10),
            'heartrate_waveform': self.create_subscription(
                Float32MultiArray, 'mmwave/heartrate_waveform', self.heartrate_waveform_callback, 10),
            'presence': self.create_subscription(
                Bool, 'mmwave/presence', self.presence_callback, 10),
            'distance': self.create_subscription(
                Int32, 'mmwave/distance', self.distance_callback, 10),
            'position': self.create_subscription(
                Point, 'mmwave/position', self.position_callback, 10),
            'movement': self.create_subscription(
                Int32, 'mmwave/movement', self.movement_callback, 10),
            'activity': self.create_subscription(
                String, 'mmwave/activity', self.activity_callback, 10),
            'breathing': self.create_subscription(
                String, 'mmwave/breathing', self.breathing_callback, 10),
            'respiratory_rate': self.create_subscription(
                Int32, 'mmwave/respiratory_rate', self.respiratory_callback, 10),
            'respiratory_waveform': self.create_subscription(
                Float32MultiArray, 'mmwave/respiratory_waveform', self.respiratory_waveform_callback, 10),
            'bed_status': self.create_subscription(
                String, 'mmwave/bed_status', self.bed_status_callback, 10),
            'sleep_state': self.create_subscription(
                String, 'mmwave/sleep_state', self.sleep_state_callback, 10),
            'sleep_status': self.create_subscription(
                String, 'mmwave/sleep_status', self.sleep_status_callback, 10),
            'sleep_analysis': self.create_subscription(
                String, 'mmwave/sleep_analysis', self.sleep_analysis_callback, 10),
            'sleep_quality': self.create_subscription(
                String, 'mmwave/sleep_quality', self.sleep_quality_callback, 10),
            'struggling': self.create_subscription(
                String, 'mmwave/struggling', self.struggling_callback, 10),
            'sleep_abnormality': self.create_subscription(
                String, 'mmwave/sleep_abnormality', self.sleep_abnormality_callback, 10),
        }

    # Basic vital signs callbacks
    def heartrate_callback(self, msg):
        self.get_logger().info(f'Heart Rate: {msg.data} bpm')
        
    def heartrate_waveform_callback(self, msg):
        waveform_data = ', '.join([str(i) for i in msg.data])
        self.get_logger().info(f'Heart Rate Waveform: [{waveform_data}]')

    # Presence and position callbacks
    def presence_callback(self, msg):
        self.get_logger().info(f'Human Presence: {msg.data}')
        
    def distance_callback(self, msg):
        self.get_logger().info(f'Distance: {msg.data} cm')
        
    def position_callback(self, msg):
        self.get_logger().info(f'Position: x={msg.x}, y={msg.y}, z={msg.z}')
        
    def movement_callback(self, msg):
        self.get_logger().info(f'Movement Level: {msg.data}')
        
    def activity_callback(self, msg):
        self.get_logger().info(f'Activity Status: {msg.data}')

    # Breathing related callbacks
    def breathing_callback(self, msg):
        self.get_logger().info(f'Breathing Status: {msg.data}')
        
    def respiratory_callback(self, msg):
        self.get_logger().info(f'Respiratory Rate: {msg.data} bpm')
        
    def respiratory_waveform_callback(self, msg):
        waveform_data = ', '.join([str(i) for i in msg.data])
        self.get_logger().info(f'Respiratory Waveform: [{waveform_data}]')

    # Sleep related callbacks
    def bed_status_callback(self, msg):
        self.get_logger().info(f'Bed Status: {msg.data}')
        
    def sleep_state_callback(self, msg):
        self.get_logger().info(f'Sleep State: {msg.data}')
        
    def sleep_status_callback(self, msg):
        self.get_logger().info(f'Sleep Status: {msg.data}')
        
    def sleep_analysis_callback(self, msg):
        self.get_logger().info(f'Sleep Analysis: {msg.data}')
        
    def sleep_quality_callback(self, msg):
        self.get_logger().info(f'Sleep Quality: {msg.data}')
        
    def struggling_callback(self, msg):
        self.get_logger().info(f'Struggling Status: {msg.data}')
        
    def sleep_abnormality_callback(self, msg):
        self.get_logger().info(f'Sleep Abnormality: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = MMWaveSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print("\nShutting down mmwave subscriber...")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()