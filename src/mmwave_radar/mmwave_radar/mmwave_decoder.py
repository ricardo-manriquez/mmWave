from typing import Optional, Tuple
import struct
from std_msgs.msg import Float32, Bool, Int32, Float32MultiArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
from .mmwave_defs import *

class MMWaveDecoder:
    def __init__(self, publishers, filter_check):
        """
        Initialize decoder with a dictionary of publishers
        publishers: Dictionary containing all ROS2 publishers
        """
        self.pubs = publishers
        self.filter_check = filter_check
        
    def decode_packet(self, data: bytes):
        try:
            if not (data.startswith(HEADER) and data.endswith(FOOTER)):
                return None
                
            control = data[2]
            command = data[3]
            
            # Basic sensor reports (control 0x01)
            if control == CONTROL_BASIC:
                if command == 0x01:  # REPORT_PING_PACKET
                    self._publish_string('ping', "Pong")
            
            # Human presence reports (control 0x80)
            elif control == CONTROL_PRESENCE:
                self._handle_presence_reports(command, data)
            
            # Breathing reports (control 0x81)
            elif control == CONTROL_BREATHING:
                self._handle_breathing_reports(command, data)
            
            # Sleep monitoring reports (control 0x84)
            elif control == CONTROL_SLEEP:
                self._handle_sleep_reports(command, data)
            
            # Heart rate reports (control 0x85)
            elif control == CONTROL_HEARTRATE:
                self._handle_heartrate_reports(command, data)
            
        except Exception as e:
            print(f'Decoding error: {e}')

    def _handle_presence_reports(self, command, data):
        if command == 0x01:  # REPORT_HUMAN_PRESENCE
            self._publish_bool('presence', bool(data[6]))
        elif command == 0x02:  # REPORT_HUMAN_SPORTS_INFORMATION
            self._publish_string('activity', MessageTypes.ACTIVITY_LEVELS[data[6]])
        elif command == 0x03:  # REPORT_HUMAN_BODY_MOVEMENT
            self._publish_int('movement', data[6])
        elif command == 0x04:  # REPORT_HUMAN_DISTANCE
            distance = struct.unpack(">h", data[6:8])[0]
            self._publish_int('distance', distance)
        elif command == 0x05:  # REPORT_HUMAN_POSITION
            x, y, z = struct.unpack(">hhh", data[6:12])
            self._publish_position(
                (x & 0x7fff) * (-1 if x & 0x8000 else 1),
                (y & 0x7fff) * (-1 if y & 0x8000 else 1),
                (z & 0x7fff) * (-1 if z & 0x8000 else 1)
            )

    def _handle_breathing_reports(self, command, data):
        if command == 0x01:  # REPORT_BREATHING_INFORMATION
            self._publish_string('breathing', MessageTypes.BREATHING_TYPES[data[6]-1])
        elif command == 0x02:  # REPORT_RESPIRATORY_RATE
            self._publish_int('respiratory_rate', data[6])
        elif command == 0x05:  # REPORT_RESPIRATORY_WAVEFORM
            self._publish_array('respiratory_waveform', list(data[6:11]))

    def _handle_sleep_reports(self, command, data):
        if command == 0x01:  # REPORT_BED_OCCUPATION
            self._publish_string('bed_status', MessageTypes.BED_STATUS[data[6]])
        elif command == 0x02:  # REPORT_SLEEP_STATE
            self._publish_string('sleep_state', MessageTypes.SLEEP_STATES[data[6]])
        elif command == 0x03:  # REPORT_AWAKE_TIME
            length = struct.unpack(">h", data[6:8])[0]
            self._publish_int('awake_time', length)
        elif command == 0x0C:  # REPORT_SLEEP_STATUS
            self._publish_sleep_status(data)
        elif command == 0x0D:  # REPORT_SLEEP_QUALITY_ANALYSIS
            self._publish_sleep_analysis(data)
        elif command == 0x0E:  # REPORT_ABNORMAL_SLEEP
            self._publish_string('sleep_abnormality', MessageTypes.SLEEP_ABNORMAL[data[6]])
        elif command == 0x10:  # REPORT_SLEEP_QUALITY_RATING
            self._publish_string('sleep_quality', MessageTypes.SLEEP_QUALITY[data[6]])
        elif command == 0x11:  # REPORT_ABNORMAL_STRUGGLING
            self._publish_string('struggling', MessageTypes.STRUGGLE_STATUS[data[6]])

    def _handle_heartrate_reports(self, command, data):
        if command == 0x02:  # REPORT_HEARTRATE
            self._publish_float('heartrate', float(data[6]))
        elif command == 0x05:  # REPORT_HEARTRATE_WAVEFORM
            self._publish_array('heartrate_waveform', list(data[6:11]))

    # Helper methods for publishing
    def _publish_bool(self, topic, value):
        if topic in self.pubs and self.filter_check(topic):
            msg = Bool()
            msg.data = value
            self.pubs[topic].publish(msg)

    def _publish_int(self, topic, value):
        if topic in self.pubs and self.filter_check(topic):
            msg = Int32()
            msg.data = value
            self.pubs[topic].publish(msg)

    def _publish_float(self, topic, value):
        if topic in self.pubs and self.filter_check(topic):
            msg = Float32()
            msg.data = value
            self.pubs[topic].publish(msg)

    def _publish_string(self, topic, value):
        if topic in self.pubs and self.filter_check(topic):
            msg = String()
            msg.data = value
            self.pubs[topic].publish(msg)

    def _publish_array(self, topic, values):
        if topic in self.pubs and self.filter_check(topic):
            msg = Float32MultiArray()
            msg.data = values
            self.pubs[topic].publish(msg)

    def _publish_position(self, x, y, z):
        if 'position' in self.pubs and self.filter_check('position'):
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            self.pubs['position'].publish(msg)

    def _publish_sleep_status(self, data):
        if 'sleep_status' in self.pubs and self.filter_check('sleep_status'):
            status = {
                'occupation': bool(data[6]),
                'status': MessageTypes.SLEEP_STATES[data[7]],
                'rr_avg': data[8],
                'hr_avg': data[9],
                'turnover': data[10],
                'large_mov': data[11],
                'small_mov': data[12],
                'apnea': data[13]
            }
            # Convert to ROS message based on your needs
            self._publish_string('sleep_status', str(status))

    def _publish_sleep_analysis(self, data):
        if 'sleep_analysis' in self.pubs and self.filter_check('sleep_analysis'):
            analysis_data = struct.unpack(">BH9B", data[6:18])
            analysis = {
                'quality': analysis_data[0],
                'sleep_time': analysis_data[1],
                'awake_time': analysis_data[2],
                'light_sleep_time': analysis_data[3],
                'deep_sleep_time': analysis_data[4],
                'out_of_bed_time': analysis_data[5],
                'out_of_bed_times': analysis_data[6],
                'turnovers': analysis_data[7],
                'rr_avg': analysis_data[8],
                'hr_avg': analysis_data[9]
            }
            # Convert to ROS message based on your needs
            self._publish_string('sleep_analysis', str(analysis))

    def find_complete_packet(self, data: bytes) -> Tuple[Optional[bytes], bytes]:
        start = data.find(HEADER)
        if start == -1:
            return None, data
        
        end = data.find(FOOTER, start)
        if end == -1:
            return None, data
        
        packet = data[start:end+2]
        remaining = data[end+2:]
        
        return packet, remaining