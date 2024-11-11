import abc
import struct
from util import fill_checksum

HEADER = b"\x53\x59"
FOOTER = b"\x54\x43"

class BaseCommand(metaclass=abc.ABCMeta):
    control = int()
    command = int()
    report = False
    length = 1
    data = b'\x0f'

class Command(BaseCommand):
    def execute(self):
        if self.report:
            raise NotImplementedError
        msg = HEADER + self.control + self.command + struct.pack(
            ">H", self.length,
        ) + self.data + b'\x00' + FOOTER
        return fill_checksum(msg)
    def process(self, pkt):
        raise NotImplementedError
    def name(self):
        return type(self).__name__

# Basic sensor functions

class REPORT_PING_PACKET(Command):
    """Report ping packet"""
    control = b'\x01'
    command = b'\x01'
    report = True
    def process(self, pkt):
        return {"status": True, "message": "Pong"}

class CMD_RESET(Command):
    """Reset the sensor"""
    control = b'\x01'
    command = b'\x02'
    def process(self, pkt):
        return {"status": True, "message": "Reset successful"}

class CMD_PING(Command):
    """Ping the sensor"""
    control = b'\x01'
    command = b'\x80'
    def process(self, pkt):
        return {"status": True, "message": "Pong"}

class CMD_QUERY_PRODUCT_MODEL(Command):
    """Query product model"""
    control = b'\x02'
    command = b'\xA1'
    def process(self, pkt):
        m_len = struct.unpack(">h", pkt[4:6])[0] + 1
        return {"model": pkt[6:5+m_len]}

class CMD_QUERY_PRODUCT_ID(Command):
    """Query product ID"""
    control = b'\x02'
    command = b'\xA2'
    def process(self, pkt):
        m_len = struct.unpack(">h", pkt[4:6])[0] + 1
        return {"productid": pkt[6:5+m_len]}

class CMD_QUERY_HARDWARE_MODEL(Command):
    """Query hardware model"""
    control = b'\x02'
    command = b'\xA3'
    def process(self, pkt):
        m_len = struct.unpack(">h", pkt[4:6])[0] + 1
        return {"Hardware model": pkt[6:5+m_len]}

class CMD_QUERY_HARDWARE_VERSION(Command):
    """Query hardware version"""
    control = b'\x02'
    command = b'\xA4'
    def process(self, pkt):
        m_len = struct.unpack(">h", pkt[4:6])[0] + 1
        return {"version": pkt[6:5+m_len]}

class REPORT_INITIALIZATION_COMPLETE(Command):
    """Report completed initialization sequence"""
    control = b'\x05'
    command = b'\x01'
    report = True
    def process(self, pkt):
        return {"status": True, "message": "Initialization complete"}

class CMD_QUERY_INITIALIZATION_COMPLETE(Command):
    """Report completed initialization sequence"""
    control = b'\x05'
    command = b'\x81'
    def process(self, pkt):
        return {"status": True if pkt[6] == 1 else False}

class REPORT_OUT_OF_BOUNDS_STATUS(Command):
    """Report location out of bouds status"""
    control = b'\x07'
    command = b'\x07'
    report = True
    def process(self, pkt):
        return {"in-range": True if pkt[6] == 1 else False}

class CMD_QUERY_OUT_OF_BOUNDS_STATUS(Command):
    """Report location out of bouds status"""
    control = b'\x07'
    command = b'\x87'
    def process(self, pkt):
        return {"in-range": True if pkt[6] == 1 else False}

# Human presence functions

class CMD_ACTIVATE_HUMAN_PRESENCE(Command):
    """Activate human presence function"""
    control = b'\x80'
    command = b'\x00'
    def execute(self, active: bool):
        self.data = b'\x01' if active else b'\x00'
        return super().execute()
    def process(self, pkt):
        return {"active": True if pkt[6] else False}

class REPORT_HUMAN_PRESENCE(Command):
    """Report human presence"""
    control = b'\x80'
    command = b'\x01'
    report = True
    def process(self, pkt):
        return {"presence": True if pkt[6] else False}

class REPORT_HUMAN_SPORTS_INFORMATION(Command):
    """Report human activity level"""
    control = b'\x80'
    command = b'\x02'
    report = True
    def process(self, pkt):
        level = ["None", "Still", "Active"]
        return {"activity": level[pkt[6]]}

class REPORT_HUMAN_BODY_MOVEMENT(Command):
    """Report body movement level"""
    control = b'\x80'
    command = b'\x03'
    report = True
    def process(self, pkt):
        return {"movement": pkt[6]}

class REPORT_HUMAN_DISTANCE(Command):
    """Report human distance to the sensor [cm]"""
    control = b'\x80'
    command = b'\x04'
    report = True
    def process(self, pkt):
        distance = struct.unpack(">h", pkt[6:8])[0]
        return {"distance": distance}

class REPORT_HUMAN_POSITION(Command):
    """Report human position with respect to the sensor [cm]"""
    control = b'\x80'
    command = b'\x05'
    report = True
    def process(self, pkt):
        x, y, z = struct.unpack(">hhh", pkt[6:12])
        return {
            "x": (x&0x7fff) * (-1 if x&0x8000 else 1),
            "y": (y&0x7fff) * (-1 if y&0x8000 else 1),
            "z": (z&0x7fff) * (-1 if z&0x8000 else 1),
        }

class CMD_QUERY_HUMAN_PRESENCE_SWITCH(Command):
    """Query human presence function"""
    control = b'\x80'
    command = b'\x80'
    def process(self, pkt):
        return {"active": True if pkt[6] else False}

class CMD_QUERY_HUMAN_PRESENCE(Command):
    """Query human presence"""
    control = b'\x80'
    command = b'\x81'
    def process(self, pkt):
        return {"presence": True if pkt[6] else False}

class CMD_QUERY_HUMAN_SPORTS_INFORMATION(Command):
    """Query human activity level"""
    control = b'\x80'
    command = b'\x82'
    def process(self, pkt):
        level = ["None", "Still", "Active"]
        return {"activity": level[pkt[6]]}

class CMD_QUERY_HUMAN_BODY_MOVEMENT(Command):
    """Query body movement level"""
    control = b'\x80'
    command = b'\x83'
    def process(self, pkt):
        return {"movement": pkt[6]}

class CMD_QUERY_HUMAN_DISTANCE(Command):
    """Query human distance to the sensor [cm]"""
    control = b'\x80'
    command = b'\x84'
    def process(self, pkt):
        distance = struct.unpack(">h", pkt[6:8])[0]
        return {"distance": distance}

class CMD_QUERY_HUMAN_POSITION(Command):
    """Query human position with respect to the sensor [cm]"""
    control = b'\x80'
    command = b'\x85'
    def process(self, pkt):
        x, y, z = struct.unpack(">hhh", pkt[6:12])
        x, y, z = struct.unpack(">hhh", pkt[6:12])
        return {
            "x": (x&0x7fff) * (-1 if x&0x8000 else 1),
            "y": (y&0x7fff) * (-1 if y&0x8000 else 1),
            "z": (z&0x7fff) * (-1 if z&0x8000 else 1),
        }

# Breathing functions

class CMD_ACTIVATE_BREATHING_REPORT(Command):
    """Turn the breathing rate monitoring function on or off"""
    control = b'\x81'
    command = b'\x00'
    def execute(self, active: bool):
        self.data = b'\x01' if active else b'\x00'
        return super().execute()
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class REPORT_BREATHING_INFORMATION(Command):
    """Report human respiratory information"""
    control = b'\x81'
    command = b'\x01'
    report = True
    def process(self, pkt):
        _type = ["Normal", "Fast breathing", "Slow breathing", "None"]
        #print(pkt[6]-1); print(_type[pkt[6]-1])
        return {"breathing": _type[pkt[6]-1]}

class REPORT_RESPIRATORY_RATE(Command):
    """Report human respiratory rate"""
    control = b'\x81'
    command = b'\x02'
    report = True
    def process(self, pkt):
        return {"respiratoryrate": pkt[6]}

class REPORT_RESPIRATORY_WAVEFORM(Command):
    """Report human respiratory waveform"""
    control = b'\x81'
    command = b'\x05'
    report = True
    def process(self, pkt):
        return {"waveform": tuple(pkt[6:11])}

class CMD_QUERY_BREATHING_REPORT(Command):
    """Query the breathing rate monitoring function"""
    control = b'\x81'
    command = b'\x80'
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class CMD_QUERY_BREATHING_INFORMATION(Command):
    """Report human respiratory information"""
    control = b'\x81'
    command = b'\x81'
    def process(self, pkt):
        _type = ["Normal", "Fast breathing", "Slow breathing", "None"]
        #print(pkt[6]-1); print(_type[pkt[6]-1])
        return {"breathing": _type[pkt[6]-1]}

class CMD_QUERY_RESPIRATORY_RATE(Command):
    """Report human respiratory rate"""
    control = b'\x81'
    command = b'\x82'
    def process(self, pkt):
        return {"respiratoryrate": pkt[6]}

class CMD_QUERY_RESPIRATORY_WAVEFORM(Command):
    """Report human respiratory waveform"""
    control = b'\x81'
    command = b'\x85'
    def process(self, pkt):
        return {"waveform": tuple(pkt[6:11])}

# Sleeping functions

class CMD_SLEEPING_MONITOR_SWITCH(Command):
    """Turn the sleeping monitoring function on or off"""
    control = b'\x84'
    command = b'\x00'
    def execute(self, active: bool):
        self.data = b'\x01' if active else b'\x00'
        return super().execute()
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class REPORT_BED_OCCUPATION(Command):
    """Report in / out of bed status"""
    control = b'\x84'
    command = b'\x01'
    report = True
    def process(self, pkt):
        occupation = ["got out of bed", "got into bed", "None"]
        return {"occupation": occupation[pkt[6]]}

class REPORT_SLEEP_STATE(Command):
    """Report sleep state"""
    control = b'\x84'
    command = b'\x02'
    report = True
    def process(self, pkt):
        state = ["Deep sleep", "Light sleep", "Awake", "None"]
        return {"state": state[pkt[6]]}

class REPORT_AWAKE_TIME(Command):
    """Report awake time"""
    control = b'\x84'
    command = b'\x03'
    report = True
    def process(self, pkt):
        sleep_length = struct.unpack(">h", pkt[6:8])[0]
        return {"length": sleep_length}

class REPORT_LIGHT_SLEEP_TIME(Command):
    """Report light sleep time"""
    control = b'\x84'
    command = b'\x04'
    report = True
    def process(self, pkt):
        sleep_length = struct.unpack(">h", pkt[6:8])[0]
        return {"length": sleep_length}

class REPORT_DEEP_SLEEP_TIME(Command):
    """Report deep sleep time"""
    control = b'\x84'
    command = b'\x05'
    report = True
    def process(self, pkt):
        sleep_length = struct.unpack(">h", pkt[6:8])[0]
        return {"length": sleep_length}

class REPORT_SLEEP_QUALITY_SCORE(Command):
    """Report sleep quality score"""
    control = b'\x84'
    command = b'\x06'
    report = True
    def process(self, pkt):
        return {"score": pkt[6]}

class REPORT_SLEEP_STATUS(Command):
    """Report sleep status"""
    control = b'\x84'
    command = b'\x0C'
    report = True
    def process(self, pkt):
        return {
            "occupation": True if pkt[6] else False,
            "status": ["Deep sleep", "Light sleep", "Awake", "Empty"][pkt[7]],
            "rr_avg": pkt[8],
            "hr_avg": pkt[9],
            "turnover": pkt[10],
            "large_mov": pkt[11],
            "small_mov": pkt[12],
            "apnea": pkt[13],
        }

class REPORT_SLEEP_QUALITY_ANALISYS(Command):
    """Report sleep quality analisys"""
    control = b'\x84'
    command = b'\x0D'
    report = True
    def process(self, pkt):
        data = struct.unpack(">BH9B", pkt[6:18])
        return {
            "quality": data[0],
            "sleep_time": data[1],
            "awake_time": data[2],
            "light_sleep_time": data[3],
            "deep_sleep_time": data[4],
            "out_of_bed_time": data[5],
            "out_of_bed_times": data[6],
            "turnovers": data[7],
            "rr_avg": data[8],
            "hr_avg": data[9],
            "reserved": data[10],
        }

class REPORT_ABNORMAL_SLEEP(Command):
    """Report abnormal sleep"""
    control = b'\x84'
    command = b'\x0E'
    report = True
    def process(self, pkt):
        return {"abnormality": ["less than 4 hours", "more than 12 hours", "Normal", "None"][pkt[6]]}

class CMD_ACTIVATE_SLEEP_MODE(Command):
    """Activate sleep tracking mode"""
    control = b'\x84'
    command = b'\x0F'
    def execute(self, active: bool):
        self.data = b'\x01' if active else b'\x00'
        return super().execute()
    def process(self, pkt):
        return {"sleep_mode": True if pkt[6] else False}

class REPORT_SLEEP_QUALITY_RATING(Command):
    """Report sleep quality rating"""
    control = b'\x84'
    command = b'\x10'
    report = True
    def process(self, pkt):
        return {"quality": ["None", "High", "Medium", "Poor"][pkt[6]]}

class REPORT_ABNORMAL_STRUGGLING(Command):
    """Report sleep quality rating"""
    control = b'\x84'
    command = b'\x11'
    report = True
    def process(self, pkt):
        return {"struggling": ["None", "Normal", "Struggling"][pkt[6]]}

class REPORT_UNOCCUPIED_STATUS(Command):
    """Report sleep quality rating"""
    control = b'\x84'
    command = b'\x12'
    report = True
    def process(self, pkt):
        return {"unoccupied_timing": ["None", "Normal", "Abnormal"][pkt[6]]}

class CMD_ABNORMAL_STRUGGLING_STATE_SWITCH(Command):
    """Turn the struggling state function on or off"""
    control = b'\x84'
    command = b'\x13'
    def execute(self, active: bool):
        self.data = b'\x00' if active else b'\x01'
        return super().execute()
    def process(self, pkt):
        return {"status": False if pkt[6] else True}

class CMD_UNONCCUPIED_TIMING_REPORT_SWITCH(Command):
    """Turn the unoccupied timing report function on or off"""
    control = b'\x84'
    command = b'\x14'
    def execute(self, active: bool):
        self.data = b'\x01' if active else b'\x00'
        return super().execute()
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class CMD_UNOCCUPIED_SLEEP_TIMER_SETTING(Command):
    """Set the amount of time to consider the bed unoccupied"""
    control = b'\x84'
    command = b'\x15'
    def execute(self, time: int):
        self.data = time.to_bytes()
        return super().execute()
    def process(self, pkt):
        return {"time": pkt[6]}

class CMD_SET_TIMER_SLEEPING_STOP_STATE(Command):
    """Change timer for stop-sleeping"""
    control = b'\x84'
    command = b'\x16'
    def execute(self, time: int):
        self.data = time.to_bytes()
        return super().execute()
    def process(self, pkt):
        return {"time": pkt[6]}

class CMD_QUERY_SLEEPING_MONITOR_SWITCH(Command):
    """Turn the sleeping monitoring function on or off"""
    control = b'\x84'
    command = b'\x80'
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class CMD_QUERY_BED_OCCUPATION(Command):
    """Query in / out of bed status"""
    control = b'\x84'
    command = b'\x81'
    def process(self, pkt):
        occupation = ["out of bed", "in bed"]
        return {"occupation": occupation[pkt[6]]}

class CMD_QUERY_SLEEP_STATE(Command):
    """Query sleep state"""
    control = b'\x84'
    command = b'\x82'
    def process(self, pkt):
        state = ["Deep sleep", "Light sleep", "Awake", "None"]
        return {"state": state[pkt[6]]}

class CMD_QUERY_AWAKE_TIME(Command):
    """Query awake time"""
    control = b'\x84'
    command = b'\x83'
    def process(self, pkt):
        sleep_length = struct.unpack(">h", pkt[6:8])[0]
        return {"length": sleep_length}

class CMD_QUERY_LIGHT_SLEEP_TIME(Command):
    """Query light sleep time"""
    control = b'\x84'
    command = b'\x84'
    def process(self, pkt):
        sleep_length = struct.unpack(">h", pkt[6:8])[0]
        return {"length": sleep_length}

class CMD_QUERY_DEEP_SLEEP_TIME(Command):
    """Query deep sleep time"""
    control = b'\x84'
    command = b'\x85'
    def process(self, pkt):
        sleep_length = struct.unpack(">h", pkt[6:8])[0]
        return {"length": sleep_length}

class CMD_QUERY_SLEEP_QUALITY_SCORE(Command):
    """Query sleep quality score"""
    control = b'\x84'
    command = b'\x86'
    def process(self, pkt):
        return {"score": pkt[6]}

class CMD_QUERY_SLEEP_MODE_REPORT(Command):
    """Query the report mode either realtime heartrate monitoring or sleep data"""
    control = b'\x84'
    command = b'\x8C'
    def process(self, pkt):
        return {"status": "sleep" if pkt[6] else "realtime"}

class CMD_QUERY_SLEEP_STATUS(Command):
    """Query sleep status"""
    control = b'\x84'
    command = b'\x8D'
    def process(self, pkt):
        return {
            "occupation": True if pkt[6] else False,
            "status": ["Deep sleep", "Light sleep", "Awake", "Empty"][pkt[7]],
            "rr_avg": pkt[8],
            "hr_avg": pkt[9],
            "turnover": pkt[10],
            "large_mov": pkt[11],
            "small_mov": pkt[12],
            "apnea": pkt[13],
        }

class CMD_QUERY_ABNORMAL_SLEEP(Command):
    """Query abnormal sleep"""
    control = b'\x84'
    command = b'\x8E'
    def process(self, pkt):
        return {"abnormality": ["less than 4 hours", "more than 12 hours", "Normal", "None"][pkt[6]]}


class CMD_QUERY_SLEEP_QUALITY_ANALISYS(Command):
    """Query sleep quality analisys"""
    control = b'\x84'
    command = b'\x8F'
    def process(self, pkt):
        data = struct.unpack(">BH9B", pkt[6:18])
        return {
            "quality": data[0],
            "sleep_time": data[1],
            "awake_time": data[2],
            "light_sleep_time": data[3],
            "deep_sleep_time": data[4],
            "out_of_bed_time": data[5],
            "out_of_bed_times": data[6],
            "turnovers": data[7],
            "rr_avg": data[8],
            "hr_avg": data[9],
            "reserved": data[10],
        }

class CMD_QUERY_SLEEP_QUALITY_RATING(Command):
    """Query sleep quality rating"""
    control = b'\x84'
    command = b'\x90'
    def process(self, pkt):
        return {"quality": ["None", "High", "Medium", "Poor"][pkt[6]]}

class CMD_QUERY_ABNORMAL_STRUGGLING(Command):
    """Query abnormal struggling status"""
    control = b'\x84'
    command = b'\x91'
    def process(self, pkt):
        return {"struggling": ["None", "Normal", "Struggling"][pkt[6]]}

class CMD_QUERY_UNOCCUPIED_STATUS(Command):
    """Query timing unoccupied status"""
    control = b'\x84'
    command = b'\x92'
    def process(self, pkt):
        return {"unoccupied_timing": ["None", "Normal", "Abnormal"][pkt[6]]}

class CMD_QUERY_ABNORMAL_STRUGGLING_STATE_SWITCH(Command):
    """Turn the struggling state function on or off"""
    control = b'\x84'
    command = b'\x93'
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class CMD_QUERY_UNONCCUPIED_TIMING_REPORT_SWITCH(Command):
    """Turn the unoccupied timing report function on or off"""
    control = b'\x84'
    command = b'\x94'
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class CMD_QUERY_UNOCCUPIED_SLEEP_TIMER_SETTING(Command):
    """Query the amount of time to consider the bed unoccupied"""
    control = b'\x84'
    command = b'\x95'
    def process(self, pkt):
        return {"time": pkt[6]}

class CMD_QUERY_TIMER_SLEEPING_STOP_STATE(Command):
    """Query the timer for stop-sleeping"""
    control = b'\x84'
    command = b'\x96'
    def process(self, pkt):
        return {"time": pkt[6]}

# Heartrate functions

class CMD_HEARTRATE_MONITOR_SWITCH(Command):
    """Turn the heartrate monitoring function on or off"""
    control = b'\x85'
    command = b'\x00'
    def execute(self, active: bool):
        self.data = b'\x01' if active else b'\x00'
        return super().execute()
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class REPORT_HEARTRATE(Command):
    """Report human heartrate"""
    control = b'\x85'
    command = b'\x02'
    report = True
    def process(self, pkt):
        return {"heartrate": pkt[6]}

class REPORT_HEARTRATE_WAVEFORM(Command):
    """Report human heartrate waveform"""
    control = b'\x85'
    command = b'\x05'
    report = True
    def process(self, pkt):
        return {"waveform": tuple(pkt[6:11])}

class CMD_QUERY_HEARTRATE_MONITOR_SWITCH(Command):
    """Turn the heartrate monitoring function on or off"""
    control = b'\x85'
    command = b'\x80'            
    def process(self, pkt):
        return {"status": True if pkt[6] else False}

class CMD_QUERY_HEARTRATE(Command):
    """Report human heartrate"""
    control = b'\x85'
    command = b'\x82'
    def process(self, pkt):
        return {"heartrate": pkt[6]}

class CMD_QUERY_HEARTRATE_WAVEFORM(Command):
    """Report human heartrate waveform"""
    control = b'\x85'
    command = b'\x85'
    def process(self, pkt):
        return {"waveform": tuple(pkt[6:11])}

commands = (
    REPORT_PING_PACKET,
    CMD_RESET,
    CMD_PING,
    CMD_QUERY_PRODUCT_MODEL,
    CMD_QUERY_PRODUCT_ID,
    CMD_QUERY_HARDWARE_MODEL,
    CMD_QUERY_HARDWARE_VERSION,
    REPORT_INITIALIZATION_COMPLETE,
    CMD_QUERY_INITIALIZATION_COMPLETE,
    REPORT_OUT_OF_BOUNDS_STATUS,
    CMD_QUERY_OUT_OF_BOUNDS_STATUS,
    CMD_ACTIVATE_HUMAN_PRESENCE,
    CMD_ACTIVATE_BREATHING_REPORT,
    REPORT_HUMAN_PRESENCE,
    REPORT_HUMAN_SPORTS_INFORMATION,
    REPORT_HUMAN_BODY_MOVEMENT,
    REPORT_HUMAN_DISTANCE,
    REPORT_HUMAN_POSITION,
    CMD_QUERY_HUMAN_PRESENCE_SWITCH,
    CMD_QUERY_HUMAN_PRESENCE,
    CMD_QUERY_HUMAN_SPORTS_INFORMATION,
    CMD_QUERY_HUMAN_BODY_MOVEMENT,
    CMD_QUERY_HUMAN_DISTANCE,
    CMD_QUERY_HUMAN_POSITION,
    REPORT_BREATHING_INFORMATION,
    REPORT_RESPIRATORY_RATE,
    REPORT_RESPIRATORY_WAVEFORM,
    CMD_QUERY_BREATHING_REPORT,
    CMD_QUERY_BREATHING_INFORMATION,
    CMD_QUERY_RESPIRATORY_RATE,
    CMD_QUERY_RESPIRATORY_WAVEFORM,
    CMD_SLEEPING_MONITOR_SWITCH,
    REPORT_BED_OCCUPATION,
    REPORT_SLEEP_STATE,
    REPORT_AWAKE_TIME,
    REPORT_LIGHT_SLEEP_TIME,
    REPORT_DEEP_SLEEP_TIME,
    REPORT_SLEEP_QUALITY_SCORE,
    REPORT_SLEEP_STATUS,
    REPORT_SLEEP_QUALITY_ANALISYS,
    REPORT_ABNORMAL_SLEEP,
    CMD_ACTIVATE_SLEEP_MODE,
    REPORT_SLEEP_QUALITY_RATING,
    REPORT_ABNORMAL_STRUGGLING,
    REPORT_UNOCCUPIED_STATUS,
    CMD_ABNORMAL_STRUGGLING_STATE_SWITCH,
    CMD_UNONCCUPIED_TIMING_REPORT_SWITCH,
    CMD_UNOCCUPIED_SLEEP_TIMER_SETTING,
    CMD_SET_TIMER_SLEEPING_STOP_STATE,
    CMD_QUERY_SLEEPING_MONITOR_SWITCH,
    CMD_QUERY_BED_OCCUPATION,
    CMD_QUERY_SLEEP_STATE,
    CMD_QUERY_AWAKE_TIME,
    CMD_QUERY_LIGHT_SLEEP_TIME,
    CMD_QUERY_DEEP_SLEEP_TIME,
    CMD_QUERY_SLEEP_QUALITY_SCORE,
    CMD_QUERY_SLEEP_MODE_REPORT,
    CMD_QUERY_SLEEP_STATUS,
    CMD_QUERY_ABNORMAL_SLEEP,
    CMD_QUERY_SLEEP_QUALITY_ANALISYS,
    CMD_QUERY_SLEEP_QUALITY_RATING,
    CMD_QUERY_ABNORMAL_STRUGGLING,
    CMD_QUERY_UNOCCUPIED_STATUS,
    CMD_QUERY_ABNORMAL_STRUGGLING_STATE_SWITCH,
    CMD_QUERY_UNONCCUPIED_TIMING_REPORT_SWITCH,
    CMD_QUERY_UNOCCUPIED_SLEEP_TIMER_SETTING,
    CMD_QUERY_TIMER_SLEEPING_STOP_STATE,
    CMD_HEARTRATE_MONITOR_SWITCH,
    REPORT_HEARTRATE,
    REPORT_HEARTRATE_WAVEFORM,
    CMD_QUERY_HEARTRATE_MONITOR_SWITCH,
    CMD_QUERY_HEARTRATE,
    CMD_QUERY_HEARTRATE_WAVEFORM,
)