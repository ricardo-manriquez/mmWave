# Constants
HEADER = b'\x53\x59'  # SY
FOOTER = b'\x54\x43'  # TC

# Control codes
CONTROL_BASIC = 0x01      # Basic functions
CONTROL_PRODUCT = 0x02    # Product information
CONTROL_INIT = 0x05       # Initialization
CONTROL_BOUNDS = 0x07     # Out of bounds
CONTROL_PRESENCE = 0x80   # Human presence
CONTROL_BREATHING = 0x81  # Breathing
CONTROL_SLEEP = 0x84      # Sleep monitoring
CONTROL_HEARTRATE = 0x85  # Heart rate

# Message types and status
class MessageTypes:
    # Basic status
    NORMAL = "Normal"
    NONE = "None"
    
    # Activity levels
    ACTIVITY_LEVELS = ["None", "Still", "Active"]
    
    # Breathing types
    BREATHING_TYPES = ["Normal", "Fast breathing", "Slow breathing", "None"]
    
    # Sleep states
    SLEEP_STATES = ["Deep sleep", "Light sleep", "Awake", "None"]
    
    # Bed status
    BED_STATUS = ["got out of bed", "got into bed", "None"]
    
    # Sleep quality
    SLEEP_QUALITY = ["None", "High", "Medium", "Poor"]
    
    # Struggling status
    STRUGGLE_STATUS = ["None", "Normal", "Struggling"]
    
    # Sleep abnormality
    SLEEP_ABNORMAL = ["less than 4 hours", "more than 12 hours", "Normal", "None"]
    
# Filter categories
class FilterCategories:
    VITAL_SIGNS = "vital_signs"  # About breathing and heart rate
    PRESENCE = "presence"        # About presence and movement
    SLEEP = "sleep"              # About sleep monitoring
    WAVE_FORM = "waveform"       # About waveforms
    ALL = "all"                  # all categories

# Topic groupings
TOPIC_GROUPS = {
    FilterCategories.VITAL_SIGNS: [
        'heartrate',
        'breathing',
        'respiratory_rate',
    ],
    FilterCategories.PRESENCE: [
        'presence',
        'distance',
        'position',
        'movement',
        'activity'
    ],
    FilterCategories.SLEEP: [
        'bed_status',
        'sleep_state',
        'sleep_status',
        'sleep_analysis',
        'sleep_quality',
        'struggling',
        'sleep_abnormality'
    ],
    FilterCategories.WAVE_FORM: [
        'heartrate_waveform',
        'respiratory_waveform'
    ]
}