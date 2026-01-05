# UM982 RTK GPS Status Message
# Contains all data from UM982 dual-antenna GNSS module

std_msgs/Header header

# Position (WGS84)
float64 latitude              # Latitude in decimal degrees
float64 longitude             # Longitude in decimal degrees
float64 altitude              # Altitude in meters (above ellipsoid)

# RTK Status
uint8 rtk_status              # 0=unknown, 1=standalone, 2=dgps, 4=rtk_fix, 5=rtk_float
string rtk_status_string      # Human readable RTK status
uint8 num_satellites          # Number of satellites used
float32 hdop                  # Horizontal Dilution of Precision
float32 diff_age              # Age of differential corrections (seconds)

# Dual Antenna Heading (from UNIHEADING/HEADINGA)
bool heading_valid            # True if heading data is available
float32 heading               # Heading in degrees (0-360, true north)
float32 pitch                 # Pitch angle in degrees
float32 heading_stddev        # Heading standard deviation (degrees)
float32 pitch_stddev          # Pitch standard deviation (degrees)
float32 baseline              # Baseline length between antennas (meters)

# Velocity (from RMC)
bool velocity_valid           # True if velocity data is available
float32 speed_mps             # Ground speed in m/s
float32 speed_knots           # Ground speed in knots
float32 course                # Course over ground in degrees

# NTRIP/RTCM Statistics
uint32 rtcm_bytes_received    # Total RTCM bytes received

# RTK Status Constants
uint8 STATUS_UNKNOWN=0
uint8 STATUS_STANDALONE=1
uint8 STATUS_DGPS=2
uint8 STATUS_RTK_FIX=4
uint8 STATUS_RTK_FLOAT=5
