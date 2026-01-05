# LiDAR-GPS Fusion Status Message

# State constants
uint8 STATE_WAITING_INIT = 0
uint8 STATE_RUNNING = 1

# Current state
uint8 state

# Initialization info
bool initialized
float64 init_timestamp

# GPS status
bool gps_available
uint8 rtk_status              # 0:unknown, 1:standalone, 2:dgps, 4:fix, 5:float
float32 hdop
bool gps_used                 # Currently using GPS for correction

# Error (GPS - Fused position)
float64 error_x               # East error [m]
float64 error_y               # North error [m]
float64 error_z               # Up error [m]
float64 error_yaw             # Yaw error [deg]
float64 error_distance        # Horizontal distance error [m]

# Accumulated correction offsets
float64 correction_x
float64 correction_y
float64 correction_z
float64 correction_yaw        # [deg]
