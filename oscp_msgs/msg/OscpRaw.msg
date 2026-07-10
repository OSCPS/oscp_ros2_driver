# OSCP Raw IMU Frame
# Corresponds to oscp_raw_t structure

std_msgs/Header header

uint8 header_byte       # Misalignment Correction (2b) | Operating Mode (3b) | Frame Type (3b)
uint8 counter           # Wrapping frame counter [0,255]
uint64 timestamp_ms     # Timestamp in ms since last power up or reset

float32 gyro_x          # °/sec
float32 gyro_y          # °/sec
float32 gyro_z          # °/sec

float32 accel_x         # g
float32 accel_y         # g
float32 accel_z         # g

float32 incl_x          # mg
float32 incl_y          # mg

float32 mag_x           # μT
float32 mag_y           # μT
float32 mag_z           # μT

float32 temp            # °C

uint8 status            # Status byte - use OSCP_STATUS_* masks
uint16 crc              # Checksum (CRC-16)
