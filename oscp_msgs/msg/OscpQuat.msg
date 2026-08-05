# OSCP Quaternion IMU Frame
# Corresponds to oscp_quat_t structure

std_msgs/Header header

uint8 header_byte       # Misalignment Correction (2b) | Operating Mode (3b) | Frame Type (3b)
uint8 counter           # Wrapping frame counter [0,255]
uint64 timestamp_ms     # Timestamp in ms since last power up or reset

float32 quat_w
float32 quat_x
float32 quat_y
float32 quat_z

uint8 status            # Status byte - use OSCP_STATUS_* masks
uint16 crc              # Checksum (CRC-16)
