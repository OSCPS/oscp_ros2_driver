# OSCP Rotation Matrix Frame
# Corresponds to oscp_rot_mat_t structure

std_msgs/Header header

uint8 header_byte           # Misalignment Correction (2b) | Operating Mode (3b) | Frame Type (3b)
uint8 counter               # Wrapping frame counter [0,255]
uint64 timestamp_ms         # Timestamp in ms since last power up or reset

# 3x3 Rotation Matrix (row-major order)
float32[9] rotation_matrix  # [m00, m01, m02, m10, m11, m12, m20, m21, m22]

uint8 status                # Status byte - use OSCP_STATUS_* masks
uint16 crc                  # Checksum (CRC-16)
