# OSCP GNSS Operating Frame
# Corresponds to oscp_gnss_t structure from manual Table 11

std_msgs/Header header

uint8 header_byte       # Misalignment Correction (2b) | Operating Mode (3b) | Frame Type (3b)
uint8 counter           # Wrapping frame counter [0,255]
uint64 timestamp_ms     # Timestamp in ms since last power up or reset

uint8 fix_type          # GNSS Fix Type (see Table 26)
uint8 num_satellites    # Number of satellites used

float32 longitude       # degrees
float32 latitude        # degrees
int32 height_mm         # mm above ellipsoid (signed)

uint32 h_accuracy_mm    # Horizontal accuracy estimate (mm)
uint32 v_accuracy_mm    # Vertical accuracy estimate (mm)

int32 vel_north_mms     # Velocity North in mm/sec [NED Frame] (signed)
int32 vel_east_mms      # Velocity East in mm/sec [NED Frame] (signed)
int32 vel_down_mms      # Velocity Down in mm/sec [NED Frame] (signed)

uint32 speed_accuracy_mms  # Speed Accuracy Estimate (mm/sec)

float32 heading_motion  # Heading of Motion (°) [2D]
float32 heading_accuracy # Heading Accuracy Estimate (°)

float32 position_dop    # Position DOP

uint8 last_correction_age  # Age of last correction (see Table 27)
uint8 gnss_status       # [7:2] RESERVED | [1] Invalid LLH | [0] GNSS Fix OK

uint8 status            # Status byte - use OSCP_STATUS_* masks
uint16 crc              # Checksum (CRC-16)