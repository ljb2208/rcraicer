std_msgs/Header header

# Satellites used in solution
uint16 satellites_used # Number of satellites
int32[] satellite_used_prn # PRN identifiers

# Satellites visible
uint16 satellites_visible
int32[] satellite_visible_prn # PRN identifiers
int32[] satellite_visible_z # Elevation of satellites
int32[] satellite_visible_azimuth # Azimuth of satellites
int32[] satellite_visible_snr # Signal-to-noise ratios (dB)

# Measurement status
int16 STATUS_NO_FIX=-1   # Unable to fix position
int16 STATUS_FIX=0       # Normal fix
int16 STATUS_SBAS_FIX=1  # Fixed using a satellite-based augmentation system
int16 STATUS_GBAS_FIX=2  #          or a ground-based augmentation system
int16 STATUS_DGPS_FIX=18 # Fixed with DGPS
int16 STATUS_WAAS_FIX=33 # Fixed with WAAS

int16 status

int16 RTK_STATUS_NONE = 0
int16 RTK_STATUS_FLOAT = 1
int16 RTK_STATUS_FIXED = 2

int16 rtk_status

float32 vdop
float32 hdop
float32 pdop