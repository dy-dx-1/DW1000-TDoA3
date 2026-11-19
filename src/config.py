"""
Config file for code deployment. Controls: 
- Tag SPI connection 
- Anchor definition 
- Localization frequency 
- Units
- Output control
"""
################################## TAG CONFIGURATION ####################################
TAG_ID  = 10 
SPI_BUS = 0   # SPI bus for the DW1000 connection
SPI_CS  = 0   # SPI chip select for the DW1000 connection

################################## ANCHOR CONFIGURATION ##################################
# Edit anchor_dict to define or override specific anchor positions instead of the ones set in their firmware
# All anchors you use MUST have a position defined either in their firmware OR on here. 
    # The code will prioritise position information defined here, if it isn't present, it will use the firmware definition of the anchor
# Positions in ANCHORS are always in meters, matching firmware positions.
ANCHORS = {}
"""
MIST = {1: (4.736, 2.194, 1.213), 
           2: (1.373, 5.520, 2.190), 
           3: (1.372, 0.150, 0.128), 
           4: (4.736, 2.194, 0.302), 
           5: (0.793, 2.971, 2.190), 
           6: (4.582, 6.040, 1.460), 
           7: (0.223, 0.458, 2.190), 
           8: (5.045, 4.194, 1.070)} # Expected format is {anchor_id: (x, y, z)}

ETS = {
    0: (-1.482, -2.771, 2.737),
    1: (-1.454, 2.829, 1.908),
    2: (2.689, 9.408, 1.268),
    3: (7.852, 4.481, 2.054),
    4: (7.887, 0.505, 2.29),
    5: (7.777, -3.364, 2.14),
    6: (2.41, -4.38, 2.3),
    7: (3.765, 6.231, 2.195),
    8: (-1.473, 6.343, 2.75),
    9: (-1.177, 9.418, 2.26)}
"""
################################## UNIT CONFIGURATION ##################################
# All internal computations and anchor positions are in meters. 
# UNITS applies to console, CSV, and dynamic output ONLY. ROS remains in meters. 
UNITS    = "m"  # Any of: "m", "cm", "mm"
DECIMALS = 3    # Output coordinates are rounded to this number of decimals (in UNITS), 0 for whole numbers

################################## SOLVER CONFIGURATION ##################################
SOLVER = {
    "mode_2d": True,            # Solve x,y only, with z fixed at antenna_height_m. More stable when the tag's height is known
    "antenna_height_m": 0.25,     # Height of the tag's antenna, in meters (regardless of UNITS). Only used when mode_2d is True
    "warm_start": False,         # Start each solve from the last pose instead of the anchor centroid
    "max_residual_rms_m": None,  # Reject poses whose TDOA pairs disagree by more than this RMS, in meters. None = disabled
                                 # When enabled, at least 4 TDOA pairs are needed per pose (see README)
}

################################## OUTPUT CONTROL ##################################
LOCALIZATION_FREQUENCY = 10  # Frequency of updates in Hz
OUTPUTS = ["console"]  # Output controllers to enable, any of: "console", "csv", "dynamic", "ros2"
CSV_SAVE_INTERVAL_SECONDS = 3600  # Flush buffered localization rows at most hourly

################################## ROS 2 OUTPUT ##################################
# Only used when "ros2" is in OUTPUTS. Needs a ROS 2 install (see README)
ROS2 = {
    "node_name": "uwb_tdoa3_tag",
    "topic": "/mobile_manip/sensors/uwb/pose",  # Absolute topic name of the published geometry_msgs/PoseWithCovarianceStamped
    "frame_id": "map",                          # Frame of the anchor coordinates
}