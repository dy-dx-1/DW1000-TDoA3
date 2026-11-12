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
# Positions in ANCHORS are in UNITS (see UNIT CONFIGURATION). Firmware positions should match. 
ANCHORS = {
    0: (-1.4700000286102295, -2.4000000953674316, 2.7100000381469727),
    1: (-1.5399999618530273, 1.659999966621399, 1.7200000286102295),
    2: (2.640000104904175, 9.979999542236328, 1.0499999523162842),
    3: (7.900000095367432, 5.0, 2.0),
    4: (7.96999979019165, 1.090000033378601, 2.240000009536743),
    5: (8.020000457763672, -2.7300000190734863, 2.0799999237060547),
    6: (2.309999942779541, -3.869999885559082, 2.25),
    7: (4.690000057220459, 5.610000133514404, 2.1500000953674316),
    8: (-1.5499999523162842, 6.840000152587891, 2.700000047683716),
    9: (-1.5700000524520874, 9.25, 2.8499999046325684),
}
"""
MIST = {1: (4.736, 2.194, 1.213), 
           2: (1.373, 5.520, 2.190), 
           3: (1.372, 0.150, 0.128), 
           4: (4.736, 2.194, 0.302), 
           5: (0.793, 2.971, 2.190), 
           6: (4.582, 6.040, 1.460), 
           7: (0.223, 0.458, 2.190), 
           8: (5.045, 4.194, 1.070)} # Expected format is {anchor_id: (x, y, z)}
"""
################################## UNIT CONFIGURATION ##################################
# All internal computations are done in meters. UNITS applies to the ANCHORS dict and to the output coordinates 
UNITS    = "m"  # Any of: "m", "cm", "mm"
DECIMALS = 3    # Output coordinates are rounded to this number of decimals (in UNITS), 0 for whole numbers

################################## SOLVER CONFIGURATION ##################################
SOLVER = {
    "mode_2d": False,            # Solve x,y only, with z fixed at antenna_height_m. More stable when the tag's height is known
    "antenna_height_m": 0.0,     # Height of the tag's antenna, in meters (regardless of UNITS). Only used when mode_2d is True
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