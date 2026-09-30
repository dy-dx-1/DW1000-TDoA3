"""
Config file for code deployment. Controls: 
- Tag SPI connection 
- Anchor definition 

TODO add: 
- Output control (print/csv/ROS?)
- Units (mm/m/...) #
"""
################################## TAG CONFIGURATION ####################################
TAG_ID  = 10 
SPI_BUS = 0   # SPI bus for the DW1000 connection
SPI_CS  = 0   # SPI chip select for the DW1000 connection

################################## ANCHOR CONFIGURATION ##################################
# Edit anchor_dict to define or override specific anchor positions instead of the ones set in their firmware
# All anchors you use MUST have a position defined either in their firmware OR on here. 
    # The code will prioritise position information defined here, if it isn't present, it will use the firmware 
    # definition that is shared by the anchors. 
# Ensure units are consistent between firmware & config.py 
# NOTE currently expecting meters
ANCHORS = {} # Expected format is {anchor_id: (x, y, z)}