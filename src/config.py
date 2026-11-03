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
    # The code will prioritise position information defined here, if it isn't present, it will use the firmware 
    # definition that is shared by the anchors. 
# Positions in ANCHORS are in UNITS (see UNIT CONFIGURATION). Firmware positions are always in meters.
ANCHORS = {1: (4.736, 2.194, 1.213), 
           2: (1.373, 5.520, 2.190), 
           3: (1.372, 0.150, 0.128), 
           4: (4.736, 2.194, 0.302), 
           5: (0.793, 2.971, 2.190), 
           6: (4.582, 6.040, 1.460), 
           7: (0.223, 0.458, 2.190), 
           8: (5.045, 4.194, 1.070)} # Expected format is {anchor_id: (x, y, z)}

################################## UNIT CONFIGURATION ##################################
# All internal computations are done in meters. UNITS only applies to the ANCHORS dict above
# and to the output coordinates (console and CSV).
UNITS    = "m"  # Any of: "m", "cm", "mm"
DECIMALS = 3    # Output coordinates are rounded to this number of decimals (in UNITS), 0 for whole numbers

################################## OUTPUT CONTROL ##################################
LOCALIZATION_FREQUENCY = 1  # Frequency of updates in Hz
OUTPUTS = ["console", "dynamic"]  # Output controllers to enable, any of: "console", "csv", "dynamic"
CSV_SAVE_INTERVAL_SECONDS = 3600  # Flush buffered localization rows at most hourly