import sys 
from pathlib import Path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import time 
from src.dw1000 import DW1000

with DW1000(bus=0, cs=0, channel=2, PRF=64, bitrate=6, preamble_length=128, preamble_code=9, smart_tx_power=True, tx_power_settings=None) as dw: 
    t1 = time.perf_counter() 
    while time.perf_counter()-t1 < 10: 
        print(dw.listen(return_ints=False)) 
        time.sleep(0.1)  