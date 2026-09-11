import sys 
from pathlib import Path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import time 
from src.tdoa3_tag import TDOA3_Tag
import struct

with TDOA3_Tag(1) as tag: 
    m = tag._dw.listen()
    if m: 
        print(tag.interpret_anchor_msg(m))