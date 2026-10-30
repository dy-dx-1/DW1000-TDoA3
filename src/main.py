from tdoa3_tag import TDOA3_Tag
from config import TAG_ID, SPI_BUS, SPI_CS

def main():
    with TDOA3_Tag(id=TAG_ID, bus=SPI_BUS, cs=SPI_CS) as tag: 
        tag.run(enable_print=True) 

if __name__ == "__main__": 
    main() 