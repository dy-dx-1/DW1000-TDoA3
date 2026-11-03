from tdoa3_tag import TDOA3_Tag
from config import TAG_ID, SPI_BUS, SPI_CS, ANCHORS
from units import anchors_to_meters

def main():

   # with TDOA3_Tag(id=TAG_ID, bus=SPI_BUS, cs=SPI_CS, anchors=anchors_to_meters(ANCHORS)) as tag:
   #     tag.run()
   from tdoa3_tag_synthetic import TDOA3_Tag
   with TDOA3_Tag(id=TAG_ID, bus=SPI_BUS, cs=SPI_CS, anchors=anchors_to_meters(ANCHORS)) as tag:
        tag.run_synthetic(path = "60_1s_raw_datas.pkl", enable_print=False) 

if __name__ == "__main__":
    main()
