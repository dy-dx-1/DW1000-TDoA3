from tdoa3_tag_synthetic import TDOA3_Tag

def main():
    with TDOA3_Tag(id=1) as tag: 
        tag.run_synthetic(enable_print=True) 

if __name__ == "__main__": 
    main() 