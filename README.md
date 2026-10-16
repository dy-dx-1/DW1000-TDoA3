# DW1000-TDoA3
An implementation of Bitcraze's TDoA3 positioning algorithm for a DW1000-based tag, such as the Bitcraze Loco Positioning Deck and Bitcraze Loco Positioning Nodes serving as anchors. 

## Usage 
1. Ensure all anchors are on the same firmware (code has been tested on version: TODO add link) and in TDOA3 mode
   - You can use `utils/config_loco_node.py` to change their mode/position if needed 
2. Set relevant parameters in `config.py`
3. Run `main.py`
## Theoretical principles 
### A primer on TDoA
TDoA stands for Time Difference of Arrival. TDoA positioning systems are geometrically grounded by the fact that hyperbola's (and by extension, hyperboloïds) are defined to be curves where all points $P$ satisfy the relationship $|PF_{1}-PF_{2}|=2a$, where $F_{1}, F_{2}$ are fixed points called the *foci*. Therefore, if we consider pairs of anchors as foci, we can define a hyperbola of possible positions for a tag, if we know the position of the anchors and the **difference in distances to the anchors**. This 'difference' requirement is useful, because we can measure it without explicitly knowing the position of the tag itself. We will get to how measuring _time differences of arrival_ is equivalent to using the difference in distances. 

Traditional TDoA systems require the tag to periodically broadcast pings, the time of arrival of these at each anchor is then recorded and used to compute TDoA's by a central computer. The main problem of these systems is that this requires a centralized, tightly synchronized (nanosecond-level) anchor network. Bitcraze's TDoA3 algorithm works in a different manner, which avoids this problem. 
### Bitcraze's TDoA3 
Bitcraze's TDoA3 algorithm **does not require centralized anchor synchronization, or periodic broadcasts by the tag**. Instead, anchors communicate at random times _between each other_ and tags are _pure listeners_, they never transmit. This allows for, in theory, an **unlimited** number of tags. 



