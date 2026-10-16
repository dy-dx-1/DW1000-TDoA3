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
Bitcraze's TDoA3 algorithm **does not require centralized anchor synchronization, or periodic broadcasts by the tag**. Instead, anchors communicate at random times _between each other_ and tags are _pure listeners_, they never transmit. This allows for, in theory, an **unlimited** number of tags. To understand how this is possible, we return to the basic mathematical relationship of the hyperbola where the _difference in distances_ from the tag to an anchor pair is constant. 

We call this difference $|d_{1}-d_{2}|$, because the speed of light is constant, this is proportional to $|TOF_{A_{1}-t}-TOF_{A_{2}-t}|$, where $A_1$ and $A_2$ are the anchors, $t$ is the tag and $TOF_{A_{1}-t}$ is the _Time of Flight_ (the time a signal takes to travel) from $A_1$ to $t$.

The TOF with respect to anchor $i$ can be defined as: $TOF_{A_{i}-t}=t_{ir}-\tau_{i}$ 
- $t_{ir}$ is the time that the tag recorded the message
- $\tau_{i}$ is the time when the anchor sent the message, _in the tag's clock_
    - This value cannot be directly extracted from an anchor's reporting, because clocks are not synchronized!
We will solve this through algebraic manipulation:

$TOF_{A_{1}-t}-TOF_{A_{2}-t}=t_{1r}-\tau_{1}-(t_{2r}-\tau_{2})$


$TOF_{A_{1}-t}-TOF_{A_{2}-t}=t_{1r}-t_{2r}-\tau_{1}+\tau_{2}$ 
- $TOF_{A_{1}-t}-TOF_{A_{2}-t}$ is the difference in ToF's, which is exactly the difference the tag will see in the _time of arrival_ of the messages. This _is_ the TDoA and, once again, is proportional to the difference in distances, so we can directly use it to constrain the position.
- $t_{1r}-t_{2r}$ is simply the difference in reception times $\delta RX$, already in the tag's clock so easy to measure

Therefore, we can write: $TDoA=\delta RX-(\tau_{1}-\tau_{2})$
- $(\tau_{1}-\tau_{2})$ is the difference in transmission times $\delta TX$, this is **harder**, because they are reported on different anchor clocks

To calculate $\delta TX = \tau_{1}-\tau_{2}$, the Bitcraze TDOA3 anchor firmware has the anchors do Two Way Ranging between themselves
- Since $A_1$ and $A_2$ are doing TWR, the ToF between them is known
- Any anchor can then estimate, _in their own clock_, when another anchor transmitted, by subtracting the ToF from the time it received the message from the other anchor.
    - Therefore, any anchor can estimate $\delta TX$ in their own clock. However this result will be in the anchor's clock ticks, whereas we want it in the tag's clock ticks.   




