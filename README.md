# DW1000-TDoA3
An implementation of Bitcraze's TDoA3 positioning algorithm for a DW1000-based tag, such as the Bitcraze Loco Positioning Deck and Bitcraze Loco Positioning Nodes serving as anchors. 

This README has the following sections:
1. [Usage](#usage)
2. [Theoretical principles](#theoretical-principles)
    1. [A primer on TDoA](#a-primer-on-tdoa)
    2. [Bitcraze's TDoA3](#bitcrazes-tdoa3)
3. [Code Implementation](#code-implementation)

## Usage 
0. You must have at minimum 4 anchors set up. Though it's [recommended](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa3_protocol/) that at any point in the space 5-10 anchors are in range of the tag. 
1. Ensure all anchors are on the same firmware (code has been tested on version: TODO add link) and in TDOA3 mode
   - You can use `utils/config_loco_node.py` to change their mode/position if needed 
2. Set relevant parameters in `config.py`
3. Run `main.py`
## Theoretical principles 
### A primer on TDoA
TDoA stands for _Time Difference of Arrival_. TDoA positioning systems are geometrically grounded by the fact that hyperbola's (and by extension, hyperboloids) are defined by the set of points $P$ for which the absolute difference in distances to two fixed points, called the _foci_ $F_1$ and $F_2$ is constant: $|PF_{1}-PF_{2}|=2a$. 

Considering pairs of anchors as foci, we can define a hyperbola of possible positions for a tag, if we know the position of the anchors and the **difference in distances to the anchors**. The advantage of this approach is that we don't need to know the tag's position in advance. We only need to measure the difference in distances to the anchors. TDoA systems achieve this by measuring differences in signal arrival times, which are **proportional** to differences in propagation distances.

Traditional TDoA systems require the tag to periodically broadcast messages. Each anchor records the message's arrival time, and a central computer uses these timestamps to calculate TDoA measurements. This approach requires a tightly synchronized network of anchors, typically with nanosecond-level timing accuracy.Bitcraze's TDoA3 algorithm takes a different approach, **avoiding the need for a centralized, tightly synchronized anchor network**.
### Bitcraze's TDoA3 
[Bitcraze's TDoA3 algorithm](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/functional-areas/tdoa_principles/) **does not require centralized anchor synchronization, or periodic broadcasts by the tag**. Instead, anchors communicate at random times _between each other_ and tags act as _pure listeners_; they receive messages but never transmit. This allows for, in theory, an **unlimited** number of tags. 

To understand how this is possible, we return to the basic mathematical relationship of the hyperbola where the _difference in distances_ from the tag to an anchor pair is constant. 

We call this difference $|d_{1}-d_{2}|$, because the speed of light is constant, this is proportional to $|TOF_{A_{1}-t}-TOF_{A_{2}-t}|$, where $A_1$ and $A_2$ are the anchors, $t$ is the tag and $TOF_{A_{1}-t}$ is the _Time of Flight_ (the time a signal takes to travel) from $A_1$ to $t$.

The TOF with respect to anchor $i$ can be defined as: $TOF_{A_{i}-t}=t_{ir}-\tau_{i}$ 
- $t_{ir}$ is the time that the tag recorded the message
- $\tau_{i}$ is the time when the anchor sent the message, _in the tag's clock_
    - This value cannot be directly extracted from an anchor's reporting, because clocks are not synchronized!
We will solve this through algebraic manipulation:

$$
\boxed{TOF_{A_{1}-t}-TOF_{A_{2}-t}=t_{1r}-\tau_{1}-(t_{2r}-\tau_{2})}
$$

$$
\boxed{TOF_{A_{1}-t}-TOF_{A_{2}-t}=t_{1r}-t_{2r}-\tau_{1}+\tau_{2}}
$$ 
- $TOF_{A_{1}-t}-TOF_{A_{2}-t}$ is the difference in ToF's, which once again is proportional to the difference in distances
- $t_{1r}-t_{2r}$ is simply the difference in reception times $\delta RX$, already in the tag's clock so easy to measure

Therefore, we can write: $\delta TOF=\delta RX-(\tau_{1}-\tau_{2})$
- $(\tau_{1}-\tau_{2})$ is the difference in transmission times $\delta TX$, this is **harder**, because they are reported on different anchor clocks

To calculate $\delta TX = \tau_{1}-\tau_{2}$, the Bitcraze TDoA3 anchor firmware has the anchors do Two Way Ranging between themselves
- Since $A_1$ and $A_2$ are doing TWR, the ToF between them is known
- Any anchor can then estimate, _in their own clock_, when another anchor transmitted, by subtracting the ToF from the time it received the message from the other anchor.
    - Therefore, any anchor can estimate $\delta TX$ in their own clock. However this result will be in the anchor's clock ticks, whereas we want it in the tag's clock ticks.
    - To solve this, the tag listens to 2 subsequent transmissions from the same anchor and records the delta between them $\delta rx'$ . In the transmissions, the anchor embeds what it thinks is the delta $\delta tx'$.
        - The tag can then compute a coefficient that estimates the amount of tag clock ticks per this anchor clock ticks: $\alpha=\frac{\delta rx'}{\delta tx'}$

Putting everything together, we obtain:

$$
\boxed{\delta TOF=\delta RX-\alpha\delta TX}
$$

where:

- $\delta RX$ is the difference in reception times of the two messages, measured in tag-clock ticks.
- $\delta TX$ is the difference in transmission times between the anchors, estimated in anchor-clock ticks.
- $\alpha$ is the clock correction coefficient that converts anchor-clock ticks to tag-clock ticks.

You may notice that Bitcraze describes this calculation in terms of TDoA. TDoA stands for *Time Difference of Arrival*, but the raw difference in arrival times is not necessarily equal to the difference in propagation times. When two anchors transmit at different times, the difference in arrival times includes both the difference in transmission times and the difference in propagation times.

TDoA3 therefore corrects the measured reception-time difference by subtracting the transmission-time difference, converted to the tag's clock domain. The result, $\delta TOF$, estimates the difference in propagation times between the two signals.

Since propagation time is proportional to distance, this measurement constrains the tag's position to a hyperbola (or a hyperboloid in 3D). The system calculates these measurements for multiple anchor pairs and uses nonlinear least-squares optimization to find the position that best matches them.

### Code




