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

### Code Implementation
#### Configuration 
`config.py` controls all user-relevant configuration parameters. Anchors and their positions may be defined here in the `ANCHORS` dictionnary. During operation, if the tag hears from an anchor that was not defined in `config.py`, it will add it to it's anchor dict **if and only if** the anchor also broadcasts it's position. Anchor position broadcasts are overwritten by `config.py` if a value is already present there. In other words, one may configure anchor positions in their firmware or at the `config.py` level, with the latter overwritting the former if there's conflict. 
#### Main localization loop
The `TDOA3_Tag` class is the main coordinator and is used to derive the different elements of the TDoA equation. It offers continuous localization through the `TDOA3_Tag.run()` method. 

`TDOA3_Tag.run()` is an infinite loop that: 
1. Captures as many UWB packets as possible from the anchors for a duration corresponding to `LOCALIZATION_FREQUENCY` in `config.py`
    - Relevant function: `DW1000.listen()`
2. Aggregates the raw data per anchor in a dictionnary: `{anchor_id: [msg, ..., msg]}` where `msg = {seq, rx, tx, remote_data}`
    - Relevant function: `TDOA3_Tag.aggregate_raw_pkts()`
3. Processes the raw data into TDOA info for independent anchor pairs in a list: `[(TDOA, anchor_1_pos, anchor_2_pos), ...]`
    - Relevant function: `TDOA3_Tag.process_anchor_data()`
4. ***TODO*** - to complete with EKF / multilateration process 
5. Prints and/or saves the resulting position 
##### Data aggregation step: `TDOA3_Tag.aggregate_raw_pkts()`
This function takes a list of the raw packets we heard `[(raw_packet, local_rx_time), ...]` and creates a dict of relevant data per anchor.

We iterate through all raw packets. Each one is passed through `TDOA3_Tag.interpret_anchor_msg()` to validate it's relevance and extract the following: 
- Anchor ID 
- Transaction SEQ number
- Transmission time in the anchor's clock 
- Remote anchor data reported by the anchor
    - This is the latest info the anchor had on other anchors in a dict of shape `{remoteAnchorIds: (seq, rx_ts, r_tof), ...}`.  
        - `seq`: SEQ of the last message the remote anchor sent 
        - `rx_ts`: Reception time of said message in the anchor's clock (not in the remote's) 
        - `r_tof`: Measured ToF to this anchor in radio ticks 
- Anchor position 

Then, anchors that were not defined in `config.py` are added to the tag's internal `anchors` dict with their shared firmware position. If no position is shared, the anchor is ignored and a warning is printed on the terminal.

Just like in `__init__`, any time a new anchor is added, `self.update_ref_TOFs()` is called to make sure distance info between anchors is kept up to date. 

The extracted information is added to the internal `self.history` dict of shape `{anchor_id: [msg, ..., msg]}` where `msg = {seq, rx, tx, remote_data}`. The keys of this dict are `deque` with a max length capped by the `HISTORY` constant. This allows us to retain only relevant messages. In theory, we only need the 2 most recent messages to compute the necessary elements. However, keeping a longer history increases our chances of finding info from matching `SEQ` transactions, which is necessary in `process_anchor_data()`. 

We also add anchor ID's that we heard from to the internal set `self.fresh_anchors`, which is refreshed on each call of `aggregate_raw_pkts()`. This allows us to keep old anchor data, while knowing what anchors recentrly transmitted





