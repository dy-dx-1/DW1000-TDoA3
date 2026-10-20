# DW1000-TDoA3
An implementation of Bitcraze's TDoA3 positioning algorithm for a DW1000-based tag, such as the Bitcraze Loco Positioning Deck, with Bitcraze Loco Positioning Nodes serving as anchors.

This README has the following sections:
1. [Usage](#usage)
2. [Theoretical principles](#theoretical-principles)
    1. [A primer on TDoA](#a-primer-on-tdoa)
    2. [Bitcraze's TDoA3](#bitcrazes-tdoa3)
3. [Code Implementation](#code-implementation)

## Usage 
0. You must have at least 4 anchors set up. It is [recommended](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa3_protocol/) that 5-10 anchors are in range of the tag at any point in the space.
1. Ensure all anchors are on the same firmware (code has been tested on version: TODO add link) and in TDOA3 mode
   - You can use `utils/config_loco_node.py` to change their mode/position if needed 
2. Set relevant parameters in `config.py`
3. Run `main.py`

Localization results can be saved by setting `SAVE_TO_CSV = True` in `config.py`.
The program appends timestamp, known-anchor count, processed-pair count, and
estimated position coordinates to `tdoa3_tag_<id>.csv` in the working directory.
Rows are flushed hourly by default. `CSV_SAVE_INTERVAL_SECONDS` is the only
buffering parameter: it sets both the maximum elapsed time between writes and
the derived row limit (`LOCALIZATION_FREQUENCY * CSV_SAVE_INTERVAL_SECONDS`),
rounded up. The row limit bounds memory if localization iterations run faster
than their configured rate. Pending rows are also flushed when the run loop
exits due to a Python exception or the tag context manager closes.

At the default 1 Hz rate and 3600-second interval, at most 3600 results are
buffered. A result row contains a timestamp, two counts, and three coordinates;
on a typical 64-bit CPython build this is roughly 300-400 bytes of Python object
memory, or about 1-1.5 MiB for 3600 rows (plus small list/allocator overhead).
This is an estimate rather than a platform-independent guarantee. To reduce
buffer RAM or limit data loss on an unclean Python exit, lower
`CSV_SAVE_INTERVAL_SECONDS`; increasing it reduces SD-card writes but increases
both the buffer and the amount of unsaved data.
## Theoretical principles 
### A primer on TDoA
TDoA stands for _Time Difference of Arrival_. TDoA positioning systems are geometrically grounded by the fact that hyperbolas (and, by extension, hyperboloids) are defined by the set of points $P$ for which the absolute difference in distances to two fixed points, called the _foci_ $F_1$ and $F_2$, is constant: $|PF_{1}-PF_{2}|=2a$.

Considering pairs of anchors as foci, we can define a hyperbola of possible positions for a tag, if we know the position of the anchors and the **difference in distances to the anchors**. The advantage of this approach is that we don't need to know the tag's position in advance. We only need to measure the difference in distances to the anchors. TDoA systems achieve this by measuring differences in signal arrival times, which are **proportional** to differences in propagation distances.

Traditional TDoA systems require the tag to periodically broadcast messages. Each anchor records the message's arrival time, and a central computer uses these timestamps to calculate TDoA measurements. This approach requires a tightly synchronized network of anchors, typically with nanosecond-level timing accuracy. Bitcraze's TDoA3 algorithm takes a different approach, **avoiding the need for a centralized, tightly synchronized anchor network**.
### Bitcraze's TDoA3 
[Bitcraze's TDoA3 algorithm](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/functional-areas/tdoa_principles/) **does not require centralized anchor synchronization, or periodic broadcasts by the tag**. Instead, anchors communicate at random times _between each other_ and tags act as _pure listeners_; they receive messages but never transmit. This allows for, in theory, an **unlimited** number of tags. 

To understand how this is possible, we return to the basic mathematical relationship of the hyperbola where the _difference in distances_ from the tag to an anchor pair is constant. 

We call this difference $|d_{1}-d_{2}|$. Because the speed of light is constant, it is proportional to $|TOF_{A_{1}-t}-TOF_{A_{2}-t}|$, where $A_1$ and $A_2$ are the anchors, $t$ is the tag, and $TOF_{A_{1}-t}$ is the _Time of Flight_ (the time a signal takes to travel) from $A_1$ to $t$.

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
- $TOF_{A_{1}-t}-TOF_{A_{2}-t}$ is the difference in ToFs, which is proportional to the difference in distances
- $t_{1r}-t_{2r}$ is simply the difference in reception times $\delta RX$, already in the tag's clock so easy to measure

Therefore, we can write: $\delta TOF=\delta RX-(\tau_{1}-\tau_{2})$
- $(\tau_{1}-\tau_{2})$ is the difference in transmission times $\delta TX$, this is **harder**, because they are reported on different anchor clocks

To calculate $\delta TX = \tau_{1}-\tau_{2}$, the Bitcraze TDoA3 anchor firmware has the anchors perform Two-Way Ranging (TWR) with each other:
- Since $A_1$ and $A_2$ are doing TWR, the ToF between them is known
- Any anchor can then estimate, _in their own clock_, when another anchor transmitted, by subtracting the ToF from the time it received the message from the other anchor.
    - Therefore, any anchor can estimate $\delta TX$ in their own clock. However this result will be in the anchor's clock ticks, whereas we want it in the tag's clock ticks.
    - To solve this, the tag listens to two consecutive transmissions from the same anchor and records the interval between them, $\delta rx'$. The anchor embeds its own estimate of that interval, $\delta tx'$, in the transmissions.
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
`config.py` controls all user-relevant configuration parameters. Anchors and their positions may be defined here in the `ANCHORS` dictionary. During operation, if the tag hears from an anchor that was not defined in `config.py`, it will add it to its internal anchor dictionary **if and only if** the anchor also broadcasts its position. Anchor position broadcasts are overwritten by `config.py` if a value is already present there. In other words, anchor positions may be configured in the firmware or in `config.py`; if there is a conflict, the `config.py` value takes precedence.
#### Instantiating the tag
Create a `TDOA3_Tag` with the tag ID and, optionally, the DW1000 SPI bus and chip-select values (both default to 0). Use it as a context manager (`with TDOA3_Tag(...) as tag:`) so the radio is closed when the block exits.

At initialization, the class copies the configured `ANCHORS` and creates its working state:
- `history` keeps a bounded deque of recent messages per anchor (`HISTORY` messages), while `fresh_anchors` tracks which anchors sent a message in the latest batch. These are used by the aggregation and processing steps below.
- `alpha_history` keeps recent clock-correction coefficients per reference anchor to reject sudden outliers during data processing.
- `TOF_ref_anchors` stores the geometric time of flight between configured anchor pairs, used to validate reported ranging data and reject physically impossible TDoA measurements. It is updated through `update_ref_TOFs()` when a new anchor with a known position is added.
- `initial` is set to the centroid of the configured anchor positions when at least one anchor is configured. It is the starting estimate used by the position solver.

#### Main localization loop
The `TDOA3_Tag` class is the main coordinator and is used to derive the different elements of the TDoA equation. It offers continuous localization through the `TDOA3_Tag.run()` method. 

`TDOA3_Tag.run()` is an infinite loop that: 
1. Captures as many UWB packets as possible from the anchors for a duration corresponding to `LOCALIZATION_FREQUENCY` in `config.py`
    - Relevant function: `DW1000.listen()`
2. Aggregates the raw data per anchor in a dictionary: `{anchor_id: [msg, ..., msg]}` where `msg = {seq, rx, tx, remote_data}`
    - Relevant function: `TDOA3_Tag.aggregate_raw_pkts()`
3. Processes the raw data into TDoA measurements for anchor pairs in a list: `[(TDOA, anchor_1_pos, anchor_2_pos), ...]`
    - Relevant function: `TDOA3_Tag.process_anchor_data()`
4. ***TODO*** - to complete with EKF / multilateration process 
5. Prints and/or saves the resulting position 
##### Data aggregation step: `TDOA3_Tag.aggregate_raw_pkts()`
This function takes a list of received raw packets, `[(raw_packet, local_rx_time), ...]`, and creates a dictionary of relevant data for each anchor.

We iterate through all raw packets. Each one is passed through `TDOA3_Tag.interpret_anchor_msg()` to check its relevance and extract the following:
- Anchor ID 
- Transaction SEQ number
- Transmission time in the anchor's clock 
- Remote anchor data reported by the anchor
    - This is the latest information the anchor had about other anchors, in a dictionary of shape `{remoteAnchorId: (seq, rx_ts, r_tof), ...}`.
        - `seq`: SEQ of the last message the remote anchor sent
        - `rx_ts`: Reception time of that message in the reporting anchor's clock (not the remote anchor's)
        - `r_tof`: Measured ToF to the reporting anchor in radio ticks
- Anchor position 

Then, anchors that were not defined in `config.py` are added to the tag's internal `anchors` dictionary if their firmware position is included in the message. If no position is shared, the anchor is ignored and an error is printed in the terminal.

As during initialization, whenever a new anchor is added, `self.update_ref_TOFs()` is called to keep the distance information between anchors up to date.

The extracted information is added to the internal `self.history` dictionary, shaped as `{anchor_id: [msg, ..., msg]}`, where `msg = {seq, rx, tx, remote_data}`. Each value is a `deque` with a maximum length set by the `HISTORY` constant. This keeps recent messages available for processing. In theory, only the two most recent messages are needed to compute the required values. However, retaining a longer history increases the chance of finding messages with matching `SEQ` values, which is necessary in `process_anchor_data()`.

We also add the IDs of anchors heard in the current batch to the internal set `self.fresh_anchors`, which is refreshed on each call of `aggregate_raw_pkts()`. This lets us keep older anchor data while tracking which anchors transmitted recently.

##### Data processing step: `TDOA3_Tag.process_anchor_data()` and `TDOA3_Tag._tdoa_from_ref()`
The processing step converts the aggregated messages into TDoA measurements paired with the positions of the anchors involved. `process_anchor_data()` tries each anchor that sent a message in the latest batch as the reference anchor, using `_tdoa_from_ref()` to build measurements from it. It returns the set of measurements from the reference that produced the most valid pairs; if none can be formed, the result is an empty list.

Choosing a reference anchor gives a compact set of independent measurements. With three anchors A, B, and C, let $t_A$, $t_B$, and $t_C$ denote the tag's reception times for their messages. We can use the two differences relative to A: $\Delta t_{AB}=t_A-t_B$ and $\Delta t_{AC}=t_A-t_C$. The third pair adds no independent information because $\Delta t_{BC}=t_B-t_C=\Delta t_{AC}-\Delta t_{AB}$. In general, $n$ anchors provide at most $n-1$ independent pairwise differences; all other pairs can be expressed as combinations of those. The code tries each recently heard anchor as the reference and keeps the one that yields the most valid measurements.

For a reference anchor, `_tdoa_from_ref()` first checks that its two newest messages are consecutive according to the wrapping SEQ counter, as this is *required* to evaluate `alpha`. It uses their tag-side reception times and anchor-side transmission times to estimate `alpha`, the conversion from reference-anchor clock ticks to tag-clock ticks. It rejects the reference if the estimate is zero or outside the configured error bounds, or if it differs too much from the recent median; accepted values are added to that anchor's `alpha_history`.

Now that the clock correction coefficient is known, we build TDoA pairs with respect to the reference anchor. The function then considers each remote-anchor report in the reference's newest message. A report is used only when:
- The tag also *independently* received the exact reported remote message, identified by its SEQ, in its retained history.
- The report includes a measured time of flight, and that measurement agrees with the geometric reference distance within the validation tolerance.

For each surviving pair, the reference anchor's reported reception time and measured time of flight are used to estimate the remote anchor's transmission time in the reference clock: $\widehat{tx}_2 = rx_{2 \to 1} - TOF_{2 \to 1}$. Subtracting this from the reference transmission time gives the transmit-time difference, $\Delta TX = tx_1 - \widehat{tx}_2$. The tag's reception timestamps give the reception-time difference, $\Delta RX = rx_1 - rx_2$. The TDoA in tag-clock ticks is then $\Delta TDoA = \Delta RX - \alpha\Delta TX$, where $\alpha$ converts the reference-clock transmit-time difference into tag-clock ticks.

The timestamps have finite widths and can wrap around. `wrap_diff()` computes a difference modulo the counter range, but by itself cannot distinguish aliases if the true 32-bit transmit-time difference spans more than half that range. `unwrap_delta()` resolves this ambiguity by selecting the modular alias of $\Delta TX$ closest to the independent estimate $\Delta RX/\alpha$. The function rejects measurements whose absolute TDoA exceeds the geometric time of flight between the anchors, since that would imply a path-length difference greater than the distance between them.

Each accepted result is converted from DW1000 clock ticks to seconds and returned as `(TDoA, reference_anchor_position, remote_anchor_position)`.
