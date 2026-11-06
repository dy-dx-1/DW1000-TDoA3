"""
Synthetic tag: replays recorded raw data (from TDOA3_Tag.save_runs()) through the real tag's run(), without a DW1000.
All the computations come from tdoa3_tag.py, only the listening step is replaced.
"""
import pickle
import time

from tdoa3_tag import TDOA3_Tag


class SyntheticTag(TDOA3_Tag):
    """
    A TDOA3_Tag whose listen() returns the next recorded batch instead of listening with the radio.
    run() ends once all batches have been processed. Only use with a context manager, like the real tag.

    ARGS:
    - path: .pkl recording, a list of batches of [(raw_packet, local_rx_time), ...]
    - realtime_period_s: Wait this long before each batch to replay at the recording's pace (e.g. 1 for the
                         1 s batches of *_1s_raw_datas.pkl). None replays as fast as possible.
    - Other arguments are the same as TDOA3_Tag (bus and cs are ignored)
    """
    def __init__(self, path:str, realtime_period_s:float|None=None, **kwargs):
        # Loaded before the output controllers are built, so a bad path doesn't leave them open
        with open(path, "rb") as file:
            self._batches = iter(pickle.load(file))
        self._realtime_period_s = realtime_period_s
        super().__init__(**kwargs)

    def _open_radio(self, bus, cs):
        return None

    def listen(self)->list[tuple]|None:
        if self._realtime_period_s:
            time.sleep(self._realtime_period_s)
        return next(self._batches, None)
