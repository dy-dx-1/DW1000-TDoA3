from pose import CycleUpdate
from outputs.base import OutputController


class ConsoleOutputController(OutputController):
    """Prints every cycle's result to stdout."""
    def __init__(self, tag_id):
        self.tag_id = tag_id

    def publish(self, update: CycleUpdate) -> None:
        print(f"Sending a processed list of size {update.n_pairs}")
        position = update.pose.position if update.pose is not None else None
        print(f"[INFO] -----> New position estimated: {position}")
