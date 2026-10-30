from pose import CycleUpdate


class OutputController:
    """
    Base class for everything that consumes localization results (console, CSV, ...).
    The tag calls publish() once per loop iteration and close() once on exit.
    """
    def publish(self, update: CycleUpdate) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass
