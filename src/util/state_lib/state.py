from __future__ import annotations

from abc import ABC, abstractmethod


class State(ABC):
    """
    Abstract class that represents a state in the state machine.
    """

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def on_enter(self, context):
        """
        Called exactly once when the state is entered. If any state needs to be initialized, do it here.
        :param context: The context object that is passed to the state machine.
        """
        raise NotImplementedError

    @abstractmethod
    def on_exit(self, context):
        """
        Called exactly once when the state is exited.
        No cleanup of internal state is necessary since this state will be destroyed.
        An example usecase of this may be to write to an external log or send a service call to an external system.
        :param context: The context object that is passed to the state machine.
        """
        raise NotImplementedError

    @abstractmethod
    def on_loop(self, context) -> State:
        """
        Called repeatedly while the state is active.
        :param context: The context object that is passed to the state machine.
        :return: The next state to transition to. If the state should not change, return self.
        """
        raise NotImplementedError

    def __repr__(self):
        return self.__class__.__name__

    def __str__(self):
        return self.__repr__()

    def __eq__(self, other):
        return self.__class__ == other.__class__

    def __hash__(self):
        return hash(self.__class__)

    def __ne__(self, other):
        return not self.__eq__(other)


##state to be returned to signfy that the state machine should exit
class ExitState(State):
    def on_enter(self, ctx):
        pass

    def on_exit(self, ctx):
        pass

    def on_loop(self, ctx) -> State:
        return self
