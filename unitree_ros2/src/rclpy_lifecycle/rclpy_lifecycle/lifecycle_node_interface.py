import enum
from lifecycle_msgs.msg import Transition, State


class LifecycleNodeInterface:
    """Interface class for a managed node.

    Virtual functions as defined in
    http://design.ros2.org/articles/node_lifecycle.html
    If the callback function returns successfully,
    the specified transition is completed.
    If the callback function fails or throws an
    uncaught exception, the on_error function is
    called.
    By default, all functions remain optional to overwrite
    and return true. Except the on_error function, which
    returns false and thus goes to shutdown/finalize state.
    """

    class CallbackReturn(enum.IntEnum):
        """An enumeration of the possible callback returns."""

        SUCCESS = Transition.TRANSITION_CALLBACK_SUCCESS
        FAILURE = Transition.TRANSITION_CALLBACK_FAILURE
        ERROR = Transition.TRANSITION_CALLBACK_ERROR


    def on_configure(self, previous_state: State) -> CallbackReturn:
        """Callback function for configure transition.

        :return: success by default
        """
        return LifecycleNodeInterface.CallbackReturn.SUCCESS


    def on_cleanup(self, previous_state: State) -> CallbackReturn:
        """Callback function for cleanup transition.

        :return: success by default
        """
        return LifecycleNodeInterface.CallbackReturn.SUCCESS


    def on_shutdown(self, previous_state: State) -> CallbackReturn:
        """Callback function for shutdown transition.

        :return: success by default
        """
        return LifecycleNodeInterface.CallbackReturn.SUCCESS


    def on_activate(self, previous_state: State) -> CallbackReturn:
        """Callback function for activate transition.

        :return: success by default
        """
        return LifecycleNodeInterface.CallbackReturn.SUCCESS


    def on_deactivate(self, previous_state: State) -> CallbackReturn:
        """Callback function for deactivate transition.

        :return: success by default
        """
        return LifecycleNodeInterface.CallbackReturn.SUCCESS


    def on_error(self, previous_state: State) -> CallbackReturn:
        """Callback function for error transition.

        :return: success by default
        """
        return LifecycleNodeInterface.CallbackReturn.SUCCESS
