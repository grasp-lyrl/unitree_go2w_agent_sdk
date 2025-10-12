from typing import List, Union, Callable
from rclpy.node import Node
from lifecycle_msgs.msg import State, Transition

from .lifecycle_node_interface import LifecycleNodeInterface
from .lifecycle_node_impl import LifecycleNodeImpl

CallbackFunction = Callable[[State], LifecycleNodeInterface.CallbackReturn]

class LifecycleNode(Node, LifecycleNodeInterface):
    """LifecycleNode for creating lifecycle components.

    has lifecycle nodeinterface for configuring this node.
    """

    def __init__(self, node_name: str, *, 
                 enable_communication_interface: bool = True,
                 **options: dict):
        """Create a new lifecycle node with the specified name.

        @param node_name: Name of the node.
        @param options Additional options to control creation of the node.
        @param enable_communication_interface: Deciding whether the communication interface of the underlying node shall be enabled.
        """
        super().__init__(node_name, **options)
        # initialize impl
        self.__impl = LifecycleNodeImpl(self)
        self.__impl.init(enable_communication_interface)
        # register callbacks
        self.register_on_configure(self.on_configure)
        self.register_on_cleanup(self.on_cleanup)
        self.register_on_shutdown(self.on_shutdown)
        self.register_on_activate(self.on_activate)
        self.register_on_deactivate(self.on_deactivate)
        self.register_on_error(self.on_error)

    '''
    def create_publisher(self, msg_type, topic: str, qos_profile, **options):
        """Create and return a Publisher.
        
        @param msg_type: The type of ROS messages the publisher will publish.
        @param topic: The name of the topic the publisher will publish to.
        @param qos: The Quality of Service settings for this publisher.
        @param options: The publisher options for this publisher.
        @return: the created lifecycle publisher.
        """
        #TODO
        pass
    '''

    def get_current_state(self) -> State:
        """Return the current State.

        @return: the current state
        """
        return self.__impl.state_machine.get_current_state()

    def get_available_states(self) -> List[State]:
        """Return a list with the available states.

        @return list with the available states.
        """
        return self.__impl.state_machine.get_available_states()

    def get_available_transitions(self) -> List[Transition]:
        """Return a list with the current available transitions.

        @return list with the current available transitions.
        """
        return self.__impl.state_machine.get_available_transitions()

    def get_transition_graph(self) -> List[Transition]:
        """Return a list with the all the transitions.

        @return list with the all the transitions in the transition graph.
        """
        return self.__impl.state_machine.get_transition_graph()

    def trigger_transition(self, 
                           transition: Union[int, Transition]) -> LifecycleNodeInterface.CallbackReturn:
        """Trigger the specified transition and get the callback return code.

        @return: transition callback return code
        """
        if isinstance(transition, Transition):
            return self.__impl.state_machine.trigger_transition(transition)
        elif isinstance(transition, Transition):
            return self.__impl.state_machine.trigger_transition(Transition(id=transition))
        else:
            return LifecycleNodeInterface.CallbackReturn.ERROR

    def configure(self) -> LifecycleNodeInterface.CallbackReturn:
        """Trigger the configure transition and get the callback return code.

        @return: transition callback return code
        """
        return self.trigger_transition(Transition.TRANSITION_CONFIGURE)

    def cleanup(self) -> LifecycleNodeInterface.CallbackReturn:
        """Trigger the cleanup transition and get the callback return code.

        @return: transition callback return code
        """
        return self.trigger_transition(Transition.TRANSITION_CLEANUP)
    
    def activate(self) -> LifecycleNodeInterface.CallbackReturn:
        """Trigger the activate transition and get the callback return code.

        @return: transition callback return code
        """
        return self.trigger_transition(Transition.TRANSITION_ACTIVATE)
    
    def deactivate(self) -> LifecycleNodeInterface.CallbackReturn:
        """Trigger the deactivate transition and get the callback return code.

        @return: transition callback return code
        """
        return self.trigger_transition(Transition.TRANSITION_DEACTIVATE)
    
    def shutdown(self) -> LifecycleNodeInterface.CallbackReturn:
        """Trigger the shutdown transition and get the callback return code.

        @return: transition callback return code
        """
        return self.__impl.state_machine.trigger_transition("shutdown")
    
    def register_on_configure(self, fcn: CallbackFunction) -> bool:
        """Register the configure callback.

        This callback will be called when the transition to this state is triggered
        @param fcn: callback function to call
        @return: always true
        """
        return self.__impl.register_callback(State.TRANSITION_STATE_CONFIGURING, fcn)

    def register_on_cleanup(self, fcn: CallbackFunction) -> bool:
        """Register the cleanup callback.

        This callback will be called when the transition to this state is triggered
        @param fcn: callback function to call
        @return: always true
        """
        return self.__impl.register_callback(State.TRANSITION_STATE_CLEANINGUP, fcn)

    def register_on_shutdown(self, fcn: CallbackFunction) -> bool:
        """Register the shutdown callback.

        This callback will be called when the transition to this state is triggered
        @param fcn: callback function to call
        @return: always true
        """
        return self.__impl.register_callback(State.TRANSITION_STATE_SHUTTINGDOWN, fcn)

    def register_on_activate(self, fcn: CallbackFunction) -> bool:
        """Register the activate callback.

        This callback will be called when the transition to this state is triggered
        @param fcn: callback function to call
        @return: always true
        """
        return self.__impl.register_callback(State.TRANSITION_STATE_ACTIVATING, fcn)

    def register_on_deactivate(self, fcn: CallbackFunction) -> bool:
        """Register the deactivate callback.

        This callback will be called when the transition to this state is triggered
        @param fcn: callback function to call
        @return: always true
        """
        return self.__impl.register_callback(State.TRANSITION_STATE_DEACTIVATING, fcn)

    def register_on_error(self, fcn: CallbackFunction) -> bool:
        """Register the error callback.

        This callback will be called when the transition to this state is triggered
        @param fcn: callback function to call
        @return: always true
        """
        return self.__impl.register_callback(State.TRANSITION_STATE_ERRORPROCESSING, fcn)
