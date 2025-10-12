from rclpy.node import Node
from rclpy.qos import qos_profile_services_default, qos_profile_action_status_default
from lifecycle_msgs.srv import ChangeState, GetState, GetAvailableStates, GetAvailableTransitions
from lifecycle_msgs.msg import TransitionEvent, State, Transition, TransitionDescription

from .lifecycle_state_machine import LifecycleStateMachine
from .lifecycle_node_interface import LifecycleNodeInterface

from typing import Callable
CallbackFunction = Callable[[State], LifecycleNodeInterface.CallbackReturn]


class LifecycleNodeImpl:
    def __init__(self, node: Node):
        self.__node = node
        self.__cb_map = dict()

    def init(self, enable_communication_interface: bool = True):
        if enable_communication_interface:
            # initialize change state service
            self.__srv_change_state = self.__node.create_service(ChangeState, 
                "~/change_state", self.__on_change_state,
                qos_profile=qos_profile_services_default)
            # initialize get state service
            self.__srv_get_state = self.__node.create_service(GetState, 
                "~/get_state", self.__on_get_state,
                qos_profile=qos_profile_services_default)
            # initialize get available states service
            self.__srv_get_available_states = self.__node.create_service(GetAvailableStates, 
                "~/get_available_states", self.__on_get_available_states,
                qos_profile=qos_profile_services_default)
            # initialize get available transitions service
            self.__srv_get_available_transitions = self.__node.create_service(GetAvailableTransitions, 
                "~/get_available_transitions", self.__on_get_available_transitions,
                qos_profile=qos_profile_services_default)
            # initialize get transition graph service
            self.__srv_get_transition_graph = self.__node.create_service(GetAvailableTransitions, 
                "~/get_transition_graph", self.__on_get_transition_graph,
                qos_profile=qos_profile_services_default)
            # initialize publisher
            self.__pub_transition_event = self.__node.create_publisher(TransitionEvent, 
                "~/transition_event", qos_profile=qos_profile_action_status_default)

        self.__enable_communication_interface = enable_communication_interface
        # initialize state machine
        self.__state_machine = LifecycleStateMachine.init_default_state_machine()
        # create
        req = ChangeState.Request(transition = Transition(id=Transition.TRANSITION_CREATE, label='create'))
        self.__on_change_state(req, ChangeState.Response())

    @property
    def state_machine(self) -> LifecycleStateMachine:
        return self.__state_machine

    def __on_change_state(self, request: ChangeState.Request, response: ChangeState.Response):
        self.__node.get_logger().debug(f"[lifecycle] received change request {request}")
        current_state_id = self.__state_machine.get_current_state().id
        if len(request.transition.label) > 0:
            try:
                transition = self.__state_machine.get_transition_by_label(current_state_id, request.transition.label)
                transition_id = transition.transition.id
            except Exception as ex:
                self.__node.get_logger().error(f"{ex}")
                response.success = False
                return response
        else:
            transition_id = request.transition.id
        self.__node.get_logger().debug(f"[lifecycle] transition id {transition_id}")
        
        response.success = self.__change_state(transition_id)
        return response

    def __on_get_state(self, request: GetState.Request, response: GetState.Response):
        response.current_state = self.__state_machine.get_current_state()
        return response

    def __on_get_available_states(self, request: GetAvailableStates.Request, response: GetAvailableStates.Response):
        response.available_states = self.__state_machine.get_states()
        return response

    def __on_get_available_transitions(self, request: GetAvailableTransitions.Request, response: GetAvailableTransitions.Response):
        current_state_id = self.__state_machine.get_current_state().id
        response.available_transitions = self.__state_machine.get_valid_transitions(current_state_id)
        return response

    def __on_get_transition_graph(self, request: GetAvailableTransitions.Request, response: GetAvailableTransitions.Response):
        response.available_transitions = [t 
            for s in self.__state_machine.get_states()
            for t in self.__state_machine.get_valid_transitions(s.id) 
            ]
        return response

    def __publish_notification(self, start: State, goal: State, transition: TransitionDescription):
        if self.__enable_communication_interface:
            msg = TransitionEvent()
            msg.start_state = start
            msg.goal_state = goal
            msg.transition = transition
            self.__pub_transition_event.publish(msg)

    def register_callback(self, lifecycle_transition: int, cb: CallbackFunction) -> bool:
        self.__cb_map[lifecycle_transition] = cb
        self.__node.get_logger().debug(f"[lifecycle] callback map {lifecycle_transition} -> {cb}")
        return True

    def __change_state(self, transition_id: int) -> bool:
        current_state = self.__state_machine.get_current_state()
        initial_state = State(id=current_state.id, label=current_state.label)
        self.__node.get_logger().debug(f"[lifecycle] current state: {current_state}")
        if not self.__trigger_transition_by_id(current_state.id, transition_id):
            self.__node.get_logger().error(f"[lifecycle] impossible to start transition {transition_id} from current state {current_state}")
            return False
        cb_return_code = self.__execute_callback(self.__state_machine.get_current_state().id, initial_state)
        if cb_return_code == Transition.TRANSITION_CALLBACK_SUCCESS:
            transition_label = "transition_success"
        elif cb_return_code == Transition.TRANSITION_CALLBACK_FAILURE:
            transition_label = "transition_failure"
        else:
            transition_label = "transition_error"
        current_state = self.__state_machine.get_current_state()
        if not self.__trigger_transition_by_label(current_state.id, transition_label):
            # failed to finish transition
            self.__node.get_logger().error(f"[lifecycle] failed to finish transition {transition_label}")
            return False
        # error handling
        if cb_return_code == LifecycleNodeInterface.CallbackReturn.ERROR:
            error_cb_code = self.__execute_callback(self.__state_machine.get_current_state().id, initial_state)
            if error_cb_code == Transition.TRANSITION_CALLBACK_SUCCESS:
                error_label = "transition_success"
            elif error_cb_code == Transition.TRANSITION_CALLBACK_FAILURE:
                error_label = "transition_failure"
            else:
                error_label = "transition_error"
            current_state = self.__state_machine.get_current_state()
            if not self.__trigger_transition_by_label(current_state.id, error_label):
                self.__node.get_logger().error(f"[lifecycle] failed to call cleanup on error state with transition {error_label}")
                return False
        # valid transition
        return True

    def __trigger_transition_by_id(self, state_id: int, transition_id: int) -> bool:
        try:
            transition = self.__state_machine.get_transition_by_id(state_id, transition_id)
        except StopIteration:
            self.__node.get_logger().debug(f"transition {transition_id} does not exist from state {state_id}")
            return False
        return self.__trigger_transition(transition.transition)

    def __trigger_transition_by_label(self, state_id: int, transition_label: str) -> bool:
        try:
            transition = self.__state_machine.get_transition_by_label(state_id, transition_label)
        except StopIteration:
            self.__node.get_logger().debug(f"transition {transition_label} does not exist from state {state_id}")
            return False
        return self.__trigger_transition(transition.transition)

    def __trigger_transition(self, transition: Transition) -> bool:
        start_state = self.__state_machine.get_current_state()
        self.__node.get_logger().debug(f"[lifecycle] trigger transition {transition} from {start_state}")
        if self.__state_machine.trigger_transition(transition):
            goal_state = self.__state_machine.get_current_state()
            self.__node.get_logger().debug(f"[lifecycle] goal state: {goal_state}")
            # publish
            self.__publish_notification(start_state, goal_state, transition)
            return True
        else:
            self.__node.get_logger().debug(f"[lifecycle] error triggering transition {transition} from {start_state}")
            return False

    def __execute_callback(self, cb_id: int, previous_state: State) -> LifecycleNodeInterface.CallbackReturn:
        cb_success = LifecycleNodeInterface.CallbackReturn.SUCCESS
        if cb_id in self.__cb_map:
            try:
                self.__node.get_logger().debug(f"[lifecycle] executing callback {cb_id}:{self.__cb_map[cb_id]} from {previous_state}")
                cb_success = self.__cb_map[cb_id](previous_state)
            except Exception as ex:
                self.__node.get_logger().debug(f"[lifecycle] catched callback exception {ex}")
                cb_success = LifecycleNodeInterface.CallbackReturn.ERROR
        self.__node.get_logger().debug(f"[lifecycle] callback returned {cb_success}")
        return cb_success