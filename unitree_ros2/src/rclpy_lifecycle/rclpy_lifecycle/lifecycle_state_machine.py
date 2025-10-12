from typing import List, Dict
from collections import defaultdict

from lifecycle_msgs.msg import State, Transition, TransitionDescription


class LifecycleStateMachine:

    def __init__(self):
        self.__states = dict()
        self.__transitions = list()
        self.__valid_transitions : Dict[int, List[TransitionDescription]] = defaultdict(list)
        self.__current_state = State.PRIMARY_STATE_UNKNOWN

    def register_state(self, state: State) -> bool:
        if state.id in self.__states:
            # state already registered
            return False
        else:
            self.__states[state.id] = state
            return True

    def register_transition(self, transition: TransitionDescription) -> bool:
        if transition.start_state.id not in self.__states:
            # start state not registered
            return False
        elif transition.goal_state.id not in self.__states:
            # goal state not registered
            return False
        self.__transitions.append(transition.transition)
        self.__valid_transitions[transition.start_state.id].append(transition)
        return True

    def get_state(self, state_id: int) -> State:
        return self.__states[state_id]

    def get_current_state(self) -> State:
        return self.get_state(self.__current_state)

    def get_states(self) -> List[State]:
        return list(self.__states.values())

    def get_transitions(self) -> List[Transition]:
        return self.__transitions

    def get_valid_transitions(self, state_id: int) -> List[TransitionDescription]:
        return self.__valid_transitions[state_id]

    def get_transition_by_id(self, state_id: int, transition_id: int) -> TransitionDescription:
        return next(t for t in self.__valid_transitions[state_id] if t.transition.id == transition_id)

    def get_transition_by_label(self, state_id: int, label: str) -> TransitionDescription:
        return next(t for t in self.__valid_transitions[state_id] if t.transition.label == label)

    def trigger_transition(self, transition: Transition) -> bool:
        valid_transitions = self.__valid_transitions[self.__current_state]
        for t in valid_transitions:
            if t.start_state.id == self.__current_state and t.transition.id == transition.id:
                # applying transition
                self.__current_state = t.goal_state.id
                return True
        return False

    @classmethod
    def init_default_state_machine(cls) -> 'LifecycleStateMachine':
        """Initialize a default state machine.

        This function initializes a default state machine. It registers all: primary states,
        transition states, transitions and the initial state. The primary state is unconfigured.
 
        States: unknown, unconfigured, inactive, active and finalized.
        Transition states: configuring, cleaningup, activating, deactivating, errorprocessing
                           and shuttingdown.
        Transitions:
           - unconfigured to configuring
           - unconfigured to shuttingdown
           - configuring to inactive
           - configuring to unconfigured
           - configuring to errorprocessing
           - inactive to activating
           - inactive to cleaningup
           - inactive to shuttingdown
           - cleaningup to unconfigured
           - cleaningup to inactive
           - cleaniningup to errorprocessing
           - activating to active
           - activating to inactive
           - activating to errorprocessing
           - active to deactivating
           - active to shuttingdown
           - deactivating to inactive
           - deactivating to active
           - deactivating to errorprocessing
           - shutting down to finalized
           - shutting down to finalized
           - shutting down to errorprocessing
           - errorprocessing to uncofigured
           - errorprocessing to finalized
           - errorprocessing to finalized
        """
        state_machine = cls()
        # register all primary states
        state_machine.register_state(State(id=State.PRIMARY_STATE_UNKNOWN, label="unknown"))
        state_machine.register_state(State(id=State.PRIMARY_STATE_UNCONFIGURED, label="unconfigured"))
        state_machine.register_state(State(id=State.PRIMARY_STATE_INACTIVE, label="inactive"))
        state_machine.register_state(State(id=State.PRIMARY_STATE_ACTIVE, label="active"))
        state_machine.register_state(State(id=State.PRIMARY_STATE_FINALIZED, label="finalized"))
        unknown_state = state_machine.get_state(State.PRIMARY_STATE_UNKNOWN)
        unconfigured_state = state_machine.get_state(State.PRIMARY_STATE_UNCONFIGURED)
        inactive_state = state_machine.get_state(State.PRIMARY_STATE_INACTIVE)
        active_state = state_machine.get_state(State.PRIMARY_STATE_ACTIVE)
        finalized_state = state_machine.get_state(State.PRIMARY_STATE_FINALIZED)
        
        # register all transition states
        state_machine.register_state(State(id=State.TRANSITION_STATE_CONFIGURING, label="configuring"))
        state_machine.register_state(State(id=State.TRANSITION_STATE_CLEANINGUP, label="cleaningup"))
        state_machine.register_state(State(id=State.TRANSITION_STATE_SHUTTINGDOWN, label="shuttingdown"))
        state_machine.register_state(State(id=State.TRANSITION_STATE_ACTIVATING, label="activating"))
        state_machine.register_state(State(id=State.TRANSITION_STATE_DEACTIVATING, label="deactivating"))
        state_machine.register_state(State(id=State.TRANSITION_STATE_ERRORPROCESSING, label="errorprocessing"))
        configuring_state = state_machine.get_state(State.TRANSITION_STATE_CONFIGURING)
        errorprocessing_state = state_machine.get_state(State.TRANSITION_STATE_ERRORPROCESSING)
        cleaningup_state = state_machine.get_state(State.TRANSITION_STATE_CLEANINGUP)
        activating_state = state_machine.get_state(State.TRANSITION_STATE_ACTIVATING)
        deactivating_state = state_machine.get_state(State.TRANSITION_STATE_DEACTIVATING)
        shuttingdown_state = state_machine.get_state(State.TRANSITION_STATE_SHUTTINGDOWN)
        
        # register all transitions
        ## register transition from unknown to unconfigured
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_CREATE, label="create"), start_state=unknown_state, goal_state=unconfigured_state))
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_CREATE, label="transition_success"), start_state=unconfigured_state, goal_state=unconfigured_state))
        ## register transition from unconfigured to configuring
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_CONFIGURE, label="configure"), start_state=unconfigured_state, goal_state=configuring_state))
        ## register transition from configuring to inactive
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_CONFIGURE_SUCCESS, label="transition_success"), start_state=configuring_state, goal_state=inactive_state))
        ## register transition from configuring to unconfigured
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_CONFIGURE_FAILURE, label="transition_failure"), start_state=configuring_state, goal_state=unconfigured_state))
        ## register transition from configuring to errorprocessing
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_CONFIGURE_ERROR, label="transition_error"), start_state=configuring_state, goal_state=errorprocessing_state))
        ## register transition from inactive to cleaningup
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_CLEANUP, label="cleanup"), start_state=inactive_state, goal_state=cleaningup_state))
        ## register transition from cleaningup to unconfigured
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_CLEANUP_SUCCESS, label="transition_success"), start_state=cleaningup_state, goal_state=unconfigured_state))
        ## register transition from cleaningup to inactive
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_CLEANUP_FAILURE, label="transition_failure"), start_state=cleaningup_state, goal_state=inactive_state))
        ## register transition from cleaningup to errorprocessing
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_CLEANUP_ERROR, label="transition_error"), start_state=cleaningup_state, goal_state=errorprocessing_state))
        ## register transition from inactive to activating
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ACTIVATE, label="activate"), start_state=inactive_state, goal_state=activating_state))
        ## register transition from activating to active
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_ACTIVATE_SUCCESS, label="transition_success"), start_state=activating_state, goal_state=active_state))
        ## register transition from activating to inactive
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_ACTIVATE_FAILURE, label="transition_failure"), start_state=activating_state, goal_state=inactive_state))
        ## register transition from activating to errorprocessing
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_ACTIVATE_ERROR, label="transition_error"), start_state=activating_state, goal_state=errorprocessing_state))
        ## register transition from active to deactivating
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_DEACTIVATE, label="deactivate"), start_state=active_state, goal_state=deactivating_state))
        ## register transition from deactivating to inactive
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_DEACTIVATE_SUCCESS, label="transition_success"), start_state=deactivating_state, goal_state=inactive_state))
        ## register transition from deactivating to active
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_DEACTIVATE_FAILURE, label="transition_failure"), start_state=deactivating_state, goal_state=active_state))
        ## register transition from deactivating to errorprocessing
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_DEACTIVATE_ERROR, label="transition_error"), start_state=deactivating_state, goal_state=errorprocessing_state))
        ## register transition from unconfigured to shuttingdown
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, label="shutdown"), start_state=unconfigured_state, goal_state=shuttingdown_state))
        ## register transition from inactive to shuttingdown
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_INACTIVE_SHUTDOWN, label="shutdown"), start_state=inactive_state, goal_state=shuttingdown_state))
        ## register transition from active to shuttingdown
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ACTIVE_SHUTDOWN, label="shutdown"), start_state=active_state, goal_state=shuttingdown_state))
        ## register transition from shuttingdown to finalized
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_SHUTDOWN_SUCCESS, label="transition_success"), start_state=shuttingdown_state, goal_state=finalized_state))
        ## register transition from shuttingdown to finalized
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_SHUTDOWN_FAILURE, label="transition_failure"), start_state=shuttingdown_state, goal_state=finalized_state))
        ## register transition from shuttingdown to errorprocessing
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_SHUTDOWN_ERROR, label="transition_error"), start_state=shuttingdown_state, goal_state=errorprocessing_state))
        ## register transition from errorprocessing to unconfigured
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_ERROR_SUCCESS, label="transition_success"), start_state=errorprocessing_state, goal_state=unconfigured_state))
        ## register transition from errorprocessing to finalized
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_ERROR_FAILURE, label="transition_failure"), start_state=errorprocessing_state, goal_state=finalized_state))
        ## register transition from errorprocessing to finalized
        state_machine.register_transition(TransitionDescription(transition=Transition(id=Transition.TRANSITION_ON_ERROR_ERROR, label="transition_error"), start_state=errorprocessing_state, goal_state=finalized_state))
        
        # set the initial state to unconfigured
        state_machine.__current_state = State.PRIMARY_STATE_UNKNOWN
        return state_machine