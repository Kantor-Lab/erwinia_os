import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

import yasmin
import time
from yasmin import Blackboard
from yasmin import State
from yasmin import StateMachine
from yasmin_ros import MonitorState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import TIMEOUT
from yasmin_viewer import YasminViewerPub

# Define the FooState class, inheriting from the State class
class Navigation_state(State):
    """
    Represents the Foo state in the state machine.

    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self) -> None:
        """
        Initializes the Navigation State instance, setting up the outcomes.

        Outcomes:
            navigation_done: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["nav_done", "global_plan_done"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the navigation state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state navigation")
        nav_stat_param = self.get_parameter('nav_stat').get_parameter_value().bool_value
        plan_stat_param = self.get_parameter('plan_stat').get_parameter_value().bool_value

        if nav_stat_param:
            blackboard["nav_status"] = "Navigation done"
            return "nav_done"
        if plan_stat_param:
            blackboard["plan_status"] = "Plan finished"
            return "global_plan_done"
        # else:
        #     return "outcome2"
        
class ReadJoyState(MonitorState):
    """
    MonitorState subclass to handle Odometry messages.

    This state monitors Odometry messages from the specified ROS topic,
    logging them and transitioning based on the number of messages received.

    Attributes:
        times (int): The number of messages to monitor before transitioning
                     to the next outcome.

    Parameters:
        times (int): The initial count of how many Odometry messages to
                     process before changing state.

    Methods:
        monitor_handler(blackboard: Blackboard, msg: Odometry) -> str:
            Handles incoming Odometry messages, logging the message and
            returning the appropriate outcome based on the remaining count.
    """

    def __init__(self, times: int) -> None:
        """
        Initializes the PrintOdometryState.

        Parameters:
            times (int): The number of Odometry messages to monitor before
                         transitioning to the next outcome.
        """
        super().__init__(
            Joy,  # msg type
            "joy",  # topic name
            ["resume_nav", "move_fwd"],  # outcomes
            self.monitor_handler,  # monitor handler callback
            qos=qos_profile_sensor_data,  # qos for the topic subscription
            msg_queue=10,  # queue for the monitor handler callback
            timeout=10,  # timeout to wait for messages in seconds
        )
        self.times = times

    def monitor_handler(self, blackboard: Blackboard, msg: Joy) -> str:
        """
        Handles incoming Odometry messages.

        This method is called whenever a new Odometry message is received.
        It logs the message, decrements the count of messages to process,
        and determines the next state outcome.

        Parameters:
            blackboard (Blackboard): The shared data storage for states.
            msg (Odometry): The incoming Odometry message.

        Returns:
            str: The next state outcome, either "outcome1" to continue
                 monitoring or "outcome2" to transition to the next state.

        Exceptions:
            None
        """
        yasmin.YASMIN_LOG_INFO(msg)
        x_button_state = msg.buttons[3]
        y_button_state = msg.buttons[4]

        if x_button_state:
            return "move_fwd"
        if y_button_state:
            self.set_parameters([Parameter('pruning_status', Parameter.Type.BOOL, True)])
            return "resume_nav"

class MoveForwardState(State):
    """
    Represents the Bar state in the state machine.
    """

    def __init__(self) -> None:
        """
        Initializes the BarState instance, setting up the outcome.

        Outcomes:
            outcome3: Indicates the state should transition back to the Foo state.
        """
        super().__init__(outcomes=["moved"])
        self.cmd_vel_pub = None  # Will initialize later

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Bar state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which will always be "outcome3".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        node = blackboard["node"]  # Retrieve the shared node
        yasmin.YASMIN_LOG_INFO("Executing state move_forward")
        if self.cmd_vel_pub is None:
            self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # Publish the forward command
        dist_to_move = 0.4
        linear_vel = 0.1
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        node.get_logger().info("Published forward velocity")

        time.sleep(dist_to_move*linear_vel)

        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        node.get_logger().info("Published stop velocity")

        return "moved"
    
def main():
    """
    Main function to initialize and run the ROS 2 state machine.

    This function initializes ROS 2, sets up logging, creates a finite state
    machine (FSM), adds states to the FSM, and executes the FSM. It handles
    cleanup and shutdown of ROS 2 gracefully.

    Exceptions:
        KeyboardInterrupt: Caught to allow graceful cancellation of the
                          state machine during execution.
    """
    yasmin.YASMIN_LOG_INFO("state_machine_joystick")

    # Initialize ROS 2
    rclpy.init()

    node = rclpy.create_node("state_machine_node")
    Blackboard()["node"] = node  
    # Set ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["plan_finished"])

    # Add states to the FSM
    sm.add_state(
        "Navigation",
        Navigation_state(),
        transitions={
            "nav_done": "Reading_Joy",
            "global_plan_done": "plan_finished",
        },
    )
    
    sm.add_state(
        "Reading_Joy",
        ReadJoyState(5),
        transitions={
            "move_fwd": "Move_forward",
            "resume_nav": "Navigation",
        },
    )

    sm.add_state(
        "Move_forward",
        MoveForwardState(),
        transitions={
            "moved": "Reading_Joy",
        },
    )    

    # Publish FSM information
    YasminViewerPub("YASMIN_MONITOR_DEMO", sm)

    # Execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()