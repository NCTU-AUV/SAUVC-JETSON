class ActionBase:
    """
    Base class for all FSM actions.
    """

    def __init__(self, name: str, timeout_s: float = None):
        self.name = name
        self.timeout_s = timeout_s
        self.start_time = None

    def on_enter(self, context):
        """
        Called once when FSM enters this action.
        """
        self.start_time = context.get_clock().now()

    def update(self, context):
        """
        Called every FSM tick.

        Return:
            cmd_vel (geometry_msgs/Twist or None)
            arm_cmd (std_msgs/Int8 or None)
            hand_cmd (std_msgs/Int8 or None)
        """
        return None, None, None

    def is_success(self, context) -> bool:
        """
        Return True if action completed successfully.
        """
        return False

    def is_failed(self, context) -> bool:
        """
        Return True if action failed.
        """
        # Timeout check (generic)
        if self.timeout_s is not None and self.start_time is not None:
            now = context.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            if elapsed > self.timeout_s:
                return True

        # Kill switch handled by FSM
        return False
