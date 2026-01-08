from geometry_msgs.msg import Twist
from .action_base import ActionBase


class HoverAction(ActionBase):
    """
    Hover in place for a given duration.
    """

    def __init__(self, duration_s: float):
        super().__init__(name='Hover', timeout_s=duration_s + 1.0)
        self.duration_s = duration_s

    def on_enter(self, context):
        super().on_enter(context)
        context.get_logger().info(
            f'[HoverAction] Start hovering for {self.duration_s:.1f} s'
        )
        context.msg = f'Hovering for {self.duration_s:.1f} s'

    def update(self, context):
        """
        Publish zero cmd_vel to keep watchdog alive.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        now = context.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        context.progress = elapsed / self.duration_s
        context.progress = min(max(context.progress, 0.0), 1.0)

        return cmd, None, None

    def is_success(self, context) -> bool:
        now = context.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        return elapsed >= self.duration_s
