from geometry_msgs.msg import Twist
from .action_base import ActionBase


class SearchTargetAction(ActionBase):
    """
    Rotate AUV to search for a specific target using perception input.
    """

    def __init__(
        self,
        target_type: str,
        yaw_rate: float = 0.3,
        min_confidence: float = 0.5,
        stable_count: int = 3,
        timeout_s: float = 35.0,
    ):
        super().__init__(name=f"SearchTarget({target_type})", timeout_s=timeout_s)

        self.target_type = target_type
        self.yaw_rate = yaw_rate
        self.min_confidence = min_confidence
        self.required_stable_count = stable_count

        self._stable_counter = 0

    def on_enter(self, context):
        super().on_enter(context)
        self._stable_counter = 0
        context.get_logger().info(
            f"[SearchTarget] Searching for target: {self.target_type}"
        )
        context.msg = f"Searching for target: {self.target_type}"

    def update(self, context):
        perception = context.latest_perception

        seen = False

        if perception is not None:
            for obj in perception.objects:
                if obj.class_name == str(self.target_type):
                    # if obj.confidence >= self.min_confidence:
                    seen = True
                    break

        if seen:
            self._stable_counter += 1
        else:
            self._stable_counter = 0

        cmd = Twist()

        if self._stable_counter < self.required_stable_count:
            cmd.angular.z = self.yaw_rate
        else:
            cmd.angular.z = 0.0

        now = context.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        context.progress = elapsed / self.timeout_s
        context.progress = min(max(context.progress, 0.0), 1.0)

        return cmd, None, None

    def is_success(self, context) -> bool:
        return self._stable_counter >= self.required_stable_count
