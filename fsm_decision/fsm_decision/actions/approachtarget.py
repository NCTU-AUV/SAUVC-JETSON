from geometry_msgs.msg import Twist
from .action_base import ActionBase


class ApproachTargetAction(ActionBase):
    """
    Approach a target while maintaining horizontal alignment using lateral motion.
    """

    def __init__(
        self,
        target_type: str,

        # Forward control
        approach_speed: float = 0.3,        # m/s
        stop_distance: float = 2.5,          # meters

        # Lateral alignment control
        image_width: int = 640,              # pixels
        kp_y: float = 0.002,                 # lateral P gain (pixel-based)
        max_lateral_speed: float = 0.3,      # m/s

        # Stability & safety
        stable_count: int = 5,
        lost_count_threshold: int = 25,
        timeout_s: float = 40.0,
    ):
        super().__init__(
            name=f"ApproachTarget({target_type})",
            timeout_s=timeout_s
        )

        self.target_type = target_type

        # Forward control
        self.approach_speed = approach_speed
        self.stop_distance = stop_distance

        # Lateral alignment
        self.image_width = image_width
        self.image_center_x = image_width / 2.0
        self.kp_y = kp_y
        self.max_lateral_speed = max_lateral_speed

        # Stability & safety
        self.required_stable_count = stable_count
        self.lost_count_threshold = lost_count_threshold

        # Internal state
        self._stable_counter = 0
        self._lost_counter = 0

    def on_enter(self, context):
        super().on_enter(context)
        self._stable_counter = 0
        self._lost_counter = 0

        context.get_logger().info(
            f"[ApproachTarget] Approaching target: {self.target_type}"
        )
        context.msg = f"Approaching target: {self.target_type}"

    def update(self, context):
        perception = context.latest_perception
        cmd = Twist()

        target_obj = None

        # -----------------------
        # Find target
        # -----------------------
        if perception is not None:
            for obj in perception.objects:
                if (
                    obj.class_name == str(self.target_type)
                    and obj.valid
                ):
                    target_obj = obj
                    break

        # -----------------------
        # Target not found
        # -----------------------
        if target_obj is None:
            self._lost_counter += 1
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        else:
            self._lost_counter = 0

            # -----------------------
            # Lateral alignment (pixel-based)
            # -----------------------
            error_x = target_obj.cx - self.image_center_x
            lateral_cmd = -self.kp_y * error_x
            lateral_cmd = max(
                -self.max_lateral_speed,
                min(self.max_lateral_speed, lateral_cmd),
            )
            cmd.linear.y = lateral_cmd

            # -----------------------
            # Forward approach
            # -----------------------
            if target_obj.distance > self.stop_distance:
                cmd.linear.x = self.approach_speed
                self._stable_counter = 0
            else:
                cmd.linear.x = 0.0
                self._stable_counter += 1

        # -----------------------
        # Progress (distance-based if possible)
        # -----------------------
        if target_obj is not None:
            # Progress goes from 0 → 1 as distance decreases
            progress = 1.0 - min(
                target_obj.distance / max(self.stop_distance, 1e-3),
                1.0
            )
            context.progress = max(0.0, min(1.0, progress))
        else:
            # Fallback to time-based progress
            now = context.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            context.progress = min(elapsed / self.timeout_s, 1.0)

        return cmd, None, None

    def is_success(self, context) -> bool:
        return self._stable_counter >= self.required_stable_count

    def is_failed(self, context) -> bool:
        # Target lost for too long
        if self._lost_counter >= self.lost_count_threshold:
            context.get_logger().warn(
                f"[ApproachTarget] Target lost for "
                f"{self._lost_counter} ticks"
            )
            return True

        return super().is_failed(context)
