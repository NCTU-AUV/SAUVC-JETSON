import yaml
from pathlib import Path

from fsm_decision.actions.hover import HoverAction
from fsm_decision.actions.searchtarget import SearchTargetAction

# 之後會加：
# from fsm_decision.actions.move_forward import MoveForwardAction
# from fsm_decision.actions.rotate_yaw import RotateYawAction


# =========================
# Action registry
# =========================
ACTION_REGISTRY = {
    'Hover': HoverAction,
    # 'MoveForward': MoveForwardAction,
    # 'RotateYaw': RotateYawAction,
    'SearchTarget': SearchTargetAction,
}


class MissionLoaderError(Exception):
    pass


class MissionLoader:
    """
    Load mission YAML and convert to list of Action instances.
    """

    def __init__(self, mission_dir: str):
        self.mission_dir = Path(mission_dir)

        if not self.mission_dir.exists():
            raise MissionLoaderError(
                f'Mission directory not found: {self.mission_dir}'
            )

    def load(self, mission_file: str):
        """
        Args:
            mission_file: filename, e.g. 'test_hover.yaml'

        Returns:
            List[ActionBase]
        """
        path = self.mission_dir / mission_file
        if not path.exists():
            raise MissionLoaderError(f'Mission file not found: {path}')

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        return self._parse_mission(data)

    # -------------------------
    # Internal helpers
    # -------------------------
    def _parse_mission(self, data: dict):
        if 'actions' not in data:
            raise MissionLoaderError('Mission YAML missing "actions"')

        actions = []

        for idx, action_cfg in enumerate(data['actions']):
            if 'type' not in action_cfg:
                raise MissionLoaderError(
                    f'Action #{idx} missing "type"'
                )

            action_type = action_cfg['type']

            if action_type not in ACTION_REGISTRY:
                raise MissionLoaderError(
                    f'Unknown action type: {action_type}'
                )

            action_cls = ACTION_REGISTRY[action_type]

            # Remove 'type', remaining keys are constructor args
            params = dict(action_cfg)
            params.pop('type')

            try:
                action = action_cls(**params)
            except TypeError as e:
                raise MissionLoaderError(
                    f'Failed to create action {action_type}: {e}'
                )

            actions.append(action)

        return actions
