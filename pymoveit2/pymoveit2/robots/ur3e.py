from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"              #"ur_manipulator"


def joint_names(prefix: str = "") -> List[str]:
    return [
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        "shoulder_pan_joint",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "tool0"
