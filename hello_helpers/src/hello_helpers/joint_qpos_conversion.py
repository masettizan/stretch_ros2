#!/usr/bin/env python3


def get_Idx(tool_name):
    if tool_name == 'eoa_wrist_dw3_tool_sg3':
        return SE3_dw3_sg3_Idx
    else:
        raise ValueError('Undefined tool name in QposConversion.')

class SE3_dw3_sg3_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 7
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    HEAD_PAN = 5
    HEAD_TILT = 6
    BASE_TRANSLATE = 7
    BASE_ROTATE = 8

    num_joints = 9