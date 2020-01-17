#!/usr/bin/env python
from enum import Enum

#class enumeration representing the state of the robot as it completes its tasks
class RobotState(Enum):
    INITIALIZING = 0
    INITIALIZED = 1
    MOVE_BASE_TO_BLOCK = 2
    MOVE_ARM_OVER_BLOCK = 3
    QUERYING_CAMERA = 4
    MOVING_ARM_TO_BLOCK = 5
    GRIPPING_BLOCK = 6
    MOVING_BLOCK_TO_STANDBY = 7
    MOVING_BASE_TO_DESTINATION = 8
    MOVING_BLOCK_TO_DESTINATION = 9
    RELEASING_BLOCK = 10
    FIN = 11

#class enumeration representing the state of the robot arm as it completes its tasks
class ArmState(Enum):
    MOVING = 0
    STATIONARY = 1
    GRIPPING = 2
    RELEASING = 3
    
#class enumeration representing the state of the robot base as it completes its tasks
class BaseState(Enum):
    MOVING = 0
    STATIONARY = 1
