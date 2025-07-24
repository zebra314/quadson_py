from typing import List
from real.motor_manager import MotorManager
from common.leg_kinematics import LegKinematics

class LegGroup:
  def __init__(self, leg_index: int, motor_manager: MotorManager):
    self.leg_index = leg_index
    self.motor_manager = motor_manager
    self.leg_kinematics = LegKinematics()

  def set_motor_angles(self, motor_angles: List[float]) -> None:
    self.leg_kinematics.set_motor_angles(motor_angles)

  def set_ee_point(self, ee_point: List[float]) -> None:
    self.leg_kinematics.set_ee_point(ee_point)


  def get_motor_angles(self) -> List[float]:
    pass

  def get_ee_point(self) -> List[float]:
    pass



