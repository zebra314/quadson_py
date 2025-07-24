import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from cando import *
import time
import struct
import numpy as np
from typing import List, Any
from src.real.can_config import *
from src.leg_kinematics import LegKinematics
from src.real.motor import Motor

ID_STD_OFFSET = 6
ID_EXT_OFFSET = 24

class CanPacket:
  def __init__(self, motor_id, id_type, msg_id, data):
    self.motor_id = motor_id
    self.id_type = id_type
    self.msg_id = msg_id 
    self.data = data

class LegGroup:
  def __init__(self, motor_ids: List, dev_handle: Any):
    self.motor_ids = motor_ids
    
    self.motor_dict = {}
    for id in motor_ids:
      self.motor_dict[id] = Motor(id, exist=False)

    self.dev_handle = dev_handle
    self.receive_frame = Frame()

    self.leg_kinematics = LegKinematics()
    self.leg_kinematics.set_motor_angles([0, np.pi, np.pi/2]) # initial motor angles

  def scan_motors(self):
    print(f"Leg group {self.motor_ids} scaning...")

    for motor_id, motor in self.motor_dict.items():
      motor.send_rtr()
      time.sleep(0.001)

      if self.msg_error(self.receive_frame):
        motor.exist = False
        print("[WARN] Motor " + str(motor_id) + " no response.")
      else:
        can_pack = self._decode_frame(self.receive_frame)
        if motor_id == can_pack.motor_id:
          motor.exist = True
          print("Receive from motor " + str(motor_id))
        else:
          raise Exception("Scanning error")
        
  def _decode_frame(self, frame: Frame) -> CanPacket:
    can_id = frame.can_id & CANDO_ID_MASK
    can_dlc = frame.can_dlc
    group_mag = False

    if (frame.can_id & CANDO_ID_EXTENDED):
      motor_id = can_id>>24
      id_type = CAN_ID_TYPE.EXTENDED
      msg_id = can_id & 0xFFFFF
      value = struct.unpack("<h",bytes(frame.data[0:2]))[0]
    else:
      motor_id = can_id>>6
      if (motor_id>>10 > 0):
        group_mag = True
      id_type = CAN_ID_TYPE.STANDARD
      msg_id = can_id & 0x1F
      value = struct.unpack("<h",bytes(frame.data[0:2]))[0]

    if (group_mag):
      # TODO : add group msg handle
      pass

    return self.CanPacket(motor_id, id_type, msg_id, value)

  def handle_frame(self, frame: Frame) -> None:
    self.receive_frame = frame
    can_packet = self._decode_frame(frame)
    motor = self.motor_dict.get(can_packet.motor_id, None)
    if motor:
      motor.update_param(can_packet.id_type, can_packet.msg_id, can_packet.value)
    else:
      print(f"[WARN] Unknown motor id {can_packet.motor_id} in group.")

  def msg_error(self, rec_frame: Frame) -> bool:
    if rec_frame.can_id & CANDO_ID_ERR:
      error_code, err_tx, err_rx = parse_err_frame(rec_frame)
      print("Error: ")
      print(error_code)
      if error_code & CAN_ERR_BUSOFF:
        print(" CAN_ERR_BUSOFF")
      if error_code & CAN_ERR_RX_TX_WARNING:
        print(" CAN_ERR_RX_TX_WARNING")
      if error_code & CAN_ERR_RX_TX_PASSIVE:
        print(" CAN_ERR_RX_TX_PASSIVE")
      if error_code & CAN_ERR_OVERLOAD:
        print(" CAN_ERR_OVERLOAD")
      if error_code & CAN_ERR_STUFF:
        print(" CAN_ERR_STUFF")
      if error_code & CAN_ERR_FORM:
        print(" CAN_ERR_FORM")
      if error_code & CAN_ERR_ACK:
        print(" CAN_ERR_ACK")
      if error_code & CAN_ERR_BIT_RECESSIVE:
        print(" CAN_ERR_BIT_RECESSIVE")
      if error_code & CAN_ERR_BIT_DOMINANT:
        print(" CAN_ERR_BIT_DOMINANT")
      if error_code & CAN_ERR_CRC:
        print(" CAN_ERR_CRC")
        print(" err_tx: " + str(err_tx))
        print(" err_rx: " + str(err_rx))
      return True
    else:
      return False

  def set_motor_angles(self, motor_angles: List) -> None:
    self.leg_kinematics.set_motor_angles(motor_angles)
    theory_angles = self.leg_kinematics.get_angles()
    env_angles = self.get_env_angles(theory_angles)
    for motor_id, angle in zip(self.motor_dict.keys(), env_angles):
        self.motor_dict[motor_id].set_angle(angle)
