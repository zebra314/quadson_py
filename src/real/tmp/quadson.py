import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import threading
from cando import *
import time
import sys
import struct
from src.real.can_config import *

class Quadson:
  def __init__(self):
    self.device_handle = None
    self.receive_frame = Frame()

    # Init Leg group

    self.connect_can_device()
    self.thread_pause_event = threading.Event()
    self.thread_stop_event = threading.Event()
    self.reading_thread = threading.Thread(target = self.handle_can_read)

    self.thread_stop_event.set()
    self.reading_thread.start()

    self.set_auto_receive(True)

  def connect_can_device(self):
    self.dev_lists = list_scan()
    self.device_handle = self.dev_lists[0]

    if len(self.dev_lists) == 0:
      raise Exception("No can device found")

    if os.name == 'posix':  # for linux os
      if self.device_handle.is_kernel_driver_active(0):
        reattach = True
        self.device_handle.detach_kernel_driver(0)

    # set baudrate: 500K, sample point: 87.5%
    dev_set_timing(self.device_handle, 1, 12, 6, 1, 6)

    dev_start(self.device_handle, CANDO_MODE_NORMAL|CANDO_MODE_NO_ECHO_BACK)

  def disconnect_can_device(self) -> None:
    self.thread_stop_event.clear()
    dev_stop(self.device_handle)

  def handle_can_read(self):
    while self.thread_stop_event.isSet():
      self.thread_pause_event.wait()
      if dev_frame_read(self.device_handle, self.receive_frame, 1):
        if (not self.msg_error(self.receive_frame)):
          self.dispatch_to_group(self.receive_frame)
        else:
          print("[ERROR] Can reading error.")
          self.disconnect_can_device()
          
  def handle_can_frame(self, frame):
    motor_id = self.extract_motor_id(frame)
    group = self.motor_id_to_group.get(motor_id, None)
    if group:
        group.handle_scan_response(motor_id, frame)
    else:
        print(f"[WARN] Received frame from unknown motor ID {motor_id}")

  def set_auto_receive(self, enable: bool):
    if enable:
      self.thread_pause_event.set()
    else:
      self.thread_pause_event.clear()

  def scan_motors(self):
    print("Scaning...")

    for i in range(1,13):
      send_frame = Frame()
      send_frame.can_id = 0x00 | (i<<6)
      send_frame.can_id |= CANDO_ID_RTR
      # send_frame.can_dlc = 8
      # send_frame.data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
      dev_frame_send(self.device_handle, send_frame)
      # print("scannnig "+str(i))
      time.sleep(0.001)

      if dev_frame_read(self.device_handle, self.receive_frame, 1):
        if (not self.msg_error(self.receive_frame)):
          can_pack = self.decode_msg(self.receive_frame)
          if i == can_pack.motor_id:
            self.motor_list[i-1].exist = True
            print("Receive from motor " + str(i))
          else:
            print("Scanning error")
            self.disconnect_can_device()
            sys.exit(0)

      else:
        self.motor_list[i-1].exist = False
        print("Motor " + str(i) + " no response")

    for x in range(10):
      dev_frame_read(self.device_handle, self.receive_frame, 1)


  def dispatch_to_group(self, receive_frame: Frame):
    # print(" is_extend : " + ("True" if rec_frame.can_id & CANDO_ID_EXTENDED else "False"))
    # print(" is_rtr : " + ("True" if rec_frame.can_id & CANDO_ID_RTR  else "False"))
    # print(" can_id : " + str(rec_frame.can_id & CANDO_ID_MASK))

    can_id = receive_frame.can_id & CANDO_ID_MASK
    can_dlc = receive_frame.can_dlc
    group_mag = False
    if (receive_frame.can_id & CANDO_ID_EXTENDED):
      motor_id = can_id>>24
      id_type = CAN_ID_TYPE.EXTENDED
      msg_id = can_id & 0xFFFFF
      value = struct.unpack("<h",bytes(receive_frame.data[0:2]))[0]
      # print("from motor " + str(motor_id) + " get msg: " + str(CAN_EXT_TYPE(msg_id)) + " value: " + str(value))

    else:
      motor_id = can_id>>6
      if (motor_id>>10 > 0):
        group_mag = True
      id_type = CAN_ID_TYPE.STANDARD
      msg_id = can_id & 0x1F
      value = struct.unpack("<h",bytes(receive_frame.data[0:2]))[0]
      # print("from motor " + str(motor_id) + " get msg: " + str(CAN_STD_TYPE(msg_id)) + " value: " + str(value))

    if (group_mag):
      # TODO : add group msg handle
      pass

    self.motor_list[motor_id-1].update_param(id_type, msg_id, value)
    
    return self.CanMessage(motor_id, id_type, msg_id, value)

  def msg_error(self, rec_frame):
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
