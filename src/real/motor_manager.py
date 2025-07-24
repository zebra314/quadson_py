import struct
import threading
import time
import cando
import math
import sys
import os
from real.can_config import *
from real.can_message import CanMessage
from real.motor import Motor

ID_STD_OFFSET = 6
ID_EXT_OFFSET = 24

class MotorManager:
	def __init__(self):
		self.dev_lists = None
		self.motor_list = [None] * 12

		# Create 12 motor objects
		for x in range(12):
			self.motor_list[x] = Motor(x, False)
		self.rec_frame = cando.Frame()

		self.connect_can_device()
		self.thread_pause_event = threading.Event()
		self.thread_stop_event = threading.Event()
		self.reading_thread = threading.Thread(target = self._can_read_handle)

		self.thread_stop_event.set()
		self.reading_thread.start()

		self._scan_motors()
		self._set_auto_receive(True)
	
	def connect_can_device(self):
		self.dev_lists = cando.list_scan()

		if len(self.dev_lists) == 0:
			print("Device not found!")
			sys.exit(0)

		if os.name == 'posix':  # for linux os
			if self.dev_lists[0].is_kernel_driver_active(0):
				self.dev_lists[0].detach_kernel_driver(0)

		# set baudrate: 500K, sample point: 87.5%
		cando.dev_set_timing(self.dev_lists[0], 1, 12, 6, 1, 6)

		cando.dev_start(self.dev_lists[0], cando.CANDO_MODE_NORMAL | cando.CANDO_MODE_NO_ECHO_BACK)

	def disconnect_can_device(self):
		self.thread_stop_event.clear()
		cando.dev_stop(self.dev_lists[0])
	
	def send_motor_cmd(self, motor_index, cmd, value):
		if self.motor_list[motor_index-1].exist == False:
			print("Motor unconnected")
			return

		send_frame = cando.Frame()
		
		if (cmd.__class__) == CAN_STD_TYPE:
			send_frame.can_id = 0x00 | (motor_index << ID_STD_OFFSET) | cmd.value
		else:
			send_frame.can_id = 0x00 | (motor_index << ID_EXT_OFFSET) | cmd.value
			send_frame.can_id |= cando.CANDO_ID_EXTENDED

		send_frame.can_dlc = 2
		send_frame.data = [value >> 8, value & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		cando.dev_frame_send(self.dev_lists[0], send_frame)
		# print("sending message id:" + str(send_frame.can_id))
		
	def get_angle(self, motor_id):
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
		
		revolution = self.motor_list[motor_id-1].get_param(CMD_TYPE.PRESENT_REVOLUTION)
		angle_16 = self.motor_list[motor_id-1].get_param(CMD_TYPE.PRESENT_POSITION_DEG)
		# print("rev: " + str(revolution) + "  angle: " + str(angle_16))

		angle = (revolution + ((angle_16) / 65536))/71.96 * 2*math.pi
		# print("degree: " + str(math.degrees(angle)))
		
		return angle

	def set_angle(self, motor_id, angle):
		can_signal = angle * 32768 / 180
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG, can_signal)

	def get_omega(self, motor_id):
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS, 0)
		velocity_01 = self.motor_list[motor_id-1].get_param(CMD_TYPE.PRESENT_VELOCITY_DPS)

		omega = (((velocity_01*10) / 65536))/71.96 * 2 * math.pi
		# print("omega: " + str(omega))
		return omega
	
	def set_omega(self, motor_id, omega):
		can_signal = omega * 32768 / 180
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS, can_signal)

	def _can_read_handle(self):
		while self.thread_stop_event.isSet():
			self.thread_pause_event.wait()
			if cando.dev_frame_read(self.dev_lists[0], self.rec_frame, 1):
				if (not self._msg_error(self.rec_frame)):
					self._decode_msg(self.rec_frame)
				else:
					print("reading error")
					self.disconnect_can_device()
	
	def _scan_motors(self):
		print("Scaning...")

		for i in range(1, 13):
			# Prepare frame
			send_frame = cando.Frame()
			send_frame.can_id = 0x00 | (i << 6)
			send_frame.can_id |= cando.CANDO_ID_RTR
			# send_frame.can_dlc = 8
			# send_frame.data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
			cando.dev_frame_send(self.dev_lists[0], send_frame)
			time.sleep(0.001)

			if cando.dev_frame_read(self.dev_lists[0], self.rec_frame, 1):
				if (not self._msg_error(self.rec_frame)):
					can_pack = self._decode_msg(self.rec_frame)
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
			cando.dev_frame_read(self.dev_lists[0], self.rec_frame, 1)

	def _set_auto_receive(self, enable):
		if enable:
			self.thread_pause_event.set()
		else:
			self.thread_pause_event.clear()

	def _msg_error(self, rec_frame):
		if rec_frame.can_id & cando.CANDO_ID_ERR:
			error_code, err_tx, err_rx = cando.parse_err_frame(rec_frame)
			print("Error: ")
			print(error_code)

			if error_code & cando.CAN_ERR_BUSOFF:
				print(" CAN_ERR_BUSOFF")
			if error_code & cando.CAN_ERR_RX_TX_WARNING:
				print(" CAN_ERR_RX_TX_WARNING")
			if error_code & cando.CAN_ERR_RX_TX_PASSIVE:
				print(" CAN_ERR_RX_TX_PASSIVE")
			if error_code & cando.CAN_ERR_OVERLOAD:
				print(" CAN_ERR_OVERLOAD")
			if error_code & cando.CAN_ERR_STUFF:
				print(" CAN_ERR_STUFF")
			if error_code & cando.CAN_ERR_FORM:
				print(" CAN_ERR_FORM")
			if error_code & cando.CAN_ERR_ACK:
				print(" CAN_ERR_ACK")
			if error_code & cando.CAN_ERR_BIT_RECESSIVE:
				print(" CAN_ERR_BIT_RECESSIVE")
			if error_code & cando.CAN_ERR_BIT_DOMINANT:
				print(" CAN_ERR_BIT_DOMINANT")
			if error_code & cando.CAN_ERR_CRC:
				print(" CAN_ERR_CRC")
				print(" err_tx: " + str(err_tx))
				print(" err_rx: " + str(err_rx))
			return True
		else:
			return False

	def _decode_msg(self, rec_frame):
		# print(" is_extend : " + ("True" if self.rec_frame.can_id & CANDO_ID_EXTENDED else "False"))
		# print(" is_rtr : " + ("True" if self.rec_frame.can_id & CANDO_ID_RTR	else "False"))
		# print(" can_id : " + str(self.rec_frame.can_id & CANDO_ID_MASK))
		
		can_id = rec_frame.can_id & cando.CANDO_ID_MASK
		can_dlc = rec_frame.can_dlc
		group_mag = False

		if (rec_frame.can_id & cando.CANDO_ID_EXTENDED):
			motor_id = can_id >> 24
			id_type = CAN_ID_TYPE.EXTENDED
			msg_id = can_id & 0xFFFFF
			value = struct.unpack("<h",bytes(self.rec_frame.data[0:2]))[0]
			# print("from motor " + str(motor_id) + " get msg: " + str(CAN_EXT_TYPE(msg_id)) + " value: " + str(value))
		else:
			motor_id = can_id >> 6
			if (motor_id>>10 > 0):
				group_mag = True
			id_type = CAN_ID_TYPE.STANDARD
			msg_id = can_id & 0x1F
			value = struct.unpack("<h",bytes(self.rec_frame.data[0:2]))[0]
			# print("from motor " + str(motor_id) + " get msg: " + str(CAN_STD_TYPE(msg_id)) + " value: " + str(value))

		if (group_mag):
			pass

		self.motor_list[motor_id-1].update_param(id_type, msg_id, value)
		
		return CanMessage(motor_id, id_type, msg_id, value)