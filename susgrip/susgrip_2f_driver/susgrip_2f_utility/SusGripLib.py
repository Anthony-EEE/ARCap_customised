#!/usr/bin/env python3
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

class SusGrip2F:
    def __init__(self, id = 86, timeout = 0.1,*args, **kwargs):
        self.gripper_rtu = ModbusClient(
            method = 'RTU',
            port=kwargs['port'],
            baudrate=115200,
            timeout = timeout
        )
        self.is_connect = False
        connection = self.gripper_rtu.connect()
        if connection:
            self.is_connect = True
        self.id = id
        self.timeout = timeout
        self.data = []
   
    def s16(self,value):
        return -(value & 0x8000) | (value & 0x7fff)

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))
   
    # check connect
    def connect(self):
        return self.gripper_rtu.connect()

    def set_rtu_mode(self):
        self.gripper_rtu.write_registers(2000, [3], unit=self.id)

    # pos = [0,130] mm, vel = [1--100]%, torque = [1--100]%
    def rtu_set_pos_pvt(self, pos, vel, tor):
        pos = self.constrain(pos, 0, 130)
        vel = self.constrain(vel, 1, 100)
        tor = self.constrain(tor, 1, 100)
        self.gripper_rtu.write_registers(2001, [int(2*pos),int(vel*2.55), int(tor*2.55)], unit=self.id)
    # pos = [0,130] mm
    def rtu_write_pos(self, pos):
        pos = self.constrain(pos, 0, 130)
        self.gripper_rtu.write_registers(2001, [int(2*pos)], unit=self.id)
    # pos = [0,130] mm
    def rtu_write_pos2(self, pos):
        pos = self.constrain(pos, 0, 260)
        self.gripper_rtu.write_registers(2001, [int(pos)], unit=self.id) 
        
# The susgrip set parameters
# The susgrip set parameters
    def rtu_set_pos(self, pos):
        pos = self.constrain(pos, 0, 130)
        self.gripper_rtu.write_registers(2001, [int(pos*2)], unit=self.id)
        
    def rtu_set_vel(self, vel):
        vel = self.constrain(vel, 1, 100)
        self.gripper_rtu.write_registers(2002, [int(vel*2.55)], unit=self.id)
        
    def rtu_set_tor(self, tor):
        tor = self.constrain(tor, 1, 100)
        self.gripper_rtu.write_registers(2003, [int(tor*2.55)], unit=self.id)
  
    def set_gpio_mode(self):
        self.gripper_rtu.write_registers(2000, [7], unit=self.id)

    def gpio_set_pvt(self, high, low, vel, tor):
        high = self.constrain(high, 0, 130)
        low = self.constrain(low, 0, 130)
        vel = self.constrain(vel, 1, 100)
        tor = self.constrain(tor, 1, 100)
        self.gripper_rtu.write_registers(2002, [int(high*2), int(low*2), int(vel*2.55), int(tor*2.55)], unit=self.id)

# The susgrip read data
    def reload_data(self):
        if self.gripper_rtu.is_socket_open():
            registers  = self.gripper_rtu.read_input_registers(1000, 6, unit= self.id, timeout = self.timeout)
            if registers != None and registers.isError() == False:
                return registers.registers
        return "None"
    
    def read_data_json(self):
        self.data = self.reload_data()
        if self.data != None:
            gripper_status = self.extrac_status(self.data)
            gripper_pos = self.s16(self.data[2])/2
            gripper_cur = self.s16(self.data[3])/10
            gripper_temp_a =  (self.data[4]>>8)
            gripper_temp_b =  self.data[4]&0xFF
            result = {
                "status": gripper_status,
                "pos": gripper_pos,
                "cur": gripper_cur,
                "temp_a": gripper_temp_a,
                "temp_b": gripper_temp_b,
            }
            return result
        return "None" 
    
    def extrac_status(self, status):
        data = {
            "ACTIVE" : None,
            "GOTO" : None,
            "MODE" : None,
            "OBJ" : None,
            "INPUT" : None,
            "FAULT" : None
        }
        if (status>>0)&0x01:
            data["ACTIVE"] = 1
        else:
            data["ACTIVE"] = 0

        if (status>>1)&0x01:
            data["GOTO"] = 1
        else:
            data["GOTO"] = 0

        if ((status>>2)&0x03):
            data["MODE"] = "GPIO"
        else:
            data["MODE"] = "RTU"

        if (status>>4)&0x03 == 0x00:
            data["OBJ"] = "NONE"
        elif (status>>4)&0x03 == 0x01:
            data["OBJ"] = "OPEN"
        elif (status>>4)&0x03 == 0x02:
            data["OBJ"] = "CLOSE"
        elif (status>>4)&0x03 == 0x03:
            data["OBJ"] = "DROP"

        if (status>>6)&0x01 == 0x00:
            data["INPUT"] = "OFF"
        elif (status>>6)&0x01 == 0x01:
            data["INPUT"] = "ON"

        if (status>>7)&0x01 == 0x00:
            data["MOVE"] = "OFF"
        elif (status>>7)&0x01 == 0x01:
            data["MOVE"] = "ON"

        if (status>>8) == 0x00:
            data["FAULT"] = "NONE"
        elif (status>>8) == 0x01:
            data["FAULT"] = "nOCWT"
        elif (status>>8) == 0x02:
            data["FAULT"] = "nFAULT"
        elif (status>>8) == 0x04:
            data["FAULT"] = "CURRENT OVER"
        elif (status>>8) == 0x03:
            data["FAULT"] = "TEMP OVER"
        elif (status>>8) == 0x05:
            data["FAULT"] = "DIS OVER"
        return data
    
    def read_status(self):
        if self.gripper_rtu.is_socket_open():
            registers  = self.gripper_rtu.read_input_registers(1000, 1, unit= self.id, timeout = self.timeout)
            if registers != None and registers.isError() == False:
                status = registers.registers[0]
                return self.extrac_status(status)
        return "None"   
    
    def check_movedone(self):
        if self.gripper_rtu.is_socket_open():
            registers  = self.gripper_rtu.read_input_registers(1000, 1, unit= self.id, timeout = self.timeout)
            if registers != None and registers.isError() == False:
                status = registers.registers[0]
                if (status>>7)&0x01:
                    return 1
                else:
                    return 0
        return 0      
    
    def check_object(self):
        if self.gripper_rtu.is_socket_open():
            registers  = self.gripper_rtu.read_input_registers(1000, 1, unit= self.id, timeout = self.timeout)
            if registers != None and registers.isError() == False:
                status = registers.registers[0]
                if (status>>4)&0x03 == 0x00:
                    return "NONE"
                elif (status>>4)&0x03 == 0x01:
                    return "OPEN"
                elif (status>>4)&0x03 == 0x02:
                    return "CLOSE"
                elif (status>>4)&0x03 == 0x03:
                    return "DROP"
        return "NONE"     
        
    def deactive(self):
        self.gripper_rtu.write_registers(2000, [0], unit=self.id)
      
    def active(self):
        self.gripper_rtu.write_registers(2000, [1], unit=self.id)

    def disconnect(self):
        self.gripper_rtu.close()      

    