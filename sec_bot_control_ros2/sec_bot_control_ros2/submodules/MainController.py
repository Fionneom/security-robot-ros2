#!/usr/bin/env python3
import serial


class Main_Controller:
    def __init__(self,port):
        self.port = port
        self.ser = serial.Serial(
        port = self.port, 
        baudrate=115200, 
        bytesize=8, 
        timeout=1,
        stopbits=serial.STOPBITS_ONE
        )
        self.SpeedPico = 0x0000_005F
        self.SteeringPico = 0x0000_0060

    def Read(self,DeviceAddress,address):
        if address >= 127:#Max 7 bit number
            print("Max address number is 127")
            return 

        address = address | 0x0000_0080#MSB must be 1
        data = 0x0000_0000 #Clears the data that could be there
        self.ser.write(DeviceAddress.to_bytes(1,'big'))#Writes address 
        self.ser.write(address.to_bytes(1,'big'))#Writes address 
        self.ser.write(data.to_bytes(4,'big', signed=True))#Writes empty data so that the C-Script works
        line = self.ser.read(4)#reads line
        line = int.from_bytes(line,'big')#Converts from bytes to int to make it readable

        #Lines below converts negative numbers into readable numbers: Twos Comp --> Dec
        readBack = line
        NB = (line & 0x8000_0000) >> 31#Saves last bit
        if NB == 1:
            temp = readBack ^ 0xffff_ffff#Flips every bit
            readBack = (temp + 1) * -1#Reverse of Twos Comp
        elif NB ==0:
            readBack = line

        return readBack
    
    def Write(self,DeviceAddress,address,data:int):
        if address >= 127:
            print("Max address number is 127")
            return 
        address = address & 0x0000_007f#MSB must be 0
        self.ser.write(DeviceAddress.to_bytes(1,'big'))
        self.ser.write(address.to_bytes(1,'big'))
        self.ser.write(data.to_bytes(4,'big', signed=True))

#-----------------------Start of Read Functions---------------------------------

#-------------Specific Speed Controller READ Functions---------------
    def Read_SetSpeed(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0000#Represents address 0
        Speed = Main_Controller.Read(self,DeviceAddress,address)
        return Speed
    
    def Read_SetPeriod(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0001#Represents address 1
        Period = Main_Controller.Read(self,DeviceAddress,address)
        return Period
    
    def Read_KP(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0002#Represents address 2
        KP = Main_Controller.Int_2_Float(self,Main_Controller.Read(self,DeviceAddress,address))
        return KP

    def Read_KI(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0003#Represents address 3
        KI = Main_Controller.Int_2_Float(self,Main_Controller.Read(self,DeviceAddress,address))
        return KI

    def Read_KD(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0004#Represents address 4
        KD = Main_Controller.Int_2_Float(self,Main_Controller.Read(self,DeviceAddress,address))
        return KD
    
    def Read_Current_Speed(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0005#Represents address 5
        Current_Speed = Main_Controller.Int_2_Float(self,Main_Controller.Read(self,DeviceAddress,address))
        Current_Speed=round(Current_Speed,2)
        return Current_Speed
    
    def Read_Integral_Error(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0006#Represents address  6
        IntegralError = Main_Controller.Read(self,DeviceAddress,address)
        return IntegralError
    
    def Read_Derivative_Error(self):
        DeviceAddress = self.SpeedPico 
        address = 0x0000_0007#Represents address 7
        DerivativeError = Main_Controller.Read(self,DeviceAddress,address)
        return DerivativeError
    
    def Read_PID_Output(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0008#Represents address 8
        PID = Main_Controller.Read(self,DeviceAddress,address)
        return PID
    
    def Read_Current_Count_Speed(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0009#Represents address 9
        Current_Count = Main_Controller.Read(self,DeviceAddress,address)
        return Current_Count
    
    def Read_PWM_Duty(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_000a#Represents address 10
        Duty = Main_Controller.Read(self,DeviceAddress,address)
        return Duty

    def Read_PID_Error(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_000b#Represents address 11
        Error = Main_Controller.Read(self,DeviceAddress,address)
        return Error

    def Read_Device_Address(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_000c#Represents address 12
        DeviceAdd = Main_Controller.Read(self,DeviceAddress,address)
        return DeviceAdd

    
#-------------Specific Steering Controller READ Functions---------------
    def Read_SetTarget(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0040#Represents address 64
        Target = Main_Controller.Read(self,DeviceAddress,address)
        return Target
    
    def Read_SetPeriod(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0041#Represents address 65
        Period = Main_Controller.Read(self,DeviceAddress,address)
        return Period
    
    def Read_Limit_State(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0042#Represents address 66
        LimitState = Main_Controller.Read(self,DeviceAddress,address)
        return LimitState

    def Read_Direction(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0043#Represents address 67
        direction = Main_Controller.Read(self,DeviceAddress,address)
        return direction
        
    def Read_Current_Count(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0044#Represents address 68
        Current_Count = Main_Controller.Read(self,DeviceAddress,address)
        return Current_Count
    
    def Read_SensorA(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0045#Represents address 69
        SensorA = Main_Controller.Read(self,DeviceAddress,address)
        return SensorA
    
    def Read_SensorB(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0046#Represents address 70
        SensorB = Main_Controller.Read(self,DeviceAddress,address)
        return SensorB

    def Read_Device_Address_Steer(self):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0047#Represents address 71
        DeviceAddStr = Main_Controller.Read(self,DeviceAddress,address)
        return DeviceAddStr
    

#-----------------------End of Read Functions---------------------------------

#--------------------------------------------------------------------------------------------------------

#-----------------------Start of Write Functions---------------------------------

#-------------Specific Speed Controller WRITE Functions---------------
    
    def Write_SetSpeed(self,data:int):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0000 
        Main_Controller.Write(self,DeviceAddress,address,data)
    
    def Write_SetPeriod(self,data:int):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0001
        Main_Controller.Write(self,DeviceAddress,address,data)
    
    def Write_KP(self, data:float):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0002
        val:int = Main_Controller.Float_2_Int(self,data)
        Main_Controller.Write(self,DeviceAddress,address,val)

    def Write_KI(self, data:float):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0003
        val:int = Main_Controller.Float_2_Int(self,data)
        Main_Controller.Write(self,DeviceAddress,address,val)

    def Write_KD(self, data:float):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0004
        val:int = Main_Controller.Float_2_Int(self,data)
        Main_Controller.Write(self,DeviceAddress,address,val)

    def EMERGENCY_STOP(self):
        DeviceAddress = self.SpeedPico
        address = 0x0000_0005
        data = 0x0000_0000
        Main_Controller.Write(self,DeviceAddress,address,data)

#-------------Specific Steering Controller WRITE Functions---------------
    
    def Write_SetTarget(self,data:int):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0040
        Main_Controller.Write(self,DeviceAddress,address,data)
    
    def Write_SetPeriod(self,data:int):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0041
        Main_Controller.Write(self,DeviceAddress,address,data)
    
    def Write_Calibration(self,data):
        DeviceAddress = self.SteeringPico
        address = 0x0000_0042
        Main_Controller.Write(self,DeviceAddress,address,data)
#-----------------------End of Write Functions---------------------------------
    
    #Float Calcs
    def Float_2_Int(self, input:float):
        Output = input * 65536
        Output:int = round(Output)
        return Output
    
    def Int_2_Float(self, input:int):
        Output:float = input / 65536.0
        return Output
    
def main(args=None):
    print("Controller Running")


if __name__ == '__main__':
    main()