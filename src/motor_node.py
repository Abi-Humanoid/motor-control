#!/usr/bin/env python3


# import sys, tty, termios

from dynamixel_sdk import * # Uses Dynamixel SDK library


import rospy
from std_msgs.msg import Int32, String


MY_DXL = 'X_SERIES'    
BAUDRATE                    = 57600
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_GOAL_VELOCITY          = 104       # -230 to 230, 0.229/rev/min is one unit
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual


PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL1_ID                  = 2
DXL2_ID                  = 3
DXL3_ID                  = 4

DXL_ALL = [DXL1_ID, DXL2_ID, DXL3_ID]


DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold



class MotorInterface:
 
    def __init__(self):
 
        # Init node
        rospy.init_node('dynamixel_motor_interface', anonymous=False)
    
        # Get node name
        self.node_name = rospy.get_name()
    
        # Create topics

        # The motor takes a value betwen 0 and 4095; with the angle being anticlockwise from South 
        # 0 being South, 2048 being North, 4095 one unit c/w from South
        
        self.sub_motor_goal = rospy.Subscriber('motor/motor_goal', Int32, self.motor_goal_callback)


        # Connect to motor
        self.configure_motor()
        



    def disable_torque(self, motor_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))



    def configure_motor(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Enable Dynamixel Torque
        for id in DXL_ALL:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"Dynamixel {id} has been successfully connected")



    def move_motor(self, motor_id, goal_position, goal_velocity = 0):
        # Set velocity
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_VELOCITY, goal_velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Set goal position
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))



    def move_motors(self, motor_ids, goals):
        assert(len(motor_ids) == len(goals))
        for i in range(motor_ids): self.move_motor(motor_ids[i], goals[i])



    def motor_goal_callback(self, data):
        goal_position = data.data
        print("Goal: ", goal_position)

        for id in DXL_ALL:
            self.move_motor(id, goal_position)
        

    def run(self):
        # Set frequency
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            rate.sleep()
        
        self.portHandler.closePort()


if __name__ == '__main__':
   motor_interface = MotorInterface()

   try:
       motor_interface.run()
   except rospy.ROSInterruptException:
       pass
