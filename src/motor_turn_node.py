#!/usr/bin/env python3


# import sys, tty, termios

from dynamixel_sdk import * # Uses Dynamixel SDK library


import rospy
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Vector3


from time import sleep


MY_DXL = 'X_SERIES'    
BAUDRATE                    = 57600
ADDR_TORQUE_ENABLE          = 64
ADDR_PROFILE_VELOCITY       = 112
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_LOAD           = 126
ADDR_PRESENT_POSITION       = 132
ADDR_GOAL_VELOCITY          = 104       # -230 to 230, 0.229/rev/min is one unit
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual


PROTOCOL_VERSION            = 2.0

# Velocity information
VELOCITY_LIMIT = 1023
VELOCITY_UNIT = 0.229 # RPM

LEFT_SHOULDER_PITCH_ID = 4
LEFT_SHOULDER_ROLL_ID = 5
LEFT_ELBOW_ROLL_ID = 6
RIGHT_SHOULDER_PITCH_ID = 3
RIGHT_SHOULDER_ROLL_ID = 2
RIGHT_ELBOW_ROLL_ID = 1

BASE_ROLL_MOTOR_ID = 7
BASE_ROTATION_MOTOR_ID = 8

LEFT_SHOULDER_PITCH_INITIAL  = 136 # -ve out
LEFT_SHOULDER_ROLL_INITIAL  = 190 # +ve out
LEFT_ELBOW_ROLL_INITIAL  = 181 # + out
RIGHT_SHOULDER_PITCH_INITIAL  = 180 # - out
RIGHT_SHOULDER_ROLL_INITIAL  = 160 #- out
RIGHT_ELBOW_ROLL_INITIAL = 222 # - out
BASE_ROLL_INITIAL = 177 # + right
BASE_ROTATION_INITIAL = 315 # - right

ANGLE_TO_POS_UNIT = 0.087891


DXL_ALL = [LEFT_SHOULDER_PITCH_ID,
           LEFT_SHOULDER_ROLL_ID,
           LEFT_ELBOW_ROLL_ID,
           RIGHT_SHOULDER_PITCH_ID,
           RIGHT_SHOULDER_ROLL_ID,
           RIGHT_ELBOW_ROLL_ID, 
           BASE_ROLL_MOTOR_ID,
           BASE_ROTATION_MOTOR_ID, 
]
# DXL_ALL = [BASE_ROTATION_MOTOR_ID, BASE_ROLL_MOTOR_ID]



DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold





class MotorInterface:
 
    

    def __init__(self):
        self.age = 5

        self.in_hug = False 

        # Init node
        rospy.init_node('dynamixel_motor_interface', anonymous=False)
    
        # Get node name
        self.node_name = rospy.get_name()
    
        # Create topics

        # The motor takes a value betwen 0 and 4095; with the angle being anticlockwise from South 
        # 0 being South, 2048 being North, 4095 one unit c/w from South
        
        self.sub_motor_goal = rospy.Subscriber('motor/motor_goal', Int32, self.motor_goal_callback)
        self.sub_camera_vec = rospy.Subscriber('camera/person_location', Vector3, self.camera_vec_callback, queue_size=1)

        # Connect to motor
        self.configure_motor()
        



    def disable_torque(self, motor_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def disable_all_torques(self):
        for id in DXL_ALL: self.disable_torque(id)

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

    def get_motor_position(self, motor_id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, ADDR_PRESENT_POSITION)
        return dxl_present_position

    def move_motor(self, motor_id, goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def reset_arms(self):
        print("RESETTING ARMS")

        self.set_velocities_same(DXL_ALL, 0.2) 

        self.move_motors(DXL_ALL, [LEFT_SHOULDER_PITCH_INITIAL,
           LEFT_SHOULDER_ROLL_INITIAL,
           LEFT_ELBOW_ROLL_INITIAL,
           RIGHT_SHOULDER_PITCH_INITIAL,
           RIGHT_SHOULDER_ROLL_INITIAL,
           RIGHT_ELBOW_ROLL_INITIAL,
           BASE_ROLL_INITIAL, 
           BASE_ROTATION_INITIAL,])

    def move_arms_up(self):
        self.set_velocity(LEFT_SHOULDER_PITCH_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_PITCH_ID, self.get_motor_position(LEFT_SHOULDER_PITCH_ID) - 800)
        self.set_velocity(RIGHT_SHOULDER_PITCH_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_PITCH_ID, self.get_motor_position(RIGHT_SHOULDER_PITCH_ID) + 800)
        return 0

    def open_arms(self, amount = 700):
        self.set_velocity(LEFT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_ROLL_ID, self.get_motor_position(LEFT_SHOULDER_ROLL_ID) - amount)
        self.set_velocity(RIGHT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_ROLL_ID, self.get_motor_position(RIGHT_SHOULDER_ROLL_ID) + amount)
        return 0

    def close_arms(self):
        self.set_velocity(LEFT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(LEFT_ELBOW_ROLL_ID, self.get_motor_position(LEFT_ELBOW_ROLL_ID) + 500)
        self.set_velocity(RIGHT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(RIGHT_ELBOW_ROLL_ID, self.get_motor_position(RIGHT_ELBOW_ROLL_ID) - 500)
        self.set_velocity(LEFT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_ROLL_ID, self.get_motor_position(LEFT_SHOULDER_ROLL_ID) + 300)
        self.set_velocity(RIGHT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_ROLL_ID, self.get_motor_position(RIGHT_SHOULDER_ROLL_ID) - 300)
        return 0

    def up_out(self):
        self.set_velocity(LEFT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(LEFT_ELBOW_ROLL_ID, self.get_motor_position(LEFT_ELBOW_ROLL_ID)- int(15/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(RIGHT_ELBOW_ROLL_ID, self.get_motor_position(RIGHT_ELBOW_ROLL_ID) + int(15/ANGLE_TO_POS_UNIT))
        self.set_velocity(LEFT_SHOULDER_PITCH_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_PITCH_ID, self.get_motor_position(LEFT_SHOULDER_PITCH_ID) - int(120/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_SHOULDER_PITCH_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_PITCH_ID, self.get_motor_position(RIGHT_SHOULDER_PITCH_ID) + int(120/ANGLE_TO_POS_UNIT))
        self.set_velocity(LEFT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_ROLL_ID, self.get_motor_position(LEFT_SHOULDER_ROLL_ID) + int(70/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_ROLL_ID, self.get_motor_position(RIGHT_SHOULDER_ROLL_ID) - int(70/ANGLE_TO_POS_UNIT))
        
    def close_hug(self):
        self.set_velocity(LEFT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(LEFT_ELBOW_ROLL_ID, self.get_motor_position(LEFT_ELBOW_ROLL_ID) - int(15/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(RIGHT_ELBOW_ROLL_ID, self.get_motor_position(RIGHT_ELBOW_ROLL_ID) + int(15/ANGLE_TO_POS_UNIT))
        self.set_velocity(LEFT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_ROLL_ID, self.get_motor_position(LEFT_SHOULDER_ROLL_ID) - int(70/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_ROLL_ID, self.get_motor_position(RIGHT_SHOULDER_ROLL_ID) + int(70/ANGLE_TO_POS_UNIT))

    def open_hug(self):
        self.set_velocity(LEFT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(LEFT_ELBOW_ROLL_ID, self.get_motor_position(LEFT_ELBOW_ROLL_ID) + int(30/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_ELBOW_ROLL_ID, 0.2)
        self.move_motor(RIGHT_ELBOW_ROLL_ID, self.get_motor_position(RIGHT_ELBOW_ROLL_ID) - int(30/ANGLE_TO_POS_UNIT))
        self.set_velocity(LEFT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(LEFT_SHOULDER_ROLL_ID, self.get_motor_position(LEFT_SHOULDER_ROLL_ID) + int(70/ANGLE_TO_POS_UNIT))
        self.set_velocity(RIGHT_SHOULDER_ROLL_ID, 0.2)
        self.move_motor(RIGHT_SHOULDER_ROLL_ID, self.get_motor_position(RIGHT_SHOULDER_ROLL_ID) - int(70/ANGLE_TO_POS_UNIT))

    def hug(self):
        self.move_arms_up()
    
        self.open_arms()
        sleep(0.75)
        self.close_arms()

        sleep(3)
        self.open_arms(amount = 400)
        sleep(1)
        self.reset_arms()
        sleep(1.5)
        self.disable_all_torques()

    def process_hug(self):
        self.in_hug = True
        self.up_out()
        sleep(3)
        self.close_hug()
        sleep(2)
    
        while True:
            sleep(0.5)
            shoulder_load = self.get_shoulder_load()
            if (shoulder_load > 48):
                print("person opening up")
                self.open_hug()
                sleep(3)
                self.reset_arms()
                sleep(2)
                break
        sleep(8)
        self.in_hug = False

    # def wave_right_arm(self):
    #     self.set_velocity(RIGHT_LOWER_ARM_YAW_ID, 0.4)
    #     self.move_motor(RIGHT_LOWER_ARM_YAW_ID, self.get_motor_position(RIGHT_LOWER_ARM_YAW_ID) - 1000)
    #     self.set_velocity(RIGHT_SHOULDER_PITCH_ID, 0.4)
    #     self.move_motor(RIGHT_SHOULDER_PITCH_ID, self.get_motor_position(RIGHT_SHOULDER_PITCH_ID) + 300)
    #     self.set_velocity(RIGHT_ELBOW_ROLL_ID, 0.4)
    #     self.move_motor(RIGHT_ELBOW_ROLL_ID, self.get_motor_position(RIGHT_ELBOW_ROLL_ID) + 1400)
    #     sleep(1)
    #     self.set_velocity(RIGHT_LOWER_ARM_YAW_ID, 0.3)
    #     self.move_motor(RIGHT_LOWER_ARM_YAW_ID, self.get_motor_position(RIGHT_LOWER_ARM_YAW_ID) + 300)
    #     sleep(0.5)
    #     self.move_motor(RIGHT_LOWER_ARM_YAW_ID, self.get_motor_position(RIGHT_LOWER_ARM_YAW_ID) - 600)
    #     sleep(0.5)
    #     self.move_motor(RIGHT_LOWER_ARM_YAW_ID, self.get_motor_position(RIGHT_LOWER_ARM_YAW_ID) + 600)
    #     sleep(0.5)
    #     self.move_motor(RIGHT_LOWER_ARM_YAW_ID, self.get_motor_position(RIGHT_LOWER_ARM_YAW_ID) - 300)

    #     return 0

    def move_motors(self, motor_ids, goals):
       
        assert(len(motor_ids) == len(goals))
        for i in range(len(motor_ids)): self.move_motor(motor_ids[i], int(goals[i]/ANGLE_TO_POS_UNIT))

    def set_velocities(self, motor_ids, velocities):
        assert(len(motor_ids) == len(velocities))
        for i in range(len(motor_ids)): 
            self.set_velocity(motor_ids[i], velocities[i])

    def set_velocities_same(self, motor_ids, velocity):
        for i in range(len(motor_ids)): 
            self.set_velocity(motor_ids[i], velocity)

    def set_velocity(self, motor_id, speed_frac):
        max_speed = VELOCITY_UNIT * VELOCITY_LIMIT
        new_rpm = int(speed_frac * max_speed)

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_PROFILE_VELOCITY, new_rpm)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return 0

    def get_elbow_load(self):
        present_load_l, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, LEFT_ELBOW_ROLL_ID, ADDR_PRESENT_LOAD)
        present_load_r, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, RIGHT_ELBOW_ROLL_ID, ADDR_PRESENT_LOAD)

        return max(min(present_load_r, 65535-present_load_r), min(present_load_l, 65535-present_load_l))

    def get_shoulder_load(self):
        present_load_l, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, LEFT_SHOULDER_ROLL_ID, ADDR_PRESENT_LOAD)
        present_load_r, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, RIGHT_SHOULDER_ROLL_ID, ADDR_PRESENT_LOAD)

        return max(min(present_load_r, 65535-present_load_r), min(present_load_l, 65535-present_load_l))

    def motor_goal_callback(self, data):
        goal_position = data.data
        print("Goal: ", goal_position)

        for id in DXL_ALL:
            self.move_motor(id, int(goal_position))


    def camera_vec_callback(self, data):
        # turn_angle = data.data/ANGLE_TO_POS_UNIT
        # print("Desired turn value: ", turn_angle)
        # self.set_velocity(BASE_ROTATION_MOTOR_ID, 0.2)
        # self.move_motor(1, int(BASE_ROTATION_INITIAL/ANGLE_TO_POS_UNIT - turn_angle))
        print(data.x, data.y, data.z)
        self.set_velocity(BASE_ROTATION_MOTOR_ID, 0.05)
        initialpos = self.get_motor_position(BASE_ROTATION_MOTOR_ID)
        if (data.z < 400 and data.x <= 150 and data.x > -150):
            print("processing hug")
            sleep(2)
            self.process_hug()
        elif (data.x > 150 and initialpos*ANGLE_TO_POS_UNIT >= 268): # move right
            self.move_motor(BASE_ROTATION_MOTOR_ID, int((initialpos*ANGLE_TO_POS_UNIT - 7)/ANGLE_TO_POS_UNIT))
        elif (data.x < -150 and initialpos*ANGLE_TO_POS_UNIT <= 354):
            self.move_motor(BASE_ROTATION_MOTOR_ID, int((initialpos*ANGLE_TO_POS_UNIT + 7)/ANGLE_TO_POS_UNIT))
     
            

        

    def run(self):
        # Set frequency
        rate = rospy.Rate(2)

        # self.disable_all_torques()
        # return


        self.reset_arms()
        sleep(2)
    
        while not rospy.is_shutdown():
            rate.sleep()
    
        
        self.disable_all_torques()
        self.portHandler.closePort()


if __name__ == '__main__':
   motor_interface = MotorInterface()

   try:
       motor_interface.run()
   except rospy.ROSInterruptException:
       pass