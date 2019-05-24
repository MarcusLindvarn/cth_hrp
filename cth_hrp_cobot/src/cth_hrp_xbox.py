#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
#from cth_hrp_cobot.msg import ButtonMsg
from cth_hrp_cobot.msg import Command
from cth_hrp_cobot.msg import State
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus
from os import system



class agv_comms():   
    def __init__ (self):
        self.setVariables()
        
        # Publishers
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)# test diff values for queue_size
        self.ccpub = rospy.Publisher('/stateA', State, queue_size=1)
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
        #subscribing to the cmdA, button_state, sensor_status and cattery_status topics
        rospy.Subscriber("/cmdA", Command, self.callback_commandcenter)
        rospy.Subscriber('/sensor_status',SensorStatus,self.callback_sensor_status)
        rospy.Subscriber('/battery_status',BatteryStatus,self.callback_battery_status)
        rospy.Subscriber("joy", Joy, self.callback_joy)
        # starts the node
        rospy.init_node('cth_hrp_xbox')


        #self.refresh_view()
        rospy.spin()
    
    def callback_joy(self,data):
        # vertical left stick axis = linear rate
        self.twist.linear.x = self.linConst*data.axes[1]
        # horizontal left stick axis = turn rate
        self.twist.angular.z = self.angConst*data.axes[0]
        # A Pressed - Remove Loop Detection
        if (data.buttons[0]== 1):
            self.aPressed()
            self.refresh_view()
        # B Pressed - Set finished
        if (data.buttons[1]== 1):
            self.bPressed()
            self.refresh_view()
        # X Pressed - Set Executing
        if (data.buttons[2]== 1):
            self.xPressed()
            self.refresh_view()
        # Y Pressed - resend
        if (data.buttons[3]== 1):
            self.yPressed()
            self.refresh_view()
        # RB Pressed - Lock Movement
        if (data.buttons[4]== 1):
            self.lbPressed()
            self.refresh_view()
        # LB Pressed - Unock Movement
        if (data.buttons[5]== 1):
            self.rbPressed()
            self.refresh_view()
        # BACK and START Pressed
        if (data.buttons[6] == 1 and data.buttons[7]== 1):
            self.startAndBackPressed()
            
    
        self.pub_vel.publish(self.twist)



    def mowerStateToString(self, x):
        return {
            0:'OFF',
            1:'WAIT_SAFETY_PIN',
            2:'STOPPED',
            3:'FATAL_ERROR',
            4:'PENDING_START',
            5:'PAUSED',
            6:'IN_OPERATION',
            7:'RESTRICTED',
            8:'ERROR'
        }.get(x,'UNKNONW_VALUE')

    def callback_sensor_status(self,data):
    
        lastLoopState = self.loopState
        self.sensorStatus = data.sensorStatus
        if self.sensorStatus & 0x0400:
            self.loopState = 'On'
        else:
            self.loopState = 'Off'
        if lastLoopState != self.loopState:
            self.refresh_view()

    
        if (self.mowerInternalState != data.mowerInternalState):
            self.mowerInternalState = data.mowerInternalState
            self.refresh_view()

        #if in state STOPPED and loopstate != Off, try to remove loop detection - needs testing not sure of flow.
        if (self.mowerInternalState == 2 and self.loopState != 'Off'):
            self.removeLoopDetection()
            self.refresh_view()
        
    def callback_battery_status(self,data):
        self.battAVolt = data.batteryAVoltage/1000.0
        self.refresh_view()


    def callback_commandcenter(self, data):
        # update current and lastrecieved product to the recieved data - might skip
        self.last_run_recieved = data.run
    
        
        #if a new command is recieved, update current command and refresh the view
        if (self.last_command_recieved != data.command):
            self.last_command_recieved = data.command
            self.current_cmd = data.command
            self.agv_state.cmd = self.current_cmd
            self.refresh_view()
            self.ccpub.publish(self.agv_state)
        # if Run is set to False and Current status is finished, set status to init
        if (self.last_run_recieved == False and self.current_state == self.FINISHED):
            self.current_state = self.INIT
            self.agv_state.state = self.current_state
            self.refresh_view()
            self.ccpub.publish(self.agv_state)  
        #always answer a publish with a publish with current state. might need to change
        # self.ccpub.publish(agv_state)

    def aPressed(self):
        # IF A pressed - Disable Loop detection
        self.removeLoopDetection()


    def bPressed(self):
        # if current status is Exec and B is pressed, set status to finished and pub
        if (self.current_state == self.EXECUTING):
            self.current_state = self.FINISHED
            self.last_sent_cmd = self.current_state
            self.agv_state.state = self.current_state
            self.current_cmd = ""   
            self.agv_state.cmd = self.current_cmd
            self.ccpub.publish(self.agv_state) 


    def xPressed(self):
        # if current status is init and x is pressed, set status to executing and pub
        # should a check that a command has been given, meaning either button to accept or auto accept mission
        if (self.current_state == self.INIT and self.current_cmd != ""):  # more requirements needed 
            self.current_state = self.EXECUTING
            self.current_cmd = self.last_command_recieved
            self.last_sent_cmd = self.current_state
            self.agv_state.state = self.current_state
            self.agv_state.cmd = self.current_cmd
            self.ccpub.publish(self.agv_state)
    
    
    # Y - Resend State
    def yPressed(self):
        self.last_sent_cmd = self.current_state
        self.ccpub.publish(self.agv_state)
   
    #Start and Back Pressed - Shutdown
    def startAndBackPressed(self):
        print('Quitting')
        # Stop the robot
        twist = Twist()
        self.pub_vel.publish(twist)

        # Stop following loop!
        mode = UInt16()
        mode.data = 0x17 
        self.pub_mode.publish(mode)
        rospy.signal_shutdown('Shutdown')
 
    def lbPressed(self):
        #Locked Driving
        self.linConst = 0.0
        self.angConst = 0.0
        
    def rbPressed(self):    
        #Unlcked Driving
        self.linConst = 0.4
        self.angConst = 0.7
        
    def removeLoopDetection(self):
        #Removes Loop Detection
        mode = UInt16()
        mode.data = 0x111
        self.pub_mode.publish(mode)
    

    def refresh_view(self):
        system('clear')
        self.printLargeState()        
        #line1 = 'Last sent command: %s            Current State: %s' % (self.betterPrinting(self.last_sent_cmd, 9), self.current_state)
        #msg = 'BatA %.1f V                             Battery State: %s' % (self.battAVolt, self.bat_state)
        self.getCurrentStates()
        print('Last sent command: %s            Current State: %s' % (self.betterPrinting(self.last_sent_cmd, 9), self.current_state))
        print('BatA %.1f V                             Battery State: %s' % (self.battAVolt, self.bat_state))
        print('Loop Detection: %s                 ' % (self.betterPrinting(self.loopState,3)))
        print('Current Command: %s        Mower State: %s' % (self.betterPrinting (self.current_cmd,15 ), self.mowerState))
        self.printSteering()
        
    def printLargeState(self):
        if (self.current_state == self.INIT and self.current_cmd != ""):
            print(self.TEXT_MISSION)
        elif (self.current_state == self.FINISHED):
            print(self.TEXT_FINISHED)
        elif(self.current_state == self.EXECUTING):
            print(self.TEXT_EXECUTING)
        elif(self.current_state == self.INIT):
            print(self.TEXT_INITIALIZE)
        else: 
            print('error')


    def printSteering(self):
        print('\n--------------------------- CONTROLS ---------------------------')
        print('Use Left Joystick to Move               Driving: %s \n' % (self.drivingState))
        print('                              Y - Resend Last command')
        print('       State = Executing - X     B - State = Finished')
        print('                              A - Remove Loop Detection')
        print('\nLB / RB - Lock/Unlock Driving')
        print('Start + Back - Shutdown')
        
    def getCurrentStates(self):
        self.getDriveState()
        self.getBatState()
        self.getMowerState()

    def getMowerState(self):
        self.mowerState = self.mowerStateToString(self.mowerInternalState)
        
    def getDriveState(self):
        if self.linConst < 0.1:
            self.drivingState = "Locked"
        else:
            self.drivingState =  "Unlocked"
    
    def getBatState(self):
        if (self.battAVolt < 17.0):
            self.bat_state = "WARNING!"
        else:
            self.bat_state = "OK!"

    def betterPrinting(self,fix,dlen):
        newfix = fix
        fix_len = len(newfix)
        while (fix_len < dlen) :
            newfix = newfix + " "
            fix_len = len(newfix)
        return newfix    

    def setVariables(self):
        self.TEXT_INITIALIZE = """
        _____      _ _   _       _ _         
       |_   _|    (_) | (_)     | (_)        
         | | _ __  _| |_ _  __ _| |_ _______ 
         | || '_ \| | __| |/ _` | | |_  / _ \ 
        _| || | | | | |_| | (_| | | |/ /  __/
        \___/_| |_|_|\__|_|\__,_|_|_/___\___|\n\n"""

        self.TEXT_FINISHED = """
        ______ _       _     _              _ 
        |  ___(_)     (_)   | |            | |
        | |_   _ _ __  _ ___| |__   ___  __| |
        |  _| | | '_ \| / __| '_ \ / _ \/ _` |
        | |   | | | | | \__ \ | | |  __/ (_| |
        \_|   |_|_| |_|_|___/_| |_|\___|\__,_|\n\n"""

        self.TEXT_EXECUTING = """
         _____                    _   _             
        |  ___|                  | | (_)            
        | |____  _____  ___ _   _| |_ _ _ __   __ _ 
        |  __\ \/ / _ \/ __| | | | __| | '_ \ / _` |
        | |___>  <  __/ (__| |_| | |_| | | | | (_| |
        \____/_/\_\___|\___|\__,_|\__|_|_| |_|\__, |
                                               __/ |
                                              |___/ \n"""
        self.TEXT_MISSION = """
        ___  ____         _             _ 
        |  \/  (_)       (_)           | |
        | .  . |_ ___ ___ _  ___  _ __ | |
        | |\/| | / __/ __| |/ _ \| '_ \| |
        | |  | | \__ \__ \ | (_) | | | |_|
        \_|  |_/_|___/___/_|\___/|_| |_(_)\n\n"""
        # Constants
        self.FINISHED = "finished"
        self.EXECUTING = "executing"
        self.INIT = "init"



        # Initial values
        self.battAVolt = 00.0

        # Base States:
        self.agv_state = State()
        self.last_command_recieved = ""
        self.last_run_recieved = False
        self.last_product_recieved = ""
        self.current_cmd = ""
        self.current_state = self.INIT
        self.last_command_recieved = ""
        self.last_product_recieved = ""
        self.last_run_recieved = False
        self.last_sent_cmd = ""
        self.agv_state.message = ""
        self.agv_state.cmd = ""
        self.agv_state.state = ""
        self.bat_state = "UNKNOWN!"
        self.loopState = 'On'
        
        #Twist Message
        self.twist = Twist()
        
        # Velocities
        self.linConst = 0.0
        self.angConst = 0.0

        #sensor states:
        self.mowerInternalState = 0    
        # Mowerstate
        self.mowerState = self.mowerStateToString(self.mowerInternalState)
if __name__ == '__main__':
    try:
        agv_comms()
    except rospy.ROSInterruptException:
        pass