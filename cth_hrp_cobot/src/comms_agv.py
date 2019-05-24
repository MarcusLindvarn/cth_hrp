#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from cth_hrp_cobot.msg import ButtonMsg
from cth_hrp_cobot.msg import Command
from cth_hrp_cobot.msg import State
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus
from os import system



class agv_comms():   
    def __init__ (self):
        self.setVariables()
        self.ccpub = rospy.Publisher('/stateA', State, queue_size=1)
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
        #subscribing to the cmdA, button_state, sensor_status and cattery_status topics
        rospy.Subscriber("/cmdA", Command, self.callback_commandcenter)
        rospy.Subscriber("/button_state", ButtonMsg, self.callback_button_state)
        rospy.Subscriber('/sensor_status',SensorStatus,self.callback_sensor_status)
        rospy.Subscriber('/battery_status',BatteryStatus,self.callback_battery_status)
        # starts the node
        rospy.init_node('com_oper_node')
        self.refresh_view()
        rospy.spin()
    
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
        if (self.mowerInternalState != data.mowerInternalState):
            self.mowerInternalState = data.mowerInternalState
            self.refresh_view()
        
    def callback_battery_status(self,data):
        self.battAVolt = data.batteryAVoltage/1000.0
        self.battBVolt = data.batteryBVoltage/1000.0
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

    def callback_button_state(self, data):
        # if current status is init and x is pressed, set status to executing and pub
        # should a check that a command has been given, meaning either button to accept or auto accept mission
        if (self.current_state == self.INIT and self.current_cmd != ""):  # more requirements needed 
            if (data.xpress == True):
                self.current_state = self.EXECUTING
                self.current_cmd = self.last_command_recieved
                self.last_sent_cmd = self.current_state
                self.agv_state.state = self.current_state
                self.agv_state.cmd = self.current_cmd
                self.ccpub.publish(self.agv_state)
            #Add accept button ? 


        # if current status is Exec and B is pressed, set status to finished and pub
        if (self.current_state == self.EXECUTING):
            if (data.bpress == True):
                self.current_state = self.FINISHED
                self.last_sent_cmd = self.current_state
                self.agv_state.state = self.current_state
                self.current_cmd = ""   
                self.agv_state.cmd = self.current_cmd
                self.ccpub.publish(self.agv_state) 

        if data.startpress == True and data.backpress == True:
            print('Quitting')
            rospy.signal_shutdown('Shutdown')


        # IF A pressed - Disable Loop 
        if (data.apress == True):  
            mode = UInt16()
            mode.data = 0x111
            self.pub_mode.publish(mode)
        # Resend State
        if data.ypress == True:
            self.last_sent_cmd = self.current_state
            self.ccpub.publish(self.agv_state)

        #Locked Driving
        if data.lbpress == True:
            self.lockedState = True

        #Unlocked Driving
        if data.rbpress == True:
            self.lockedState = False
        self.refresh_view()


    def refresh_view(self):
        system('clear')
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
        
        line1 = 'Last sent command: %s            Current State: %s' % (self.betterPrinting(self.last_sent_cmd, 9), self.current_state)
        print(line1)
        self.getCurrentStates()
        msg = 'BatA %.1f V                             Battery State: %s' % (self.battAVolt, self.bat_state)
        print(msg)
        mowerState = self.mowerStateToString(self.mowerInternalState)

        print('\nCurrent Command: %s        Mower State: %s' % (self.betterPrinting (self.current_cmd,15 ), mowerState))
        self.printSteering()
        

    def printSteering(self):
        print('\n--------------------------- CONTROLS ---------------------------')
        print('Use Left Joystick to Move               Driving: %s \n' % (self.drivingState))
        print('                              Y - Resend Last command')
        print('       State = Executing - X     B - State = Finished')
        print('                              A - Remove Loop Detection')
        print('\nLB / RB - Lock/Unlock Driving')
        
        
    def getCurrentStates(self):
        self.getDriveState()
        self.getBatState()
        
    def getDriveState(self):
        if self.lockedState == True:
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
        \___/_| |_|_|\__|_|\__,_|_|_/___\___|
                                            

        """

        self.TEXT_FINISHED = """
        ______ _       _     _              _ 
        |  ___(_)     (_)   | |            | |
        | |_   _ _ __  _ ___| |__   ___  __| |
        |  _| | | '_ \| / __| '_ \ / _ \/ _` |
        | |   | | | | | \__ \ | | |  __/ (_| |
        \_|   |_|_| |_|_|___/_| |_|\___|\__,_|
                                                                       

        """

        self.TEXT_EXECUTING = """
         _____                    _   _             
        |  ___|                  | | (_)            
        | |____  _____  ___ _   _| |_ _ _ __   __ _ 
        |  __\ \/ / _ \/ __| | | | __| | '_ \ / _` |
        | |___>  <  __/ (__| |_| | |_| | | | | (_| |
        \____/_/\_\___|\___|\__,_|\__|_|_| |_|\__, |
                                               __/ |
                                              |___/                                                 
        """
        self.TEXT_MISSION = """
        ___  ____         _             _ 
        |  \/  (_)       (_)           | |
        | .  . |_ ___ ___ _  ___  _ __ | |
        | |\/| | / __/ __| |/ _ \| '_ \| |
        | |  | | \__ \__ \ | (_) | | | |_|
        \_|  |_/_|___/___/_|\___/|_| |_(_)
                                  
                                                                                  
        """
        # Constants
        self.FINISHED = "finished"
        self.EXECUTING = "executing"
        self.INIT = "init"



        # Initial values
        self.battAVolt = 00.0
        self.battBVolt = 00.0

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
        self.lockedState = True
        

        #sensor states:
        self.mowerInternalState = 0    

if __name__ == '__main__':
    try:
        agv_comms()
    except rospy.ROSInterruptException:
        pass
