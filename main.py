from PyQt5 import QtWidgets, uic, QtCore
from PyQt5.QtGui import QIntValidator, QDoubleValidator
import sys
import time
import datetime
import math
import serial
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.backends.backend_qt5agg import (
        FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

class Ui(QtWidgets.QMainWindow):    
    # INITIAL SETUP
    def __init__(self):
        super(Ui, self).__init__()
        # LOAD GUI FILE
        uic.loadUi('gui.ui', self)
        self.resize(2000, 1400)
		
        # CREATING ROBOT ARM OBJECT
        self.robotArm = RobotArm()
        
        # CONNECTING STANDARD BUTTONS
        self.linkStandardButtons()

        # CONNECTING SEQUENCE FILE BUTTONS
        self.linkSequenceFileButtons()
        
        # CONNECTING ARM CONTROL BUTTONS
        self.linkArmButtons()

        # CONNECTING POSITION TEXT EDITS
        self.linkManualCoordinateButtons()
		
        # CONNECTING END EFFECTOR CONTROL BUTTONS
        self.linkEffectorButtons()

        # INITIALLY DISABLE FILE MODIFIER AND MOVEMENT BUTTONS
        self.enableFileButtons(False)
        self.enablePlaybackButtons(False)
        self.enableSequenceChangeButtons(False)
        self.enableMovementButtons(False)

        # LOAD ARM SIMULATIONGRAPH SETTINGS
        self.updateGraph(True)

         # UPDATE SEQUENCE CURRENT POSITION
        self.sequenceEntryUpdate()

        # FIND AVAILABLE COM PORTS
        self.findComPorts()

        # ADD ITEMS TO SETUP MENUS
        self.addMicroStepping()
        
        # INITIALISE UI
        self.show()
        
    ############################
    ### UI LINKING FUNCTIONS ###
    ############################

    def linkStandardButtons(self):
        self.connect.clicked.connect(self.initiateSerialComms)
        self.con_stat.setStyleSheet('QLabel { color: red }')
        self.serial_port_list.activated.connect(self.changeComPort)
        self.serial_refresh.clicked.connect(self.findComPorts)
        self.arm_home.clicked.connect(self.homeRobot)
        self.endeff_home.clicked.connect(self.homeGripper)
        self.arm_calibrate.clicked.connect(self.calibrateRobot)
        self.micro_stepping_list.activated.connect(self.changeMicroStepping)
        self.max_speed_list.activated.connect(self.changeMaxFeedRate)
        self.program_exit.clicked.connect(self.programExit)

    # LINKS BUTTONS THAT CONTROL SEQUENCE FILE
    def linkSequenceFileButtons(self):
        self.sequence_add.clicked.connect(self.fileAdd)
        self.sequence_replace.clicked.connect(self.fileReplace)
        self.sequence_remove.clicked.connect(self.fileRemove)
        self.linear_motion.toggled.connect(self.linearMotionEnable)
        self.sequence_move_up.clicked.connect(self.fileMoveUp)
        self.sequence_move_down.clicked.connect(self.fileMoveDown)
        self.sequence_play.clicked.connect(self.filePlay)
        self.sequence_stop.clicked.connect(self.fileStop)
        self.sequence_repeat.toggled.connect(self.fileRepeat)
        self.sequence_next.clicked.connect(self.fileNext)
        self.sequence_previous.clicked.connect(self.filePrevious)
        self.arm_enable.toggled.connect(self.armEnable)
        self.sequence_new.clicked.connect(self.fileNew)
        self.sequence_open.clicked.connect(self.fileOpen)
        self.sequence_save.clicked.connect(self.fileSave)
        self.sequence_close.clicked.connect(self.fileClose)
        self.sequence_show.itemClicked.connect(self.sequenceItemClicked)
        self.sequence_speed.valueChanged.connect(self.sequenceSpeedChange)

    # LINKS BUTTONS THAT CONTROL ARM MOTION      
    def linkArmButtons(self):
        self.arm_forward.clicked.connect(self.incrementYPos)
        self.arm_backward.clicked.connect(self.decrementYPos)        
        self.arm_right.clicked.connect(self.incrementXPos)       
        self.arm_left.clicked.connect(self.decrementXPos)
        self.arm_up.clicked.connect(self.incrementZPos)
        self.arm_down.clicked.connect(self.decrementZPos)
        self.arm_cw_rotation.clicked.connect(self.incrementArmAngle)
        self.arm_ccw_rotation.clicked.connect(self.decrementArmAngle)
        self.arm_jog_step.valueChanged.connect(self.jogStepArm)
        self.arm_angle_step.valueChanged.connect(self.angleStepArm)
        self.arm_jog_step.setValue(self.robotArm.armJogStep)
        self.arm_jog_step_show.setText(str(self.robotArm.armJogStep))
        self.arm_angle_step.setValue(self.robotArm.armAngleStep)
        self.arm_angle_step_show.setText(str(self.robotArm.armAngleStep))
        self.sequence_speed_show.setText(str(self.robotArm.sequenceSpeed))
        self.sequence_speed.setValue(self.robotArm.sequenceSpeed)

    # LINKS POSITION TEXT BOXED SO THAT USER CAN ENTER COORDINATES FOLLOWED BY THE ENTER KEY
    def linkManualCoordinateButtons(self):
        self.x_show.returnPressed.connect(self.xPosEntered)
        self.x_show.setValidator(QDoubleValidator())
        self.y_show.returnPressed.connect(self.yPosEntered)
        self.y_show.setValidator(QDoubleValidator())
        self.z_show.returnPressed.connect(self.zPosEntered)
        self.z_show.setValidator(QDoubleValidator())
        self.arm_angle_show.returnPressed.connect(self.armRotationEntered)
        self.arm_angle_show.setValidator(QDoubleValidator())
        self.endeff_angle_show.returnPressed.connect(self.endeffRotationEntered)
        self.endeff_angle_show.setValidator(QIntValidator())
        self.endeff_gripper_show.returnPressed.connect(self.gripperPosEntered)
        self.endeff_gripper_show.setValidator(QIntValidator())

    # LINKS BUTTONS THAT CONTROL END EFFECTOR MOTION
    def linkEffectorButtons(self):
        self.endeff_cw_rotation.clicked.connect(self.incrementEffectorAngle)        
        self.endeff_ccw_rotation.clicked.connect(self.decrementEffectorAngle)
        self.endeff_close.clicked.connect(self.incrementGripper)        
        self.endeff_open.clicked.connect(self.decrementGripper)
        self.endeff_angle_step.valueChanged.connect(self.effectorAngleStep)
        self.endeff_gripper_step.valueChanged.connect(self.effectorGripperStep)
        self.endeff_angle_step.setValue(self.robotArm.endeffAngleStep)
        self.endeff_angle_step_show.setText(str(self.robotArm.endeffAngleStep))
        self.endeff_gripper_step.setValue(self.robotArm.endeffGripperStep)
        self.endeff_gripper_step_show.setText(str(self.robotArm.endeffGripperStep))
        
    # ENABLE/DISABLE SEQUENCE CONTROL BUTTONS
    def enableFileButtons(self,state):
        self.sequence_add.setEnabled(state)
        self.sequence_save.setEnabled(state)
        self.sequence_close.setEnabled(state)
        
    # ENABLE/DISABLE ARM CONTROL BUTTONS
    def enableMovementButtons(self,state):
        self.arm_forward.setEnabled(state)
        self.arm_backward.setEnabled(state)
        self.arm_left.setEnabled(state)
        self.arm_right.setEnabled(state)
        self.arm_up.setEnabled(state)
        self.arm_down.setEnabled(state)
        self.arm_ccw_rotation.setEnabled(state)
        self.arm_cw_rotation.setEnabled(state)
        self.endeff_ccw_rotation.setEnabled(state)
        self.endeff_cw_rotation.setEnabled(state)
        self.endeff_close.setEnabled(state)
        self.endeff_open.setEnabled(state)    
        self.arm_home.setEnabled(state)    
        self.endeff_home.setEnabled(state)   
        self.arm_calibrate.setEnabled(state)
        self.sequence_speed.setEnabled(state)

    # ENABLE/DISABLE SEQUENCE PLAYBACK CONTROL BUTTONS
    def enablePlaybackButtons(self,state):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
       
        # ONLY ENABLES PLAYBACK BUTTONS IF FILE IS NOT EMPTY AND SERIAL IS CONNECTED
        if self.robotArm.commsStatus == True and self.robotArm.sequenceLength > 0:
            self.sequence_play.setEnabled(state)
            self.sequence_stop.setEnabled(state)
            self.sequence_repeat.setEnabled(state)
            self.arm_enable.setEnabled(state)
        
        else:
            self.sequence_play.setEnabled(False)
            self.sequence_stop.setEnabled(False)
            self.sequence_repeat.setEnabled(False)
            self.arm_enable.setEnabled(False)

    # ENABLE/DISABLE SEQUENCE EDIT BUTTONS
    def enableSequenceChangeButtons(self,state):
        self.sequence_replace.setEnabled(state)
        self.sequence_remove.setEnabled(state)
        self.linear_motion.setEnabled(state)
        self.sequence_move_up.setEnabled(state)
        self.sequence_move_down.setEnabled(state)
        self.sequence_next.setEnabled(state)
        self.sequence_previous.setEnabled(state)

    # DISABLE EVERYTHING WHEN SEQUENCE IS BEING PLAYED BACK
    def enableSequencePlayback(self,state):
        self.sequence_play.setEnabled(state)
        self.sequence_repeat.setEnabled(state)
        self.arm_enable.setEnabled(state)
        self.sequence_new.setEnabled(state)
        self.sequence_open.setEnabled(state)
        self.connect.setEnabled(state)
        self.serial_port_list.setEnabled(state)
        self.serial_refresh.setEnabled(state)

        self.x_show.setEnabled(state)
        self.y_show.setEnabled(state)
        self.z_show.setEnabled(state)
        self.arm_angle_show.setEnabled(state)
        self.endeff_angle_show.setEnabled(state)
        self.endeff_gripper_show.setEnabled(state)     

        self.enableFileButtons(state)
        self.enableMovementButtons(state)
        self.enableSequenceChangeButtons(state)

    #####################
    ### EVENT METHODS ###
    #####################

    ##################################
    ### ROBOT SERIAL COMMUNICATION ###
    ##################################
    def findComPorts(self):
        # DISCONNECTED FROM CURRENT COM PORT IF ALREADY CONNECTED
        if self.robotArm.commsStatus == True:
            self.serialDisconnect()

        # CREATE LIST OF ALL POSSIBLE COM PORTS
        self.ports = ['COM%s' % (i + 1) for i in range(256)] 

        # CLEAR CURRENT MENU LIST
        self.serial_port_list.clear()

        # CHECK WHICH COM PORTS ARE AVAILABLE
        self.robotArm.availableComPorts = []
        self.serial_terminal.appendPlainText('Searching for available COM ports...')
        for port in self.ports:
            try:
                comms = serial.Serial(port)
                comms.close()

                # ADD AVAILABLE COM PORTS TO MENU LIST
                self.robotArm.availableComPorts.append(port)
                self.serial_port_list.addItem(port)
            
            # SKIP COM PORT IF UNAVAILABLE
            except (OSError, serial.SerialException):
                pass

        if self.robotArm.availableComPorts != []:
            self.serial_terminal.appendPlainText('%s COM ports found' % len(self.robotArm.availableComPorts))
        else:
            self.serial_terminal.appendPlainText('No COM ports available')
                                                
    def serialConnect(self):
        # CONNECT TO SELECTED COM PORT
        if self.robotArm.availableComPorts != []:
            self.comms = serial.Serial(self.robotArm.availableComPorts[self.robotArm.comPort],115200)
            # 5 SECOND SERIAL READ TIMEOUT
            self.comms.timeout = 5
            self.robotArm.commsStatus = True
            self.connect.setText('Disconnect')
            self.con_stat.setText('CONNECTED')
            self.con_stat.setStyleSheet('QLabel { color: green }')
            self.enableMovementButtons(True)
            self.serial_terminal.appendPlainText('Connected to %s' % self.robotArm.availableComPorts[self.robotArm.comPort])

            # ONLY SHOWS PLAYBACK BUTTONS IF THERE IS A VALID SEQUENCE LOADED
            if self.robotArm.fileStatus == True:
                self.enablePlaybackButtons(True)
            else:
                self.enablePlaybackButtons(False)

            # RUN ROBOT ARM INITIAL SETUP FUNCTIONS
            self.gCodeInitialSetup()
            
            # CALIBRATE ARM LOCATION USING END STOP SWITCHES
            self.calibrateRobot()

        else:
            self.serial_terminal.appendPlainText('No COM ports available')
            
    def serialDisconnect(self):
        self.comms.close()
        self.robotArm.commsStatus = False
        self.connect.setText('Connect')
        self.con_stat.setText('DISCONNECTED')
        self.con_stat.setStyleSheet('QLabel { color: red }')
        self.enableMovementButtons(False)
        self.enablePlaybackButtons(False)
        self.serial_terminal.appendPlainText('Disconnected')
        
    def changeComPort(self,item):
        # DISCONNECTED FROM CURRENT COM PORT IF ALREADY CONNECTED AND A DIFFERENT PORT IS SELECTED
        if self.robotArm.commsStatus == True and (self.robotArm.availableComPorts[self.robotArm.comPort] != self.robotArm.availableComPorts[item]):
            self.serialDisconnect()
            
        # SET COM PORT TO SELECTED ITEM
        self.robotArm.comPort = item

    def initiateSerialComms(self):
        if self.robotArm.commsStatus == False:
            try:   
                # TRY AND CONNECTED TO SELECTED COM PORT 
                self.serialConnect()
                
            except (OSError, serial.SerialException):
                self.serial_terminal.appendPlainText('Connection failed!')
            
        else:
            self.serialDisconnect()

    def programExit(self):
        pass

    #############################
    ### SEQUENCE FILE CONTROL ###
    #############################
    def fileAdd(self):
        if self.robotArm.fileStatus == True:
            
            # ADD CURRENT POSITION TO LIST
            self.robotArm.sequenceElements.append(str(self.robotArm.currentPosition))
            self.sequenceListUpdate()
            
            # SET CURRENT POSITION TO END OF LIST
            self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
            self.robotArm.sequenceSelected = self.robotArm.sequenceLength - 1
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)
            
            # SHOW SEQUENCE MODIFER BUTTONS
            self.enableSequenceChangeButtons(True)
            self.enablePlaybackButtons(True)

            # UPDATE SEQUENCE PROGRESS BAR
            self.calculateSequenceProgress()

    def fileReplace(self):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceLength > 0:
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected] = self.robotArm.currentPosition 
            self.sequenceListUpdate()
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)

    def fileRemove(self):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceLength > 0:
            # DELETES SELECTED ITEM AND INCREMENTS SEQUENCE POSITION
            del self.robotArm.sequenceElements[self.robotArm.sequenceSelected]
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)
            
        # MOVES SELECTED ITEM UP WHEN LAST ITEM IN LIST IS REMOVED    
        if self.robotArm.sequenceSelected == self.robotArm.sequenceLength - 1:
                self.robotArm.sequenceSelected -= 1
                self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)
        
        # DISABLES SEQUENCE MOFIFIER BUTTONS IF LIST IS NOW EMPTY           
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceLength == 0:
            self.enableSequenceChangeButtons(False)
            self.enablePlaybackButtons(False)

        # UPDATE SEQUENCE LIST
        self.sequenceListUpdate()
        # UPDATES SEQUENCE PROGRESS BAR
        self.calculateSequenceProgress()
    
    def fileMoveUp(self):
        if self.robotArm.sequenceSelected > 0:
            # SWAP ELEMENTS IN ARRAY
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected],\
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected - 1] =\
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected - 1],\
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected]

            # SETS NEW SELECTED LIST ITEM
            self.robotArm.sequenceSelected -= 1

            self.sequenceListUpdate()
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)

            # UPDATE SEQUENCE PROGRESS BAR
            self.calculateSequenceProgress()

    def fileMoveDown(self):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceSelected < self.robotArm.sequenceLength - 1:
            # SWAP ELEMENTS IN ARRAY
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected],\
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected + 1] =\
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected + 1],\
            self.robotArm.sequenceElements[self.robotArm.sequenceSelected]

            # SETS NEW SELECTED LIST ITEM
            self.robotArm.sequenceSelected += 1

            self.sequenceListUpdate()
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)

            # UPDATE SEQUENCE PROGRESS BAR
            self.calculateSequenceProgress()

    def filePlay(self):
        self.enableSequencePlayback(False)
        self.robotArm.playbackStatus = True
        self.serial_terminal.appendPlainText('%s sequence playback started' % self.robotArm.fileName)
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        while(self.robotArm.sequenceSelected != self.robotArm.sequenceLength) and self.robotArm.playbackStatus == True:
            # GET XYZ COORDINATES FOR CURRENT POSITION
            self.sequenceConverter()
            # UPDATE POSITION AND CALCULATE GCODE COMMAND
            self.positionUpdate()
            # SEND GCODE COMMAND TO ROBOT CONTROL BOARD
            self.comms.write(self.robotArm.outputGCode.encode('ascii'))

            while True:
                # READ CURRENT ARM POSITION FROM ARM CONTROL BOARD
                self.comms.write('?'.encode('ascii'))
                position = str(self.comms.readline().decode('ascii'))

                startTime = startTime = datetime.datetime.now()
                currentTime = 0
                timeDifference = 0

                # WAIT FOR ARM TO START MOVING (WHEN 'IDLE' BECOMES 'RUN')
                while ('Idle' in position) and self.robotArm.playbackStatus == True:
                    self.comms.write('?'.encode('ascii'))
                    position = str(self.comms.readline().decode('ascii'))
                    # UPDATE GUI
                    QtWidgets.QApplication.processEvents()
                    # MOVE ON TO NEXT POSITION IF THE ARM DOES NOT START MOVING AFTER 1 SECOND
                    # THIS OCCURS WHEN THE ARM IS ALREADY IN THE DESIRED POSITION
                    # GETTING CURRENT TIME
                    currentTime = datetime.datetime.now()
                     # CALCULATING SECONDS ELAPSED
                    timeDifference = (currentTime - startTime).total_seconds()
                    if timeDifference > 1:
                        break

                # WAIT FOR ARM TO FINISH MOVEMENT
                # GET CURRENT ARM POSITION
                while 'Run' in position:
                    gcodeCurrentPosition = position
                    # REMOVE NON-NUMERICAL CHARACTERS TO EXTRACT JUST THE COORDINATES
                    junk = ['<Idle|','<Run|','MPos:','|',',']   
                    for item in junk:
                        gcodeCurrentPosition = gcodeCurrentPosition.replace(item, ' ')
                        # PUT CURRENT POSITION IN ARRAY
                        self.robotArm.gcodeResponse = gcodeCurrentPosition.split()

                    # STORE ACTUAL ARM POSITION
                    self.robotArm.lowerArmAngle = float(self.robotArm.gcodeResponse[0])
                    self.robotArm.upperArmAngle = float(self.robotArm.gcodeResponse[1])
                    self.robotArm.rotationAngle = float(self.robotArm.gcodeResponse[2])
                    # SHOW CURRENT POSITION
                    self.kinematics_lower_show.setText(str(self.robotArm.lowerArmAngle))
                    self.kinematics_upper_show.setText(str(self.robotArm.upperArmAngle))
                    self.kinematics_rotation_show.setText(str(self.robotArm.rotationAngle))
                    # UPDATE GUI
                    QtWidgets.QApplication.processEvents()
                    # UPDATE ARM SIMULATION GRAPH
                    self.robotArm.calculateJointCoordinates()
                    self.updateGraph(False)

                    # SET REFRESH RATE
                    self.timeDelay(0.1)

                    # REFRESH CURRENT POSITION    
                    self.comms.write('?'.encode('ascii'))
                    position = str(self.comms.readline().decode('ascii'))

                # WHEN ARM HAS FINISHED MOVEMENT
                if 'Idle' in position:
                    self.serial_terminal.appendPlainText('Move complete')
                    break           

            # STOP SEQUENCE IF IT HAS REACHED LAST POSITION
            if self.robotArm.sequenceSelected == self.robotArm.sequenceLength - 1:
                self.fileStop()
            else:
                # MOVE TO NEXT POSITION
                self.fileNext()

        # RESET PLAYBACK STATUS AND RE-ENABLE BUTTONS    
        self.robotArm.playbackStatus = False
        self.enableSequencePlayback(True)

    def fileStop(self):
        self.robotArm.playbackStatus = False
        self.serial_terminal.appendPlainText('Sequence playback stopped')
        pass

    def fileRepeat(self):
        state = self.sender()
        if state.isChecked():
            self.robotArm.sequenceRepeat = True
        else:
            self.robotArm.sequenceRepeat = False
               
    def fileNext(self):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceSelected < self.robotArm.sequenceLength - 1:
            self.robotArm.sequenceSelected += 1
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)
            self.sequenceListUpdate()
            
            # UPDATE SEQUENCE PROGRESS BAR
            self.calculateSequenceProgress()
    
    def filePrevious(self):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceSelected > 0:
            self.robotArm.sequenceSelected -= 1
            self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)
            self.sequenceListUpdate()

            # UPDATE SEQUENCE PROGRESS BAR
            self.calculateSequenceProgress()

    def armEnable(self):
        state = self.sender()
        if state.isChecked():
            self.robotArm.sequenceArmEnable = True
            self.serial_terminal.appendPlainText('Arm position set to track sequence list')
            self.sequenceListUpdate()
        else:
            self.robotArm.sequenceArmEnable = False
            self.serial_terminal.appendPlainText('Arm positioning set to manual jog control')

    def linearMotionEnable(self):
        # WHEN ENABLED, ARM WILL MOVE TO POSITION IN A LINEAR FASHION VIA KINEMATIC EQUATIONS
        state = self.sender()
        if state.isChecked():
            self.robotArm.linearMotion = 1
        else:
            self.robotArm.linearMotion = 0
        self.sequenceEntryUpdate()

    def fileNew(self):
        # USER CHOOSES FILE NAME
        self.robotArm.fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,'New File','./','Text Documents (*.txt)')
        
        # IF FILE NAME IS GIVEN, FILE IS GENERATED
        if self.robotArm.fileName != '':
            self.robotArm.sequenceFile = open(self.robotArm.fileName,'w')
            # CLEARS CURRENT LIST
            self.robotArm.sequenceFile.close()
            self.robotArm.sequenceElements = []
            self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
            self.sequenceListUpdate()
            self.enableFileButtons(True)
            self.enableSequenceChangeButtons(False)
            self.enablePlaybackButtons(False)
            self.robotArm.fileStatus = True
            # UPDATE SEQUENCE PROGRESS BAR
            self.calculateSequenceProgress()
            self.serial_terminal.appendPlainText('New sequence file created')

    def fileOpen(self):
        # USER CHOOSES SEQUENCE FILE
        self.robotArm.fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Open File','./','Text Documents (*.txt)')
        if self.robotArm.fileName != '':
            self.robotArm.sequenceFile = open(self.robotArm.fileName,'r')
            
            # SPLIT EACH ROW INTO SEPERATE LIST ELEMENTS
            self.robotArm.sequenceElements = self.robotArm.sequenceFile.readlines()
            self.robotArm.sequenceFile.close()        
         
            # DISPLAY SEQUENCE AS LIST
            self.sequenceListUpdate()
            
            # DISABLE SEQUENCE MODIFIER BUTTONS IF LIST IS EMPTY
            self.enableSequenceChangeButtons(False)
            self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
            if self.robotArm.sequenceLength > 0:
                self.enableSequenceChangeButtons(True)
                
                # SET SEQUENCE LOCATION TO FIRST ITEM IN LIST
                self.robotArm.sequenceSelected = 0
                self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)

                # UPDATE SEQUENCE PROGRESS BAR
                self.calculateSequenceProgress()
            
            self.enableFileButtons(True)
            self.enablePlaybackButtons(True)
            self.robotArm.fileStatus = True
            self.serial_terminal.appendPlainText('%s sequence file opened' % self.robotArm.fileName)
        
    def fileSave(self):
        # REOPEN FILE
        if self.robotArm.fileStatus == True:
            self.robotArm.sequenceFile = open(self.robotArm.fileName,'w')

            # WRITES EACH SEQUENCE ELEMENT TO FILE
            for sequenceLine in self.robotArm.sequenceElements:
                self.robotArm.sequenceFile.write(sequenceLine)
                      
            self.robotArm.sequenceFile.close()
            self.serial_terminal.appendPlainText('Sequence file saved')
    
    def fileClose(self):
        self.robotArm.sequenceFile.close() 
        # CLEAR ALL ELEMENTS IN THE SEQUENCE
        self.robotArm.sequenceElements.clear()
        self.robotArm.sequenceArmEnable = False
        self.sequenceEntryUpdate()
        # CLEAR SEQUENCE LIST
        self.sequence_show.clear()
        self.enableFileButtons(False)
        self.enableSequenceChangeButtons(False)
        self.enablePlaybackButtons(False)
        self.robotArm.fileStatus = False
        # UPDATE SEQUENCE PROGRESS BAR
        self.calculateSequenceProgress()
        self.serial_terminal.appendPlainText('Sequence file closed')
        
    def sequenceItemClicked(self, item):
        # FINDS ARRAY INDEX OF SELECTED ITEM
        self.robotArm.sequenceSelected = self.sequence_show.selectedIndexes()[0].row()
        # SET CURRENT POSITION AS SELECTED LIST ELEMENTS IF ARM IS ENABLED
        self.sequenceListUpdate()
        # UPDATE SEQUENCE PROGRESS BAR
        self.calculateSequenceProgress()
	
    def sequenceSpeedChange(self,item):
        self.robotArm.sequenceSpeed = self.sequence_speed.value()
        self.sequence_speed_show.setText(str(self.robotArm.sequenceSpeed))
        self.sequenceEntryUpdate()

    #########################
    ### ROBOT ARM CONTROL ###
    #########################
    def addMicroStepping(self):
        # ADD AVAILABLE MICROSTEPPING OPTIONS TO DROP DOWN MENU
        for step in self.robotArm.microSteps:
            self.micro_stepping_list.addItem(step)

        # INITIALLY SET BAUD TO 9600 (ARRAY INDEX 5)
        self.micro_stepping_list.setCurrentIndex(self.robotArm.microSteps.index('8'))
        self.serialComms.microStep = self.robotArm.microSteps.index('8')

    def changeMicroStepping(self,item):  
        self.robotArm.microStep
        self.robotArm.xStepDeg = 200 * int(item) * self.robotArm.xGearRatio
    
    def homeRobot(self):
        self.serial_terminal.appendPlainText('Arm set to home position')
        self.robotArm.xPos = 0.0
        self.robotArm.yPos = 0.0
        self.robotArm.zPos = 0.0
        self.robotArm.armAngle = 0.0
        self.sequenceEntryUpdate()
        
    def homeGripper(self):
        self.serial_terminal.appendPlainText('End effector set to home position')
        self.robotArm.endeffAngle = 0.0
        self.robotArm.endeffGripper = 0.0
        self.sequenceEntryUpdate()
        
    def incrementXPos(self):
        self.robotArm.xPos += self.robotArm.armJogStep
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def decrementXPos(self):
        self.robotArm.xPos -= self.robotArm.armJogStep
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def incrementYPos(self):
        self.robotArm.yPos += self.robotArm.armJogStep
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def decrementYPos(self):
        self.robotArm.yPos -= self.robotArm.armJogStep
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def incrementZPos(self):
        self.robotArm.zPos += self.robotArm.armJogStep
        self.sequenceEntryUpdate()

    def decrementZPos(self):
        self.robotArm.zPos -= self.robotArm.armJogStep
        self.sequenceEntryUpdate()

    def incrementArmAngle(self):
        self.robotArm.armAngle += self.robotArm.armAngleStep
        if self.robotArm.armAngle > 120:
            self.robotArm.armAngle = 120
            self.serial_terminal.appendPlainText('Rotation angle limit reached')
        self.robotArm.calculateCurrentPosition(False)
        self.sequenceEntryUpdate()            

    def decrementArmAngle(self):
        self.robotArm.armAngle -= self.robotArm.armAngleStep
        if self.robotArm.armAngle < -120:
            self.robotArm.armAngle = -120
            self.serial_terminal.appendPlainText('Rotation angle limit reached')
        self.robotArm.calculateCurrentPosition(False)
        self.sequenceEntryUpdate() 
		
    def jogStepArm(self):
        self.robotArm.armJogStep = self.arm_jog_step.value()
        self.arm_jog_step_show.setText(str(self.robotArm.armJogStep))

    def angleStepArm(self):
        self.robotArm.armAngleStep = self.arm_angle_step.value()
        self.arm_angle_step_show.setText(str(self.robotArm.armAngleStep))
	
    ############################
    ### END EFFECTOR CONTROL ###
    ############################
    def incrementEffectorAngle(self):
        self.robotArm.endeffAngle += self.robotArm.endeffAngleStep
        self.sequenceEntryUpdate()

    def decrementEffectorAngle(self):
        self.robotArm.endeffAngle -= self.robotArm.endeffAngleStep
        self.sequenceEntryUpdate()

    def incrementGripper(self):
        self.robotArm.endeffGripper += self.robotArm.endeffGripperStep
        self.sequenceEntryUpdate()

    def decrementGripper(self):
        self.robotArm.endeffGripper -= self.robotArm.endeffGripperStep
        self.sequenceEntryUpdate()
		
    def effectorAngleStep(self):
        self.robotArm.endeffAngleStep = self.endeff_angle_step.value()
        self.endeff_angle_step_show.setText(str(self.robotArm.endeffAngleStep))

    def effectorGripperStep(self):
        self.robotArm.endeffGripperStep = self.endeff_gripper_step.value()
        self.endeff_gripper_step_show.setText(str(self.robotArm.endeffGripperStep))  
        
    #############################
    ### MANUAL POSITION ENTRY ###
    #############################

    def xPosEntered(self):
        self.robotArm.xPos = float(self.x_show.text())
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def yPosEntered(self):
        self.robotArm.yPos = float(self.y_show.text())
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def zPosEntered(self):
        self.robotArm.zPos = float(self.z_show.text())
        self.robotArm.calculateCurrentPosition(True)
        self.sequenceEntryUpdate()

    def armRotationEntered(self):
        self.robotArm.armAngle = float(self.arm_angle_show.text())
        self.robotArm.calculateCurrentPosition(False)
        self.sequenceEntryUpdate()

    def endeffRotationEntered(self):
        self.robotArm.endeffAngle = int(self.endeff_angle_show.text())
        self.sequenceEntryUpdate()

    def gripperPosEntered(self):
        self.robotArm.endeffGripper = int(self.endeff_gripper_show.text())
        self.sequenceEntryUpdate()

    #############################
    ###### OTHER FUNCTIONS ######
    #############################

    # FORMS STRING FROM XYZ POSITIONAL DATA AND DISPLAYS IT ABOVE THE SEQUENCE LIST
    # UPDATES ON-SCREEN POSITIONAL DATA TO CURRENT POSITION 
    def sequenceEntryUpdate(self): 
        # FORM STRING FROM POSITIONAL DATA
        self.robotArm.currentPosition = 'X'+str(format(self.robotArm.xPos,'.3f'))\
        +' '+'Y'+str(format(self.robotArm.yPos,'.3f'))\
        +' '+'Z'+str(format(self.robotArm.zPos,'.3f'))\
        +' '+'R' + str(format(self.robotArm.endeffAngle,'.3f'))\
        +' '+'G' + str(format(self.robotArm.endeffGripper,'.3f'))\
        +' '+'S' + str(self.robotArm.sequenceSpeed)\
        +' '+'L' + str(self.robotArm.linearMotion) + '\n'

        # SET LINEAR MOTION TOGGLE TO CORRECT STATE
        if self.robotArm.linearMotion == 0:
            self.linear_motion.setChecked(False)
        else:
            self.linear_motion.setChecked(True)

        # DISPLAY STRING IN FILE FORMAT
        self.sequence_edit.setText(str(self.robotArm.currentPosition))
        # CALCULATE ROTATION FROM XYZ COORDINATES
        self.robotArm.calculateCurrentPosition(True)
        # UPDATE POSITIONAL DATA
        self.positionUpdate()

    # RELOADS THE SEQUENCE LIST USING ELEMENTS IN THE sequenceElements ARRAY 
    # IF ARM IS ENABLED, CURRENT POSITION WILL ALSO BE UPDATED TO SELECTED LIST ELEMENT
    def sequenceListUpdate(self):
        # DISPLAYS SEQUENCE ITEMS FROM ARRAY IN THE LIST
        self.sequence_show.clear()
        for sequenceLine in self.robotArm.sequenceElements:
            sequenceLine = sequenceLine.replace("\n", "")
            item = QtWidgets.QListWidgetItem(sequenceLine)
            self.sequence_show.addItem(item)   
        
        # DISPLAY UPDATED LIST
        self.sequence_show.show()
        self.sequence_show.setCurrentRow(self.robotArm.sequenceSelected)

        # UPDATE CURRENT ROBOT POSITION TO SELECTED LIST ITEM IF ARM IS ENABLED 
        if self.robotArm.sequenceArmEnable == True:   
            # EXTRACT XYZ COORDINATES FROM SEQUENCE FILE
            self.sequenceConverter()
            # DISPLAY POSITION
            self.sequenceEntryUpdate()

    # CONVERTS SEQUENCE ELEMENT STRING TO DISCRETE XYZ COORDINATES
    def sequenceConverter(self):
        # TEMPORARILY USING CURRENT POSITION TO STORE THE SEQUENCE STRING
        self.robotArm.currentPosition = self.robotArm.sequenceElements[self.robotArm.sequenceSelected]         
            
        # REMOVE NON-NUMERICAL CHARACTERS TO EXTRACT JUST THE COORDINATES
        for letter in 'XYZRGSL':
            self.robotArm.currentPosition = self.robotArm.currentPosition.replace(letter, '')
        
        # EXTRACT EACH FLOAT IN THE STRING INTO A TEMPORARY ARRAY
        tempElements = []
        for word in self.robotArm.currentPosition.split():
            try:
                tempElements.append(float(word))
            except ValueError:
                pass

        # ASSIGN EXTRACTED FLOATS TO CORRESPONSING POSITION VARIABLES
        self.robotArm.xPos = tempElements[0]
        self.robotArm.yPos = tempElements[1]
        self.robotArm.zPos = tempElements[2]
        self.robotArm.endeffAngle = tempElements[3]
        self.robotArm.endeffGripper = tempElements[4]
        self.robotArm.sequenceSpeed = tempElements[5]
        self.robotArm.linearMotion = int(tempElements[6])

    # UPDATES ALL POSITION DATA TO CURRENT POSITION
    # PERFORMS INVERSE KINEMATICS TO DETERMINE ARM ANGLES
    def positionUpdate(self): 
        # UPDATE POSITIONAL DATA
        self.x_show.setText(str(format(self.robotArm.xPos,'.3f')))
        self.y_show.setText(str(format(self.robotArm.yPos,'.3f')))
        self.z_show.setText(str(format(self.robotArm.zPos,'.3f')))
        self.arm_angle_show.setText(str(format(self.robotArm.armAngle,'.3f')))
        self.endeff_angle_show.setText(str(format(self.robotArm.endeffAngle,'.3f')))
        self.endeff_gripper_show.setText(str(format(self.robotArm.endeffGripper,'.3f')))
        self.sequence_speed_show.setText(str(self.robotArm.sequenceSpeed))
        self.sequence_speed.setValue(self.robotArm.sequenceSpeed)

        # UPDATE KINEMATIC ANGLES
        self.robotArm.calculateKinematics()
        self.kinematics_lower_show.setText(format(self.robotArm.lowerArmAngle,'.3f'))
        self.kinematics_upper_show.setText(format(self.robotArm.upperArmAngle,'.3f'))
        self.kinematics_rotation_show.setText(format(self.robotArm.rotationAngle,'.3f'))

        if self.robotArm.playbackStatus == False:
        # UPDATE JOINT POSITION AND ARM SIMULATION GRAPH
            self.robotArm.calculateJointCoordinates()
            self.updateGraph(False)

        # SEND GCODE TO ARM CONTROLLER
        if self.robotArm.commsStatus == True:
            self.comms.write(self.robotArm.outputGCode.encode('ascii'))
            self.serial_terminal.appendPlainText(self.robotArm.outputGCode.replace('\r\n',''))

    # UPDATES SEQUENCE PROGRESS BAR DEPENDING ON WHICH LIST ELEMENT IS SELECTED
    def calculateSequenceProgress(self):
        self.robotArm.sequenceLength = len(self.robotArm.sequenceElements)
        if self.robotArm.sequenceLength > 1:
            # CALCULATE PROGRESS PERCENTAGE STEP PER SEQUENCE ITEM
            self.robotArm.sequenceProgressStep = 100/(self.robotArm.sequenceLength - 1)
            # DISPLAY CURRENT SEQUENCE PROGRESS
            self.sequence_progress.setValue(self.robotArm.sequenceSelected * self.robotArm.sequenceProgressStep)
        else:
            self.sequence_progress.setValue(0)

    # HANDLES INITIAL GRAPH SETUP AS WELL AS REFRESHES
    def updateGraph(self,state):
        # INITIAL GRAPH SETUP
        if state == True:
            # CREATE FIGURE
            self.figure = plt.figure()
            # CREATE CANVAS TO PLOT GRAPH ON
            self.plotWidget = FigureCanvas(self.figure)
            # DEFINE A LAYOUT AND ADD LAYOUT TO GUI WIDGET
            layout = QtWidgets.QVBoxLayout(self.simulation_graph)  
            layout.setContentsMargins(0, 0, 0, 0) 
            # DEFINE FIGURE AS 3D
            self.axes = self.figure.add_subplot(111, projection='3d')
            
            # ADD GRAPH TO CANVAS    
            layout.addWidget(self.plotWidget)

        # REFRESH ARM SIMULATION GRAPH  
        self.axes.clear()
        self.axes.set_xlabel('X Axis')
        self.axes.set_ylabel('Y Axis')
        self.axes.set_zlabel('Z Axis')
        # MAKES SURE EACH AXIS IS EQUAL TO PREVENT ARM DIMENSIONS LOOKING DISTORTED
        self.axes.set_xlim([-250-self.robotArm.baseHeight,250+self.robotArm.baseHeight])
        self.axes.set_ylim([-self.robotArm.baseHeight,450]) 
        self.axes.set_zlim([-self.robotArm.baseHeight,450])
 
        # PLOT BASE HEIGHT
        self.axes.plot([0,0], [0,0],[0,-self.robotArm.baseHeight]\
            , marker = 'o', markersize = 10, markerfacecolor = 'b', markeredgecolor = 'b', color = 'r', linewidth = 5)
        self.axes.plot([0,self.robotArm.xLowerJoint], [0,self.robotArm.yLowerJoint],[0,self.robotArm.zLowerJoint]\
            , marker = 'o', markersize = 10, markerfacecolor = 'b', markeredgecolor = 'b', color = 'r', linewidth = 5)
        # PLOT UPPER ARM
        self.axes.plot([self.robotArm.xLowerJoint,self.robotArm.xUpperJoint], [self.robotArm.yLowerJoint,self.robotArm.yUpperJoint],[self.robotArm.zLowerJoint,self.robotArm.zUpperJoint]\
            , marker = 'o', markersize = 10, markerfacecolor = 'b', markeredgecolor = 'b', color = 'r', linewidth = 5)
        
        self.plotWidget.draw()
    
    # WAITS IN WHILE LOOP FOR SPECIFIED AMOUNT OF TIME
    def timeDelay(self, time):
        startTime = datetime.datetime.now()
        currentTime = 0
        timeDifference = 0
        # LOOPS UNTIL SPECIFIED TIME DELAY HAS BEEN REACHED SINCE LAST MEASUREMENT
        while (timeDifference < float(time)):
            # GETTING CURRENT TIME
            currentTime = datetime.datetime.now()
            # CALCULATING SECONDS ELAPSED
            timeDifference = (currentTime - startTime).total_seconds()
            # KEEP GUI REPONSIVE
            QtWidgets.QApplication.processEvents()
        self.timeDifference = 0

    # SEND ROBOT ARM TO END STOP SWITCHES TO DETERMINE ITS POSITION
    def calibrateRobot(self):
        self.serial_terminal.appendPlainText('Calibration procedure started')
        self.serial_terminal.appendPlainText('Calibration procedure complete')

    # SET UP ARM CONTROLLER TO RECIEVE POSITION COMMANDS
    def gCodeInitialSetup(self):
        self.serial_terminal.appendPlainText('Sending initial setup G-Code')
        time.sleep(5)
        print(self.comms.read().decode('ascii'))
        #self.comms.write(('$100=%s' % self.robotArm.xStepDeg).encode('ascii'))
        self.comms.write(('$100=123').encode('ascii'))
        print(self.comms.read().decode('ascii'))
        
        self.serial_terminal.appendPlainText('X axis steps per degree: %s' % self.robotArm.xStepDeg)
        
        self.comms.write(('$101=%s' % self.robotArm.yStepDeg).encode('ascii'))
        self.serial_terminal.appendPlainText('Y axis steps per degree: %s' % self.robotArm.yStepDeg)
        
        self.comms.write(('$102=%s' % self.robotArm.zStepDeg).encode('ascii'))
        self.serial_terminal.appendPlainText('Z axis steps per degree: %s' % self.robotArm.zStepDeg)
       
        self.comms.write(('$110=%s' % self.robotArm.xMaxFeedRate).encode('ascii'))
        self.serial_terminal.appendPlainText('X max feedrate: %s' % self.robotArm.xMaxFeedRate)
       
        self.comms.write(('$111=%s' % self.robotArm.yMaxFeedRate).encode('ascii'))
        self.serial_terminal.appendPlainText('Y max feedrate: %s' % self.robotArm.yMaxFeedRate)
        
        self.comms.write(('$112=%s' % self.robotArm.zMaxFeedRate).encode('ascii'))
        self.serial_terminal.appendPlainText('Z max feedrate: %s' % self.robotArm.zMaxFeedRate)
        
        
        self.serial_terminal.appendPlainText('Initial setup complete')

class RobotArm():
    ##################
    ### ATTRIBUTES ###
    ##################

    # SETUP PARAMETERS
    microSteps = ['1','2','4','8','16']
    microStep = 0
    maxSpeeds = ['1000','2000','3000','4000','5000','6000','7000','8000','9000','10000']
    maxSpeed = 0

    xGearRatio = 50.89
    yGearRatio = 50.89
    zGearRatio = 26.85

    xStepDeg = 0
    yStepDeg = 0
    zStepDeg = 0
    xMaxFeedRate = 0
    yMaxFeedRate = 0
    zMaxFeedRate = 0

    # TARGET POSITIONAL COORDINATES
    xPos = 0.0
    yPos = 0.0
    zPos = 0.0
    armAngle = 0.0
    endeffAngle = 0.0
    endeffGripper = 0.0

    # POSITIONAL COORDINATES OF JOINTS
    xLowerJoint = 0
    yLowerJoint = 0
    zLowerJoint = 0
    xUpperJoint = 0
    yUpperJoint = 0
    zUpperJoint = 0

    # INVERSE KINEMATICS VARIABLES
    lowerArmLength = 200    # LENGTH OF LOWER ARM SECTION IN MM
    upperArmLength = 200    # LENGTH OF UPPER ARM SECTION IN MM
    baseHeight = 80         # DISTANCE BETWEEN GROUND PLANE AND BASE OF LOWER ARM JOINT
    xyDistance =  0         # DISTANCE BETWEEN ORIGIN AND END EFFECTOR ON XY PLANE
    xyzDistance = 0         # DISTNACE BETWEEN ORIGIN AND END EFFECTOR ON XYZ PLANE 
    rotationAngle = 0       # BASE ROTATION ANGLE
    angleA = 0              # ANGLE BETWEEN xyzDistance AND XY PLANE
    angleB = 0              # ANGLE BETWEEN xyzDistance AND lowerArmLength
    angleC = 0              # ANGLE BETWEEN lowerArmLength and upperArmLength
    lowerArmAngle = 0       # FINAL LOWER ARM ANGLE
    upperArmAngle = 0       # FINAL UPPER ARM ANGLE
 
    # JOG CONTROL (INTIAL VALUES)
    armJogStep = 10
    armAngleStep = 10
    endeffAngleStep = 10
    endeffGripperStep = 100
    sequenceSpeed = 50

    # OTHER PARAMETERS
    commsStatus = False
    fileStatus = False
    sequenceRepeat = False
    sequenceArmEnable = False
    linearMotion = 0
    availableComPorts = []
    comPort = 0
    outputGCode = ''
    gcodeResponse = []
    
    # FILE DATA
    sequenceProgressStep = 0.0
    fileName = ''
    sequenceElements = []
    currentPosition = ''
    sequenceSelected = 0
    sequenceLength = 0
    playbackStatus = False

    # USES INVERSE KINEMATIC EQUATIONS TO CALCULATE THE ARM ANGLES FOR A DESIRED XYZ POSITION
    def calculateKinematics(self):
        self.xyDistance = math.sqrt(self.xPos**2 + (self.upperArmLength + self.yPos)**2)         
        self.xyzDistance = math.sqrt(self.xyDistance**2 + (self.lowerArmLength + self.zPos)**2)
        self.rotationAngle = (180/math.pi) * math.asin(self.xPos/self.xyDistance)

        # TEMPORARY ANGLE CALCULATIONS
        # FIND ANGLE BETWEEN XY PLANE AND xyzDistance
        self.angleA = (180/math.pi) * math.atan((self.lowerArmLength + self.zPos)/self.xyDistance)
        # USE COSINE RULE TO FIND ANGLE BETWEEN lowerArmLength AND xyzDistance
        self.angleB = (180/math.pi) * math.acos((self.lowerArmLength**2 + self.xyzDistance**2 - self.upperArmLength**2)/(2 * self.lowerArmLength * self.xyzDistance))          
        # USE COSINE RULE TO FIND ANGLE BETWEEN upperArmLength AND lowerArmLength
        self.angleC = (180/math.pi) * math.acos((self.upperArmLength**2 + self.lowerArmLength**2 - self.xyzDistance**2)/(2 * self.upperArmLength * self.lowerArmLength))          
    
        # FINAL ARM ANGLE CALCULATIONS
        # CALCULATE ANGLE BETWEEN lowerArmLength AND THE VERTICAL PLANE
        self.lowerArmAngle = 90 - self.angleB - self.angleA   
        # CALCULATE ANGLE BETWEEN upperArmLength AND THE HORIZONAL PLANE 
        self.upperArmAngle = 90 + self.lowerArmAngle - self.angleC
        
        # FORM GCODE STRING TO BE SENT TO ARM CONTROLLER BOARD
        self.outputGCode = 'G01 ' + 'X' + str(format(self.lowerArmAngle,'.3f')) + \
        ' Y' + str(format(self.upperArmAngle,'.3f')) + \
        ' Z' + str(format(self.rotationAngle,'.3f')) + \
        ' F' + str(100*int(self.sequenceSpeed)) + '\r\n'

    # CALCULATES XYZ POSITION JOINTS
    def calculateJointCoordinates(self):
        xyLength = self.lowerArmLength * math.sin(math.radians(self.lowerArmAngle))
        self.xLowerJoint = xyLength * math.sin(math.radians(self.rotationAngle))
        self.yLowerJoint = xyLength * math.cos(math.radians(self.rotationAngle))
        self.zLowerJoint = self.lowerArmLength * math.cos(math.radians(self.lowerArmAngle))

        xyLength = self.upperArmLength * math.cos(math.radians(self.upperArmAngle))
        self.xUpperJoint = self.xLowerJoint + (xyLength * math.sin(math.radians(self.rotationAngle)))
        self.yUpperJoint = self.yLowerJoint + (xyLength * math.cos(math.radians(self.rotationAngle)))
        self.zUpperJoint = self.zLowerJoint - (self.upperArmLength * math.sin(math.radians(self.upperArmAngle))) 

    # CALCULATES NEW XYZ POSITION WHEN USER MOVES ARM VIA BASE ROTATION
    # SIMILARLY, CALCULATES BASE ROTATION WHEN USER MOVES ARM VIA XYZ POSITION
    def calculateCurrentPosition(self,state):
        # WHEN USER USES XYZ POSITION
        self.xyDistance = math.sqrt(self.xPos**2 + (self.upperArmLength + self.yPos)**2)
        if state == True:
            self.armAngle = (180/math.pi) * math.asin(self.xPos/self.xyDistance)
        
        # WHEN USER USES ROTATION POSITION
        if state == False: 
            self.xPos = self.xyDistance * math.sin((math.pi/180) * self.armAngle)
            self.yPos = self.xyDistance * math.cos((math.pi/180) * self.armAngle) - self.upperArmLength

# OPENS GUI WINDOW
def gui_initiate():
    #if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
    #    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    #if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
    #    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = Ui()
    app.exec_()

# MAIN PROGRAM
def main():
    gui_initiate()

if __name__ == "__main__":
    main()

    
    
    





