# -*- coding: utf-8 -*-
from math import *
import matplotlib.pyplot as plt
import sys
import time
import os
import xlrd
import csv


# Setting the paths for modules

sys.path.insert(0, 'H:\Trajectory_Simulator\Equations')
sys.path.insert(1, 'H:\Trajectory_Simulator\Control\Rotation')
sys.path.insert(2, 'H:\Trajectory_Simulator\Control\Sequences')
sys.path.insert(3, 'H:\Trajectory_Simulator\Outputs')
sys.path.insert(4, 'H:\Trajectory_Simulator\External Simulation\SU2')

from Forces import *
from Rotation import *
from EnvironConditions import *
from Sequences import *
from Pitchover import *
from VariableUpdates import *
from Plot import *
from RunSU2 import *


class TrajectorySimulation:
    def __init__(self, timeStep, inputPath, coefficientPath):
        
        self.timeStep = timeStep # Setting the timestep size
        
        "Setting up to extract the drag and lift coefficient values"
        self.wb = xlrd.open_workbook(inputPath)
        self.wb2 = xlrd.open_workbook(coefficientPath)
        self.sheet = self.wb.sheet_by_index(0)
        self.sheet2 = self.wb2.sheet_by_index(0)    
        self.sheetiter = 0
        
        "Extracting the input values from the input excel sheet"
        self.rocTilt = self.sheet.cell_value(1, 65) # Tilt of the rocket
        self.length = self.sheet.cell_value(1, 1) # Length of the rocket
        self.mDot1 = self.sheet.cell_value(1, 2)/self.timeStep # Mass flow rate for 1 nozzle
        self.iniNoOfNoz = self.sheet.cell_value(1, 3) # Initial number of active nozzles
        self.isAerospike = self.sheet.cell_value(1, 4) # The type of nozzle configuration
        
        # Single Aerospike nozzle
        if self.isAerospike == 1:
            self.thrustIni = self.sheet.cell_value(1, 5)/self.timeStep**2
            self.thrustFin = self.sheet.cell_value(1, 6)/self.timeStep**2
            
        # Hybrid Aerospike and bell nozzle for different altitudes            
        if self.isAerospike == 2:
            self.thrustIniSL = self.sheet.cell_value(1, 57)/self.timeStep**2
            self.thrustFinSL = self.sheet.cell_value(1, 58)/self.timeStep**2
            self.thrustIniVA = self.sheet.cell_value(1, 59)/self.timeStep**2
            self.thrustFinVA = self.sheet.cell_value(1, 60)/self.timeStep**2
            self.mDot1 = self.sheet.cell_value(1, 61)/self.timeStep
            self.mDot2 = self.sheet.cell_value(1, 62)/self.timeStep
            self.iniVacNoz = self.sheet.cell_value(1, 63)
            self.switchH = self.sheet.cell_value(1, 64)
        
        # Pure Bell nozzle
        else:
            self.iniVacNoz = 0
            self.mDot2 = 0
            
        self.mtow = self.sheet.cell_value(1, 7) # Maximum take off weight
        self.fPer = self.sheet.cell_value(1, 8) # Percentage of total mass to be used as fuel for ascent
        self.fDec = self.sheet.cell_value(1, 9) # Percentage of total mass to be used as fuel for descent
        if self.fPer + self.fDec > 100:
            sys.exit("The total percentage of fuel is greater than 100")

        self.H0 = self.sheet.cell_value(1, 10) # Launch altitude
        self.vExType = self.sheet.cell_value(1, 11)
        if self.vExType == 1:
            self.Ve = self.sheet.cell_value(1, 12)/self.timeStep
        elif  self.vExType == 2:
            self.vExIni = self.sheet.cell_value(1, 13)/self.timeStep
            self.vExFin = self.sheet.cell_value(1, 14)/self.timeStep
        
        self.Hf = self.sheet.cell_value(1, 15) # Final seperation altitude
        
        self.launchAzi = self.sheet.cell_value(1, 16) # Launch azimuth
        self.thetaL = self.sheet.cell_value(1, 17) # Launch latitude

        self.exitP = self.sheet.cell_value(1, 18)/self.timeStep**2 # Exit pressure of the bell nozzle
        self.exitArea = self.sheet.cell_value(1, 19) # Exit area of the bell nozzle
        self.frontDragCoeff = self.sheet.cell_value(1, 20) # Drag coefficient for the front of the rocket
        self.frontSurfArea = self.sheet.cell_value(1, 21) # Surface area for the front of the rocket
        self.throttleAl = self.sheet.cell_value(1, 22) # Altitude to throttle the engine
        self.rocNoz = self.sheet.cell_value(1, 23) # Number of active nozzles after throttle
        self.maxQType = self.sheet.cell_value(1, 24) # How to start gimbling
        self.maxQH = self.sheet.cell_value(1, 25) # Altitude to start gimbling

        self.THETAP = self.sheet.cell_value(1, 26) # Gimbling angle for first gimble
        self.THETAP2 = self.sheet.cell_value(1, 27) # Gimbling angle for second gimble
        self.pitchTime = self.sheet.cell_value(1, 28) # Pitch time for first gimble
        self.pitchTime2 = self.sheet.cell_value(1, 29) # Pitch time for second gimble
        self.pitch2H = self.sheet.cell_value(1, 30) # Altitude to start second gimble
        self.distToCom = self.sheet.cell_value(1, 31) # Distance to center of mass 
        self.comIncRate = self.sheet.cell_value(1, 32)/self.timeStep # Speed at which distance to center of mass is changing
        self.desRocTilt = self.sheet.cell_value(1, 33) # Rocket tilt to separate kick stage at

        self.mecoType = self.sheet.cell_value(1, 34) # Which condition to start MECO
        self.nozMeco = self.sheet.cell_value(1, 35) # Number of active nozzles after MECO

        self.secoType = self.sheet.cell_value(1, 36) # Which condition to start SECO

        self.reBurnInput = self.sheet.cell_value(1, 37) # Whether to have a reburn or not
                      
        self.reBurnType = self.sheet.cell_value(1, 38) # Which condition to start Reburn
        self.nozReBurn = self.sheet.cell_value(1, 39)

        self.hReBurn = self.sheet.cell_value(1, 40) # Altitude to start reburn
        self.vReBurn = self.sheet.cell_value(1, 41) # Velocity to start reburn
        self.mFReBurn = self.sheet.cell_value(1, 42) # Rocket tilt angle to start reburn

        self.hMeco = self.sheet.cell_value(1, 43) # Altitude to start MECO
        self.vMeco = self.sheet.cell_value(1, 44) # Velocity to start MECO
        self.mFMeco = self.sheet.cell_value(1, 45) # Fuel mass percentage to start MECO

        self.hSeco = self.sheet.cell_value(1, 46) # Altitude to start SECO
        self.vSeco = self.sheet.cell_value(1, 47) # Velocity to start SECO
        self.mFSeco = self.sheet.cell_value(1, 48) # Fuel mass percentage to start SECO

        self.sideDragCoeff = self.sheet.cell_value(1, 49) # Drag coefficient of the rocket's side
        self.totalSurfArea = self.sheet.cell_value(1, 50) # Total surface are of the rocket
        self.maxSideForce = self.sheet.cell_value(1, 51) # Maximum amount of drag force the rocket can witstand
        self.isGravTurnCont = self.sheet.cell_value(1, 52) # Whether to forcefully keep the rocket tilt the same angle as the velocity vector

        self.rotStartTime = self.sheet.cell_value(1, 53) # Delay to start the flip over maneovre
        self.coldGasNo = self.sheet.cell_value(1, 54) # Number of cold gas thrusters on board
        self.coldGasThrust = self.sheet.cell_value(1, 55)/self.timeStep**2 # Thrust of the cold gas thrusters
        self.coldGasThrustDep = self.sheet.cell_value(1, 56)/self.timeStep**2 # Depreciation rate of the cold gas thrusters
 
        " Initializing the variables "
        self.noOfNoz = self.iniNoOfNoz # Total number of sea level nozzles
        self.vacNoz = self.iniVacNoz # Total number of vacuum nozzles
        
        self.R0 = 6378140 # Radius of the earth in meters
        self.mew = 398600000.4 # Gravitational constant of the Earth
        
        self.Mf = self.mtow * (self.fPer/100) # Total mass of the fuel
        self.mDot = self.mDot1 * self.noOfNoz # Total mass flow rate
        self.mFDec = self.mtow * (self.fDec/100) # Total mass of fuel for descent
        
        self.vOrbit = sqrt((398600000/(self.Hf + self.R0))) # Orbital velocity of final placement altitude
        self.dV1 = self.vOrbit # Delta-V required to reach the orbital velocity (First delta-V component)
        self.orbitIncli = atan(cos(self.thetaL) * sin(self.launchAzi)) # Inclination of the trajectory
        self.dV2 = ((2*pi*((self.H0 + self.R0)*cos(radians(self.thetaL))))/86164.09) * sin(radians(self.launchAzi))/self.timeStep # Velocity contribution by rotation of the earth based on launch angle and azimuth
   	
        print ("Delta V due to the Earth's rotation is", self.dV2)
        
        self.H = self.H0 # Initializing current altitude
        self.time1 = 0 # Initializing time variable / start time
        self.mCur = self.mtow # Initializing current mass
        
        self.isMaxQ = 0 # Initializing maxQ check
        self.maxQ = 0 # Setting current maximum dynamic pressure measured
        self.q = 0.00000000001 # Initializing dynamic pressure
        self.horH = 0 # Initializing downrange (horizontal distance) travelled
        
        self.resAngle = self.rocTilt # Initializing the velocity angle
        self.pitchCount = 1 # Setting the pitch count to 1
 
        " Initializing the arrays to plot the data "
        self.time2 = []
        self.x = []
        self.y = []
        self.z = []
        self.xH = []
        self.yH = []
        self.reAg = []
        self.rcTi = []
        self.thResAn = []
        self.aOA = []
        self.Q = []
        self.sDF = []
        self.fDF = []
        self.vVel = []
        self.nNoz = []
        
        "Back to variable initialization"
        self.dV3_1 = ((6.67e-11 * 5.972e24))/((self.H + self.R0)**2)/(self.timeStep**2) # 3rd component of delta-V initialized, the loss due to gravity
         
        self.MfIni = self.mtow * (self.fPer/100) # Initial mass of fueld
        self.verVel = 0.00001 # Initializing vertical velocity
        self.horVel = self.verVel * sin(radians(self.rocTilt)) # initializing horizontal velocity
        self.rocVel = 0.000000001 # Initializing the resultant (rocket) velocity
        
        self.thetaP = 0 # Setting the gimbling angle to 0
        self.tiltVel = 0 # Initializing the tilt velocity
        self.thrustResAng = self.rocTilt # Setting the resultant thrust angle to the same as the rocket tilt
        self.omega = 0 # Setting angular accelration of the rocket tilt to 0
        self.dV3_2 = 0 
        self.i = 0 # Initializing an internal timer
        self.isGimbling = 0 # Setting the isGimbling check to 0
        self.isGimCom = False # Setting the is Gimbling complete check to False
        self.s1Time = 0 # Initializing the timer for the first stage of the ascent
        self.s2Time = 0 # Initializing the timer for the second stage of the ascent
        self.s3Time = 0 # Initializing the timer for the third stage of the ascent
        self.mecoStatus = 0 # Initializing the check whether MECO has been acheived
        self.secoStatus = 0 # Initializing the check whether SECO has been acheived
        self.reBurnStatus = 0 # Initializing the check whether Reburn has been acheived
        self.isSwitchDone = False # Setting the check for whether the switch from sea level nozzles to vacuum nozzles has occured
        
        self.noOfNoz = self.iniNoOfNoz # Setting the current number of active nozzles to the initial number of nozzles
        self.tiltDragForce = 0 # Initializing the tilt drag force
        self.maxHorH = 0 # Setting the maximum downrange traveled to 0
        self.maxVerH = 0 # Setting the maximum altitude reached to 0
        self.maxRocVel = 0 # Setting the maximum rocket velocity reached to 0
        self.maxHorVel = 0 # Setting the maximum horizontal veocity reached to 0
        self.maxAoa = 0 # Setting the maximum angle of attak received to 0
        self.centriForce = (self.mCur*((self.horVel+self.dV2)**2))/(self.H + self.R0) # Calculating the first centripetal force to avoid division by 0
        self.gAcc = (((6.67e-11 * 5.972e24))/((self.H + self.R0)**2))/(self.timeStep**2) # Calculating the first acceleration due to gravity
        self.mg = self.mCur * self.gAcc # Calculating the current weight of the rocket
        self.sideSurfArea = (self.totalSurfArea)/2 # Calculating the surface area of the side
        
        
        " Initializing descent variables"
        self.isInitiateDescent = False 
        self.isRotStart = False
        self.isRotDone = False
        self.rotStartTimer = False
        self.iniSlowCom = False
        self.isStable = False
        self.rotTimer = 0
        
        self.doneOnce = False
        self.restartIter = 0
        
        " Initializing the variables for the SU2 integraton"
        self.itTimer = 0
        self.angOfAtt = 0
        self.CD = []
        self.CL = []
        self.MA = []
        
        self.alt = 0
        self.qtype = 0
        self.thrust = 0
        self.atmosD = 1.225
        self.isSwithcDone = False

    def RunSimulation(self):
        while self.H > 0 and self.centriForce < self.mg and not self.isInitiateDescent: # runs the simulation while the altitude is above ground, has not reached orbital velocity at current altitude, and has not started descent
            
            self.THETAP, self.isGimCom, self.noOfNoz = checkGimbleStatus(self.i, self.THETAP, self.noOfNoz, self.pitchTime, self.isGimCom, self.iniNoOfNoz, self.H) # Checks whether Gimbling has completed
            self.pitchTime, self.THETAP, self.pitchCount, self.isGimCom, self.i, self.isGravTurnCont = startSecondPitchover(self.i, self.H, self.pitch2H, self.isGimCom, self.pitchCount, self.pitchTime, self.pitchTime2, self.THETAP, self.THETAP2, self.isGravTurnCont)  # Starts the second pitchover if its set to do that
            self.i, self.isGimbling = updatePitchTime(self.i, self.pitchTime, self.isMaxQ, self.timeStep) # Updates the timer of the gimbling
            
            self.mecoStatus, self.noOfNoz, self.vacNoz = getMeco(self.noOfNoz, self.vacNoz, self.mecoType, self.H, self.hMeco, self.rocVel, self.vMeco, self.nozMeco, self.mecoStatus) # Checks whether to start MECO
            self.secoStatus, self.noOfNoz, self.vacNoz = getSeco(self.noOfNoz, self.vacNoz, self.secoType, self.H, self.hSeco, self.rocVel, self.vSeco, self.secoStatus) # Checks whether to start SECO
            self.noOfNoz, self.vacNoz, self.mecoStatus, self.secoStatus, self.reBurnStatus = getReBurn(self.noOfNoz, self.vacNoz, self.reBurnInput, self.mecoStatus, self.secoStatus, self.reBurnStatus, self.reBurnType, self.H, self.hReBurn, self.nozReBurn, self.rocVel, self.vReBurn, self.rocTilt, self.mFReBurn, self.isAerospike, self.iniVacNoz) # Checks whether to starts Reburn
            
            self.noOfNoz, self.vacNoz, self.isMaxQ, self.qtype, self.thetaP, self.thrustResAng, self.maxQ = checkMaxQ(self.isMaxQ, self.H, self.throttleAl, self.iniNoOfNoz, self.noOfNoz, self.vacNoz, self.isAerospike, self.rocNoz, self.rocTilt, self.thetaP, self.thrustResAng, self.maxQType, self.q, self.maxQ, self.qtype, self.maxQH) # Checks whether MaxQ has been reached and whether to start the gimbling
            if self.qtype == 1 and check == 0:
                print(self.rocVel*self.timeStep)
            
            self.alt = checkKarmanVelocity(self.H, self.alt, self.rocVel, self.rocTilt, self.timeStep) # Gives the velocity at 100km altitude
            self.noOfNoz, self.vacNoz, self.isSwitchDone = switchEngine(self.H, self.noOfNoz, self.switchH, self.isAerospike, self.isSwitchDone, self.vacNoz, self.iniVacNoz) # Switches the engine when the condition has been met
            self.thrustResAng, self.thetaP = getGimblingAngle(self.isGimbling, self.resAngle, self.rocTilt, self.THETAP) # Gets the resultant thrust angle
            
            self.gAcc, self.mg = getWeight(self.mCur, self.H, self.R0, self.timeStep) # Calculates the weight and gravitational acceleration
            self.mDot = updateMDot(self.mDot1, self.mDot2, self.noOfNoz, self.vacNoz) # Calculates the current total mass flow rate based on number and types of active nozzle
            self.mCur, self.Mf = updateMass(self.mCur, self.Mf, self.mDot) # Updates the mass of the rocket
        
            self.atmosP, self.atmosD, self.atmosT, self.a, self.reynolds = atmosCondition(self.H, self.gAcc, self.length, self.rocVel, self.timeStep) # Calculates the atmospheric pressure, temperature, density and reynolds number
            self.mach = getMach(self.rocVel, self.a, self.timeStep) # Calculates the mach number
            
            "Forces"
            self.centriForce = getCentriForce(self.mCur, self.horVel, self.dV2, self.H, self.R0) # Calculates the centripetal force 
            self.thrust = getThrust(self.thrustIniSL, self.thrustFinSL, self.thrustIniVA, self.thrustFinVA, self.atmosP, self.noOfNoz, self.vacNoz, self.timeStep) # Calculates the thrust of the rocket
        
            self.sameQuadrant = getQuadrant(self.thrustResAng, self.rocTilt) # Checks whether the velocity and tilt angle are in the same quadrants or not
            self.horDir, self.verDir = getDir(self.horVel, self.verVel) # Checks whether the direction the rocket is moving in is either positive or negative
            
            self.sideDragCoeff, self.frontDragCoeff, self.sheetiter = getCDFromCSV(self.sideDragCoeff, self.frontDragCoeff, self.mach, self.sheetiter, self.sheet) # Gets the drag coefficient from the coefficient file
            self.frontDragForce, self.tiltDragForce, self.sideDragForce, self.liftForce = getDragForce(self.rocVel, self.tiltVel, self.rocTilt, self.resAngle, self.atmosD, self.atmosP, self.frontSurfArea, self.sideSurfArea, self.frontDragCoeff, self.sideDragCoeff) # Calculates the all the drag forces
            self.angOfAtt = CheckAoa(self.rocTilt, self.resAngle) # Calculates the angel of attack
            
            self.fVerRoc, self.fHorRoc = getForce(self.horDir, self.verDir, self.rocTilt, self.thetaP, self.frontDragForce, self.sideDragForce, self.liftForce, self.sameQuadrant, self.thrust, self.mg, self.centriForce) # Calculates the resultant vertical and horizontal forces
            
            self.verVel, self.horVel, self.rocVel = updateVelocity(self.fVerRoc, self.verVel, self.fHorRoc, self.horVel, self.mCur) # Calculates the verticle and horizontal velocity
            
            self.resAngle = getResAngle(self.horVel,self.verVel) # Calculates the velocity angle
            self.resForce = getResForce(self.fVerRoc, self.fHorRoc) # Calcualtes the resultant force
            
            "Rotation"
            self.torque = getTorque(self.isGimbling, self.tiltDragForce) # Calculates the torue for rocket tilt
            self.MoOfInertia = getMomOfInertia(self.mCur, self.length) # Calculates the moment of inertia of the rocket
            self.tiltAcc = getRotFromThrust(self.MoOfInertia, self.torque) # Calculates the angular acceleration
            self.rocTilt, self.omega = Rotate(self.torque, self.omega, self.rocTilt, self.mCur, self.isGimCom) # Calculates the angular velocity
            self.rocTilt = checkAoa(self.isGimCom, self.rocTilt, self.resAngle)
            
            self.rocTilt = resetRocketTilt(self.rocTilt)
            
            self.distToCom = updateDistToCom(self.distToCom, self.comIncRate) # Gets the distance to the center of mass
            self.H, self.horH = updateDistance(self.H, self.verVel, self.horH, self.horVel) # Calculates the altitude and downrange
            self.maxVerH, self.maxAoa, self.maxHorH, self.maxHorVel, self.maxRocVel = GetMax(self.maxVerH, self.H, self.maxAoa, self.angOfAtt, self.maxHorH, self.horH, self.maxHorVel, self.horVel, self.maxRocVel, self.rocVel) # Calculates the maximum variables
            
            self.angOfAtt = getAOA(self.rocTilt, self.resAngle)
            
            self.q = getDynamicPressure(self.atmosD, self.rocVel) # Calculates the dynamic pressure
            
            self.time1 = updateTime(self.time1, self.timeStep) # Updates the time wrt to the set timestep
            if self.time1 == 1:
                print("Working")
           
            
           
            " Appends the arrays for plotting "
            self.z.append(self.H + self.R0)
            self.y.append(self.horH*sin(self.launchAzi))
            self.x.append(self.horH*cos(self.launchAzi))
            self.xH.append(self.horH)
            self.yH.append(self.H)
            self.rcTi.append(self.rocTilt)
            if self.resAngle > 180:
                self.reAg.append(360 - self.resAngle)
            else:
                self.reAg.append(self.resAngle)
            self.thResAn.append(self.thrustResAng)
        
            # aOA.append(angOfAtt)
            if abs(self.angOfAtt) < 180:
                self.aOA.append(self.angOfAtt)
            else:
                self.aOA.append(360 - abs(self.angOfAtt))
            self.fDF.append(self.omega)
            # sDF.append(horVel*self.timeStep)
            self.sDF.append(self.atmosP)
            self.Q.append(self.torque)
            # vVel.append(verVel*self.timeStep)
            self.vVel.append(self.thrust*self.timeStep**2)
            self.nNoz.append(sqrt((self.sideDragForce*self.timeStep**2)**2+(self.frontDragForce*self.timeStep**2)**2))
            self.time2.append(self.time1)
            self.CD.append(self.sideDragCoeff)
            self.CL.append(self.rocVel)
            self.MA.append(self.mach)
        
        print ("max Aoa is: ", self.maxAoa)
        print("Max H is: ", self.maxVerH)
        
        print("Max horH is: ", self.maxHorH)
           
        print ("The landing vertical velocity is: ", abs(self.verVel))
        print ("The landing rocket velocity is: ", self.rocVel)
        print ("The max velocity achieved by the rocket is: ", self.maxRocVel)
        print ("The max horizontal velocity achieved by the rocket is: ", self.horVel*self.timeStep)
        print ("The NEEDED orbital velocity required at your altitude is: ", 0.9 * sqrt((398600000/(self.H + self.R0))))
        print ("The ACTUAL orbital velocity required at your altitude is: ", sqrt((398600000/(self.H + self.R0))))
        print ("Delta V due to the Earth's rotation is", self.dV2)
        
        
        " Plots the outputs"
        PlotOutputs(self.xH, self.yH, self.maxHorH, self.maxVerH, self.time2, self.reAg, self.rcTi, self.aOA, self.nNoz, self.CD, self.CL, self.Q, self.MA, self.sDF, self.vVel, self.fDF)

    
    
