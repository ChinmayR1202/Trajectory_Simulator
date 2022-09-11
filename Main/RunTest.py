# -*- coding: utf-8 -*-
from math import *
import matplotlib.pyplot as plt
import sys
import time
import os
import xlrd
import csv


# Paths
## Forces Module
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
        
        self.timeStep = timeStep
        
        self.wb = xlrd.open_workbook(inputPath)
        self.wb2 = xlrd.open_workbook(coefficientPath)
        self.sheet = self.wb.sheet_by_index(0)
        self.sheet2 = self.wb2.sheet_by_index(0)    
        self.sheetiter = 0
        
        self.rocTilt = self.sheet.cell_value(1, 65)
        self.length = self.sheet.cell_value(1, 1)
        self.mDot1 = self.sheet.cell_value(1, 2)/self.timeStep
        self.iniNoOfNoz = self.sheet.cell_value(1, 3)
        self.isAerospike = self.sheet.cell_value(1, 4)
        if self.isAerospike == 1:
            self.thrustIni = self.sheet.cell_value(1, 5)/self.timeStep**2
            self.thrustFin = self.sheet.cell_value(1, 6)/self.timeStep**2
            
        if self.isAerospike == 2:
            self.thrustIniSL = self.sheet.cell_value(1, 57)/self.timeStep**2
            self.thrustFinSL = self.sheet.cell_value(1, 58)/self.timeStep**2
            self.thrustIniVA = self.sheet.cell_value(1, 59)/self.timeStep**2
            self.thrustFinVA = self.sheet.cell_value(1, 60)/self.timeStep**2
            self.mDot1 = self.sheet.cell_value(1, 61)/self.timeStep
            self.mDot2 = self.sheet.cell_value(1, 62)/self.timeStep
            self.iniVacNoz = self.sheet.cell_value(1, 63)
            self.switchH = self.sheet.cell_value(1, 64)
            
        else:
            self.iniVacNoz = 0
            self.mDot2 = 0
            
        self.mtow = self.sheet.cell_value(1, 7)
        self.fPer = self.sheet.cell_value(1, 8)
        self.fDec = self.sheet.cell_value(1, 9)
        if self.fPer + self.fDec > 100:
            sys.exit("The total percentage of fuel is greater than 100")

        self.H0 = self.sheet.cell_value(1, 10)
        self.vExType = self.sheet.cell_value(1, 11)
        if self.vExType == 1:
            self.Ve = self.sheet.cell_value(1, 12)/self.timeStep
        elif  self.vExType == 2:
            self.vExIni = self.sheet.cell_value(1, 13)/self.timeStep
            self.vExFin = self.sheet.cell_value(1, 14)/self.timeStep
        
        self.Hf = self.sheet.cell_value(1, 15)
        
        self.launchAzi = self.sheet.cell_value(1, 16)
        self.thetaL = self.sheet.cell_value(1, 17)

        self.exitP = self.sheet.cell_value(1, 18)/self.timeStep**2
        self.exitArea = self.sheet.cell_value(1, 19)
        self.frontDragCoeff = self.sheet.cell_value(1, 20)
        self.frontSurfArea = self.sheet.cell_value(1, 21)
        self.throttleAl = self.sheet.cell_value(1, 22)
        self.rocNoz = self.sheet.cell_value(1, 23)
        self.maxQType = self.sheet.cell_value(1, 24)
        self.maxQH = self.sheet.cell_value(1, 25)

        self.THETAP = self.sheet.cell_value(1, 26)
        self.THETAP2 = self.sheet.cell_value(1, 27)
        self.pitchTime = self.sheet.cell_value(1, 28)
        self.pitchTime2 = self.sheet.cell_value(1, 29)
        self.pitch2H = self.sheet.cell_value(1, 30)
        self.distToCom = self.sheet.cell_value(1, 31)
        self.comIncRate = self.sheet.cell_value(1, 32)/self.timeStep
        self.desRocTilt = self.sheet.cell_value(1, 33)

        self.mecoType = self.sheet.cell_value(1, 34)
        self.nozMeco = self.sheet.cell_value(1, 35)

        self.secoType = self.sheet.cell_value(1, 36)

        self.reBurnInput = self.sheet.cell_value(1, 37)
                      
        self.reBurnType = self.sheet.cell_value(1, 38)
        self.nozReBurn = self.sheet.cell_value(1, 39)

        self.hReBurn = self.sheet.cell_value(1, 40)
        self.vReBurn = self.sheet.cell_value(1, 41)
        self.mFReBurn = self.sheet.cell_value(1, 42)

        self.hMeco = self.sheet.cell_value(1, 43)
        self.vMeco = self.sheet.cell_value(1, 44)
        self.mFMeco = self.sheet.cell_value(1, 45)

        self.hSeco = self.sheet.cell_value(1, 46)
        self.vSeco = self.sheet.cell_value(1, 47)
        self.mFSeco = self.sheet.cell_value(1, 48)

        self.sideDragCoeff = self.sheet.cell_value(1, 49)
        self.totalSurfArea = self.sheet.cell_value(1, 50)
        self.maxSideForce = self.sheet.cell_value(1, 51)
        self.isGravTurnCont = self.sheet.cell_value(1, 52)

        self.rotStartTime = self.sheet.cell_value(1, 53)
        self.coldGasNo = self.sheet.cell_value(1, 54)
        self.coldGasThrust = self.sheet.cell_value(1, 55)/self.timeStep**2
        self.coldGasThrustDep = self.sheet.cell_value(1, 56)/self.timeStep**2
 
        self.noOfNoz = self.iniNoOfNoz
        self.vacNoz = self.iniVacNoz    
        
        self.R0 = 6378140
        self.mew = 398600000.4
        
        self.Mf = self.mtow * (self.fPer/100)
        self.mDot = self.mDot1 * self.noOfNoz
        self.mFDec = self.mtow * (self.fDec/100)
        
        self.vOrbit = sqrt((398600000/(self.Hf + self.R0)))
        self.dV1 = self.vOrbit        
        self.orbitIncli = atan(cos(self.thetaL) * sin(self.launchAzi))
        self.dV2 = ((2*pi*((self.H0 + self.R0)*cos(radians(self.thetaL))))/86164.09) * sin(radians(self.launchAzi))/self.timeStep
   	
        print ("Delta V due to the Earth's rotation is", self.dV2)
        
        self.H = self.H0 # Initializing current altitude
        self.time1 = 0 # Initializing time variable / start time
        self.mCur = self.mtow # Initializing current mass
        
        self.isMaxQ = 0
        self.maxQ = 0
        self.q = 0.00000000001
        self.horH = 0
        
        self.resAngle = self.rocTilt
        self.pitchCount = 1
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
        self.dV3_1 = ((6.67e-11 * 5.972e24))/((self.H + self.R0)**2)/(self.timeStep**2)
         
        self.MfIni = self.mtow * (self.fPer/100)
        self.verVel = 0.00001
        self.horVel = self.verVel * sin(radians(self.rocTilt))
        self.rocVel = 0.000000001
        
        self.thetaP = 0
        self.tiltVel = 0
        self.thrustResAng = self.rocTilt
        self.omega = 0
        self.dV3_2 = 0
        self.i = 0
        self.isGimbling = 0
        self.isGimCom = False
        self.s1Time = 0
        self.s2Time = 0
        self.s3Time = 0
        self.mecoStatus = 0
        self.secoStatus = 0
        self.reBurnStatus = 0
        self.secoCom = 0
        self.isSwitchDone = False
        
        self.noOfNoz = self.iniNoOfNoz
        self.tiltDragForce = 0
        self.maxHorH = 0
        self.maxVerH = 0
        self.maxRocVel = 0
        self.maxHorVel = 0
        self.maxAoa = 0
        self.centriForce = (self.mCur*((self.horVel+self.dV2)**2))/(self.H + self.R0)
        self.gAcc = (((6.67e-11 * 5.972e24))/((self.H + self.R0)**2))/(self.timeStep**2)
        self.mg = self.mCur * self.gAcc
        
        self.sideSurfArea = (self.totalSurfArea)/2
        self.isInitiateDescent = False
        self.isRotStart = False
        self.isRotDone = False
        self.rotStartTimer = False
        self.iniSlowCom = False
        self.isStable = False
        self.rotTimer = 0
        
        
        self.doneOnce = False
        self.restartIter = 0
        
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
        while self.H > 0 and self.centriForce < self.mg and not self.isInitiateDescent:
            
            self.THETAP, self.isGimCom, self.noOfNoz = checkGimbleStatus(self.i, self.THETAP, self.noOfNoz, self.pitchTime, self.isGimCom, self.iniNoOfNoz, self.H)
            self.pitchTime, self.THETAP, self.pitchCount, self.isGimCom, self.i, self.isGravTurnCont = startSecondPitchover(self.i, self.H, self.pitch2H, self.isGimCom, self.pitchCount, self.pitchTime, self.pitchTime2, self.THETAP, self.THETAP2, self.isGravTurnCont) 
            self.i, self.isGimbling = updatePitchTime(self.i, self.pitchTime, self.isMaxQ, self.timeStep)
            
            self.mecoStatus, self.noOfNoz, self.vacNoz = getMeco(self.noOfNoz, self.vacNoz, self.mecoType, self.H, self.hMeco, self.rocVel, self.vMeco, self.nozMeco, self.mecoStatus)
            self.secoStatus, self.noOfNoz, self.vacNoz = getSeco(self.noOfNoz, self.vacNoz, self.secoType, self.H, self.hSeco, self.rocVel, self.vSeco, self.secoStatus)
            self.noOfNoz, self.vacNoz, self.mecoStatus, self.secoStatus, self.reBurnStatus = getReBurn(self.noOfNoz, self.vacNoz, self.reBurnInput, self.mecoStatus, self.secoStatus, self.reBurnStatus, self.reBurnType, self.H, self.hReBurn, self.nozReBurn, self.rocVel, self.vReBurn, self.rocTilt, self.mFReBurn, self.isAerospike, self.iniVacNoz)
            
            self.noOfNoz, self.vacNoz, self.isMaxQ, self.qtype, self.thetaP, self.thrustResAng, self.maxQ = checkMaxQ(self.isMaxQ, self.H, self.throttleAl, self.iniNoOfNoz, self.noOfNoz, self.vacNoz, self.isAerospike, self.rocNoz, self.rocTilt, self.thetaP, self.thrustResAng, self.maxQType, self.q, self.maxQ, self.qtype, self.maxQH)
            if self.qtype == 1 and check == 0:
                print(self.rocVel*self.timeStep)
            
            self.alt = checkKarmanVelocity(self.H, self.alt, self.rocVel, self.rocTilt, self.timeStep)
            self.noOfNoz, self.vacNoz, self.isSwitchDone = switchEngine(self.H, self.noOfNoz, self.switchH, self.isAerospike, self.isSwitchDone, self.vacNoz, self.iniVacNoz)
            self.thrustResAng, self.thetaP = getGimblingAngle(self.isGimbling, self.resAngle, self.rocTilt, self.THETAP)
            
            self.gAcc, self.mg = getWeight(self.mCur, self.H, self.R0, self.timeStep)
            self.mDot = updateMDot(self.mDot1, self.mDot2, self.noOfNoz, self.vacNoz)
            self.mCur, self.Mf = updateMass(self.mCur, self.Mf, self.mDot)
        
            self.atmosP, self.atmosD, self.atmosT, self.a, self.reynolds = atmosCondition(self.H, self.gAcc, self.length, self.rocVel, self.timeStep)
            self.mach = getMach(self.rocVel, self.a, self.timeStep)
            
            self.centriForce = getCentriForce(self.mCur, self.horVel, self.dV2, self.H, self.R0)
            self.thrust = getThrust(self.thrustIniSL, self.thrustFinSL, self.thrustIniVA, self.thrustFinVA, self.atmosP, self.noOfNoz, self.vacNoz, self.timeStep)
        
            self.sameQuadrant = getQuadrant(self.thrustResAng, self.rocTilt)
            self.horDir, self.verDir = getDir(self.horVel, self.verVel)
            
            self.sideDragCoeff, self.frontDragCoeff, self.sheetiter = getCDFromCSV(self.sideDragCoeff, self.frontDragCoeff, self.mach, self.sheetiter, self.sheet)
            self.frontDragForce, self.tiltDragForce, self.sideDragForce, self.liftForce = getDragForce(self.rocVel, self.tiltVel, self.rocTilt, self.resAngle, self.atmosD, self.atmosP, self.frontSurfArea, self.sideSurfArea, self.frontDragCoeff, self.sideDragCoeff)
            self.angOfAtt = CheckAoa(self.rocTilt, self.resAngle)
            
            self.fVerRoc, self.fHorRoc = getForce(self.horDir, self.verDir, self.rocTilt, self.thetaP, self.frontDragForce, self.sideDragForce, self.liftForce, self.sameQuadrant, self.thrust, self.mg, self.centriForce)
            
            self.verVel, self.horVel, self.rocVel = updateVelocity(self.fVerRoc, self.verVel, self.fHorRoc, self.horVel, self.mCur)
            
            self.resAngle = getResAngle(self.horVel,self.verVel)
            self.resForce = getResForce(self.fVerRoc, self.fHorRoc)
            
            self.torque = getTorque(self.isGimbling, self.tiltDragForce)
            self.MoOfInertia = getMomOfInertia(self.mCur, self.length)
            self.tiltAcc = getRotFromThrust(self.MoOfInertia, self.torque)
            self.rocTilt, self.omega = Rotate(self.torque, self.omega, self.rocTilt, self.mCur, self.isGimCom)
            self.rocTilt = checkAoa(self.isGimCom, self.rocTilt, self.resAngle)
            
            self.rocTilt = resetRocketTilt(self.rocTilt)
            
            self.distToCom = updateDistToCom(self.distToCom, self.comIncRate)
            self.H, self.horH = updateDistance(self.H, self.verVel, self.horH, self.horVel)
            self.maxVerH, self.maxAoa, self.maxHorH, self.maxHorVel, self.maxRocVel = GetMax(self.maxVerH, self.H, self.maxAoa, self.angOfAtt, self.maxHorH, self.horH, self.maxHorVel, self.horVel, self.maxRocVel, self.rocVel)
            
            self.angOfAtt = getAOA(self.rocTilt, self.resAngle)
            
            self.q = getDynamicPressure(self.atmosD, self.rocVel)
            
            self.time1 = updateTime(self.time1, self.timeStep)
            if self.time1 == 1:
                print("Working")
           
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
        
        PlotOutputs(self.xH, self.yH, self.maxHorH, self.maxVerH, self.time2, self.reAg, self.rcTi, self.aOA, self.nNoz, self.CD, self.CL, self.Q, self.MA, self.sDF, self.vVel, self.fDF)

    
    
