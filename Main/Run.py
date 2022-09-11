# -*- coding: utf-8 -*-
import math
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


class RunSimulation():
    def __init__(self, timeStep):
        self.timeStep = timeStep
        
        
        

'opening input excel file'
wb = xlrd.open_workbook("Inputs.xls")
wb2 = xlrd.open_workbook("H:\Trajectory_Simulator\External Simulation\SU2\Coefficients.xlsx")
sheet = wb.sheet_by_index(0)
sheet2 = wb2.sheet_by_index(0)    
sheetiter = 0

length = sheet.cell_value(1, 1)
mDot1 = sheet.cell_value(1, 2)/timeStep
iniNoOfNoz = sheet.cell_value(1, 3)
isAerospike = sheet.cell_value(1, 4)
if isAerospike == 1:
    thrustIni = sheet.cell_value(1, 5)/timeStep**2
    thrustFin = sheet.cell_value(1, 6)/timeStep**2
    
if isAerospike == 2:
    thrustIniSL = sheet.cell_value(1, 57)/timeStep**2
    thrustFinSL = sheet.cell_value(1, 58)/timeStep**2
    thrustIniVA = sheet.cell_value(1, 59)/timeStep**2
    thrustFinVA = sheet.cell_value(1, 60)/timeStep**2
    mDot1 = sheet.cell_value(1, 61)/timeStep
    mDot2 = sheet.cell_value(1, 62)/timeStep
    iniVacNoz = sheet.cell_value(1, 63)
    switchH = sheet.cell_value(1, 64)
    
else:
    iniVacNoz = 0
    mDot2 = 0
    
mtow = sheet.cell_value(1, 7)
fPer = sheet.cell_value(1, 8)
fDec = sheet.cell_value(1, 9)
if fPer + fDec > 100:
    sys.exit("The total percentage of fuel is greater than 100")

H0 = sheet.cell_value(1, 10)
vExType = sheet.cell_value(1, 11)
if vExType == 1:
    Ve = sheet.cell_value(1, 12)/timeStep
elif  vExType == 2:
    vExIni = sheet.cell_value(1, 13)/timeStep
    vExFin = sheet.cell_value(1, 14)/timeStep
    
noOfNoz = iniNoOfNoz
vacNoz = iniVacNoz    

Hf = sheet.cell_value(1, 15)
R0 = 6378140
mew = 398600000.4

Mf = mtow * (fPer/100)
mDot = mDot1 * noOfNoz

mFDec = mtow * (fDec/100)


# Calculating Orbital Velocity required / dV1

vOrbit = math.sqrt((398600000/(Hf + R0)))
dV1 = vOrbit
	
print ("The Orbital velcoity at that altitude is", dV1)
	
# Calculating Earth's rotation at launch latitude / dV2
# Rough estimation of Rotation Speed

launchAzi = sheet.cell_value(1, 16)
thetaL = sheet.cell_value(1, 17)
orbitIncli = math.atan(math.cos(thetaL) * math.sin(launchAzi))

dV2 = ((2*math.pi*((H0 + R0)*math.cos(math.radians(thetaL))))/86164.09) * math.sin(math.radians(launchAzi))/timeStep

   	
print ("Delta V due to the Earth's rotation is", dV2)
	

'''Calculation of launch Azimuth, latitude and inclination of the orbit'''



	
                    		# INITIAL VERTICAL ASCENT / STAGE 1
	
# Calculating LV Velocity, Time to reach and mass at Max q as well as delta V loss during this time


H = H0 # Initializing current altitude
time1 = 0 # Initializing time variable / start time
mCur = mtow # Initializing current mass

isMaxQ = 0
maxQ = 0
q = 0.00000000001
horH = 0
rocTilt = sheet.cell_value(1, 65)
resAngle = rocTilt
pitchCount = 1
time2 = []
x = []
y = []
z = []
xH = []
yH = []
reAg = []
rcTi = []
thResAn = []
aOA = []
Q = []
sDF = []
fDF = []
vVel = []
nNoz = []
dV3_1 = ((6.67e-11 * 5.972e24))/((H + R0)**2)/(timeStep**2)

exitP = sheet.cell_value(1, 18)/timeStep**2
exitArea = sheet.cell_value(1, 19)
frontDragCoeff = sheet.cell_value(1, 20)
frontSurfArea = sheet.cell_value(1, 21)
throttleAl = sheet.cell_value(1, 22)
rocNoz = sheet.cell_value(1, 23)
maxQType = sheet.cell_value(1, 24)
# if maxQType == 2:
maxQH = sheet.cell_value(1, 25)

THETAP = sheet.cell_value(1, 26)
THETAP2 = sheet.cell_value(1, 27)
pitchTime = sheet.cell_value(1, 28)
pitchTime2 = sheet.cell_value(1, 29)
pitch2H = sheet.cell_value(1, 30)
distToCom = sheet.cell_value(1, 31)
comIncRate = sheet.cell_value(1, 32)/timeStep
desRocTilt = sheet.cell_value(1, 33)

mecoType = sheet.cell_value(1, 34)
nozMeco = sheet.cell_value(1, 35)

secoType = sheet.cell_value(1, 36)

reBurn1Input = sheet.cell_value(1, 37)

# if reBurn1Input == 1:                        
reBurnType = sheet.cell_value(1, 38)
nozReBurn = sheet.cell_value(1, 39)

# if reBurnType == 1:
hReBurn = sheet.cell_value(1, 40)
# elif reBurnType == 2:
vReBurn = sheet.cell_value(1, 41)
# elif reBurnType == 3:
mFReBurn = sheet.cell_value(1, 42)



# if mecoType == 1:
hMeco = sheet.cell_value(1, 43)
# elif mecoType == 2:
vMeco = sheet.cell_value(1, 44)
# elif mecoType == 3:
mFMeco = sheet.cell_value(1, 45)


# if secoType == 1:
hSeco = sheet.cell_value(1, 46)
# elif secoType == 2:
vSeco = sheet.cell_value(1, 47)
# elif secoType == 3:
mFSeco = sheet.cell_value(1, 48)
    
    

MfIni = mtow * (fPer/100)
verVel = 0.00001
horVel = verVel * math.sin(math.radians(rocTilt))
rocVel = 0.000000001
# Velocity general 
Velocity = 0

thetaP = 0
tiltVel = 0
thrustResAng = rocTilt
omega = 0
dV3_2 = 0
i = 0
isGimbling = 0
isGimCom = False
s1Time = 0
s2Time = 0
s3Time = 0
mecoStatus = 0
secoStatus = 0
reBurnStatus = 0
secoCom = 0
isSwitchDone = False

noOfNoz = iniNoOfNoz
tiltDragForce = 0
maxHorH = 0
maxVerH = 0
maxRocVel = 0
maxHorVel = 0
maxAoa = 0
centriForce = (mCur*((horVel+dV2)**2))/(H + R0)
gAcc = (((6.67e-11 * 5.972e24))/((H + R0)**2))/(timeStep**2)
mg = mCur * gAcc


start = time.time()
sideDragCoeff = sheet.cell_value(1, 49)
totalSurfArea = sheet.cell_value(1, 50)
maxSideForce = sheet.cell_value(1, 51)
isGravTurnCont = sheet.cell_value(1, 52)
sideSurfArea = (totalSurfArea)/2

isInitiateDescent = False
isRotStart = False
isRotDone = False
rotStartTimer = False
iniSlowCom = False
isStable = False
rotTimer = 0
rotStartTime = sheet.cell_value(1, 53)
coldGasNo = sheet.cell_value(1, 54)
coldGasThrust = sheet.cell_value(1, 55)/timeStep**2
coldGasThrustDep = sheet.cell_value(1, 56)/timeStep**2

doneOnce = False
restartIter = 0

# ------------------------- Rigid_body_moment parameters ------------------------------ #
#
# Moment Variable of the rigid body - M
# Thrust magnitude                  - T_c
# Z co-ordinate of thrust vector    - zt
# zero AOA coeffcient of moment     - cmo
# coefficient of moment due to AOA  - cma
# lift coeeficient of moment        - clo 
# Lift coefficent at AOA            - cla
# X co-ordinate of cg               - xcg
# X co-ordiante of cp               - xcp
# z co-ordinatoe of cp              - zcp
# Zero AOA coefficent of drag       - cdo
# Drag at AOA                       - cda
# chord                             - ch
# AOA                               - alp
 
# t_c = Thrust
zt  = 0
clo = 0.02
cla = 0.001
xcg = 12
xcp = 15
zcp = 0
cdo = 0.01
cda = 0.01
ch  = 1
# alp = thetaP
M1  = 0
cmo = 0
cma = 0 


	

# ------------------------------tail_moment parameters -------------------------------- #
#
# Velocit of free flow at the tail    - vt
# Velocity                            - v
# density                             - rho
# Dynamic pressure ratio              - n
# Tail coefficeint of moment zero AOA - cmot
# Tail coefficent of moment at an AOA - cmat
# Tail lift coefficient at an AOA     - clat 
# Tail incidence                      - it 
# downwash angle                      - Eo
# Vertical tail parameter             - Vv
# Horizontal tail parameter           - Vh
# Tail surface area                   - St
# Body surface area                   - Aw
# Distance of cg to tail cp           - lt 
# Chord                               - ch 
St  = 6 
Aw  = 500
lt  = 18
ltz = lt
clat= 0.014
it  = 0
Eo  = 1/57.3
Ea  = 0
v   = Velocity
Vh  = (St/Aw)*(lt/ch)
Vv  = (St/Aw)*(ltz/ch)
cmot= 0
cmat= 0





# ------------------------------total_moment parameters -------------------------------- #
# AoA                                            - alp
# Moment of inertia of the body                  - MI
# Total moment coefficient                       - cm
# Total Moment                                   - M
# gam_m                                          - flight path angle correction
# Coefficent of moment due to-
# -external factors wind shear, bending etc      - cm1
# Mass as a function of time                     - m_c
# Area of the body surface                       - Aw
cmo = 0 
cma = 0
cmot= 0
cmat= 0
cm1 = 0
M1  = 0
m_c = mCur
ro  = 12
ri  = 10
	
itTimer = 0
angOfAtt = 0
CD = []
CL = []
MA = []

alt = 0
qtype = 0
thrust = 0
atmosD = 1.225
isSwithcDone = False
##
# --------------------------------------------------------------
##	
while H > 0 and centriForce < mg and not isInitiateDescent:
    
    THETAP, isGimCom, noOfNoz = checkGimbleStatus(i, THETAP, noOfNoz, pitchTime, isGimCom, iniNoOfNoz, H)
    pitchTime, THETAP, pitchCount, isGimCom, i, isGravTurnCont = startSecondPitchover(i, H, pitch2H, isGimCom, pitchCount, pitchTime, pitchTime2, THETAP, THETAP2, isGravTurnCont) 
    i, isGimbling = updatePitchTime(i, pitchTime, isMaxQ, timeStep)
    
    mecoStatus, noOfNoz, vacNoz = getMeco(noOfNoz, vacNoz, mecoType, H, hMeco, rocVel, vMeco, nozMeco, mecoStatus)
    secoStatus, noOfNoz, vacNoz = getSeco(noOfNoz, vacNoz, secoType, H, hSeco, rocVel, vSeco, secoStatus)
    noOfNoz, vacNoz, mecoStatus, secoStatus, reBurnStatus = getReBurn(noOfNoz, vacNoz, reBurn1Input, mecoStatus, secoStatus, reBurnStatus, reBurnType, H, hReBurn, nozReBurn, rocVel, vReBurn, rocTilt, mFReBurn, isAerospike, iniVacNoz)
    
    noOfNoz, vacNoz, isMaxQ, qtype, thetaP, thrustResAng, maxQ = checkMaxQ(isMaxQ, H, throttleAl, iniNoOfNoz, noOfNoz, vacNoz, isAerospike, rocNoz, rocTilt, thetaP, thrustResAng, maxQType, q, maxQ, qtype, maxQH)
    alt = checkKarmanVelocity(H, alt, rocVel, rocTilt, timeStep)
    noOfNoz, vacNoz, isSwitchDone = switchEngine(H, noOfNoz, switchH, isAerospike, isSwitchDone, vacNoz, iniVacNoz)
    thrustResAng, thetaP = getGimblingAngle(isGimbling, resAngle, rocTilt, THETAP)
    
    gAcc, mg = getWeight(mCur, H, R0, timeStep)
    mDot = updateMDot(mDot1, mDot2, noOfNoz, vacNoz)
    mCur, Mf = updateMass(mCur, Mf, mDot)

    atmosP, atmosD, atmosT, a, reynolds = atmosCondition(H, gAcc, length, rocVel, timeStep)
    mach = getMach(rocVel, a, timeStep)
    
    centriForce = getCentriForce(mCur, horVel, dV2, H, R0)
    thrust = getThrust(thrustIniSL, thrustFinSL, thrustIniVA, thrustFinVA, atmosP, noOfNoz, vacNoz, timeStep)

    sameQuadrant = getQuadrant(thrustResAng, rocTilt)
    horDir, verDir = getDir(horVel, verVel)
    
    sideDragCoeff, frontDragCoeff, sheetiter = getCDFromCSV(sideDragCoeff, frontDragCoeff, mach, sheetiter, sheet)
    frontDragForce, tiltDragForce, sideDragForce, liftForce = getDragForce(rocVel, tiltVel, rocTilt, resAngle, atmosD, atmosP, frontSurfArea, sideSurfArea, frontDragCoeff, sideDragCoeff)
    angOfAtt = CheckAoa(rocTilt, resAngle)
    
    fVerRoc, fHorRoc = getForce(horDir, verDir, rocTilt, thetaP, frontDragForce, sideDragForce, liftForce, sameQuadrant, thrust, mg, centriForce)
    
    verVel, horVel, rocVel = updateVelocity(fVerRoc, verVel, fHorRoc, horVel, mCur)
    
    resAngle = getResAngle(horVel,verVel)
    resForce = getResForce(fVerRoc, fHorRoc)
    
    torque = getTorque(isGimbling, tiltDragForce)
    MoOfInertia = getMomOfInertia(mCur, length)
    tiltAcc = getRotFromThrust(MoOfInertia, torque)
    rocTilt, omega = Rotate(torque, omega, rocTilt, mCur, isGimCom)
    rocTilt = checkAoa(isGimCom, rocTilt, resAngle)
    
    rocTilt = resetRocketTilt(rocTilt)
    
    distToCom = updateDistToCom(distToCom, comIncRate)
    H, horH = updateDistance(H, verVel, horH, horVel)
    maxVerH, maxAoa, maxHorH, maxHorVel, maxRocVel = GetMax(maxVerH, H, maxAoa, angOfAtt, maxHorH, horH, maxHorVel, horVel, maxRocVel, rocVel)
    
    angOfAtt = getAOA(rocTilt, resAngle)
    
    q = getDynamicPressure(atmosD, rocVel)
    
    time1 = updateTime(time1, timeStep)
    if time1 == 1:
        print("Working")
   
    z.append(H + R0)
    y.append(horH*math.sin(launchAzi))
    x.append(horH*math.cos(launchAzi))
    xH.append(horH)
    yH.append(H)
    rcTi.append(rocTilt)
    if resAngle > 180:
        reAg.append(360 - resAngle)
    else:
        reAg.append(resAngle)
    thResAn.append(thrustResAng)


    # aOA.append(angOfAtt)
    if abs(angOfAtt) < 180:
        aOA.append(angOfAtt)
    else:
        aOA.append(360 - abs(angOfAtt))
    fDF.append(omega)
    # sDF.append(horVel*timeStep)
    sDF.append(atmosP)
    Q.append(torque)
    # vVel.append(verVel*timeStep)
    vVel.append(thrust*timeStep**2)
    nNoz.append(math.sqrt((sideDragForce*timeStep**2)**2+(frontDragForce*timeStep**2)**2))
    time2.append(time1)
    CD.append(sideDragCoeff)
    CL.append(rocVel)
    MA.append(mach)
      



# u = np.linspace(0 , 2 * np.pi, time1)
# v = np.linspace(0 , np.pi, time1)
# ex = R0 * np.outer(np.cos(u), np.sin(v))
# ey = R0 * np.outer(np.cos(u), np.sin(v))
# ez = R0 * np.outer(np.ones(np.size(u)), np.cos(v))
# fig = plt.figure(figsize=(10,10))
# ax = fig.gca(projection='3d')

# ax.plot(x, y, z,'r')
# plt.show()

# ax.plot_surface(ex,ey,ez)
# plt.show()



# print ("Delta V due to gravity is ", dV3)

# print ("Total rocket delta V is ", (rocVel-dV2+dV3))

# print ("The respective stage times are, ", s1Time, s2Time, s3Time)

print ("max Aoa is: ", maxAoa)
print("Max H is: ", maxVerH)

print("Max horH is: ", maxHorH)
   
print ("The landing vertical velocity is: ", abs(verVel))
print ("The landing rocket velocity is: ", rocVel)
print ("The max velocity achieved by the rocket is: ", maxRocVel)
print ("The max horizontal velocity achieved by the rocket is: ", horVel*timeStep)
print ("The NEEDED orbital velocity required at your altitude is: ", 0.9 * math.sqrt((398600000/(H + R0))))
print ("The ACTUAL orbital velocity required at your altitude is: ", math.sqrt((398600000/(H + R0))))
print ("Delta V due to the Earth's rotation is", dV2)

PlotOutputs(xH, yH, maxHorH, maxVerH, time2, reAg, rcTi, aOA, nNoz, CD, CL, Q, MA, sDF, vVel, fDF)

    
    
