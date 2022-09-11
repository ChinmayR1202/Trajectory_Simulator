# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import sys
from calculators_2 import *

R0 = 6378140
decBurnAlt = []
decBurnTime = []
decBurnGimbAngle = []
isIniSlowCom = 1
isBurnCom = []
decNoz = []

wingOpenAlt = []
wingOpenTime = []
wingsLiftCoeff = []
wingRocTilt = []
isWingOpenDone = []

isSetUpDone = 0

paraDep = 0
paraVelInput = 0
j = 0
k = 0
l = 0
m = 0
wingsInput = 0
wingOpen = 0
wingsOpenNum = 0

time1 = 0
maxHorH = 0
maxRocVel = 0
maxVerH = 0
maxHorVel = 0
horH = 0
verVel = 0
xH = []
yH = []
rcTi = []
hV = []
vV = []
tP = []
nZ = []
rF2 = []
rsAg = []
aoa = []
fdf = []
rcV = []
atp = []
atd = []

mFDec = float(input("What is the amount of fuel being used for descent?: "))
H = float(input("At what altitude would you want to perform descent?: "))
horVel = float(input("What is the horizontal velocity of the rocket?: ")) / 1000
mCur = float(input("What is the mass of the rocket with the fuel?: "))
launchAzi = float(input("What is the corrected azimuth of your desired trajectory?: "))
thetaL = float(input("Please enter launch latitude angle: "))
H0 = float(input("Please enter launch altitude: "))
mDot1 = float(input("Please enter Mass Flowrate: "))/1000

frontDragCoeff = float(input("What is the frontal drag coefficient of your rocket? 0.72 is typical: "))
frontSurfArea = float(input("What is the frontal surface area of your rocket?: ")	)
sideDragCoeff = float(input("Please enter the coefficient of drag of the rocket's side: "))
totalSurfArea = float(input("What is the total surface area of your rocket?: "))
maxSideForce = float(input("What is the maximum allowable side drag force?: "))

isAerospike = float(input("Do you want an aerospike engine throughout?: "))       
if isAerospike == 1:
    thrustIni = float(input("What is the thrust at sea level?: "))/1000**2
    thrustFin = float(input("What is the thrust in vacuum?: "))/1000**2
else:
    exitP = float(input ("What is the exit pressure of your thruster?: "))/(1000**2)
    exitArea = float(input("What is the exit area of your thruster?: ")	)
    vExType = float(input("Do you want a static exhaust velocity (1) or a varying (2)? (1 if aerospike): "))
    if vExType == 1:
        Ve = float(input("Please enter exhaust velocity of 1 thruster(0 if aerospike): "))/1000
    elif  vExType == 2:
        vExIni = float(input("What is the exhaust velocity at sea level?: "))/1000
        vExFin = float(input("What is the exhaust velocity in vacuum?: "))/1000
    
# before deorbit burn
backForce = float(input("What is the force at which the kick stage seperates at?: "))
forceTime = float(input("How long does this force last for?: "))
coldGasNo = float(input("How many cold gas thrusters are going to be used for rotation of the LV?: "))
coldGasThrust = float(input("What is the thrust proovided by the cold gas thrusters?: "))/1000**2
coldGasThrustDep = float(input("What is the reduction rate for the thrust produced by the cold gas thrusters?: "))/1000**2
distToCom = float(input("What is the distance from the cold gas thrusters to the center of mass?: "))
rotStartTime = float(input("What is the time the rotation should start?: "))
isRotStart = 0
isRotDone = 0

iniSlowTime = float(input("How long would you like to do the initial slow for?: "))
iniNoz = float(input("How many engines should work during this time?: "))
noDecBurn = float(input("How many burns would you like to do during descent?: "))
isDepPara = float(input("Would you like to deploy a parachute (1) ?: "))

if isDepPara == 1:  
    depMeth = float(input("Would you like to deply parachute at a particular Altitude (1) or a vehicle velocity (2)?: "))
    if depMeth == 1:
        depAlt = float(input("What altitude would you like to deploy parachute at?: "))
    else:
        depVel = float(input("What velocity of the rocket should the parachute be deployed at?: "))
    paraNo = float(input("How many parachutes are you deploying?: "))
    paraDragCoef = float(input("What is the drag coefficient of each parachute? (1.3 - 1.8): "))
    paraArea = float(input("What is the area of each parachute?: "))
else:
    depMeth = 0

if noDecBurn > 0:
    while j < noDecBurn:
        
        decBurn = float(input("What altitude would this burn be?: "))
        decTime = float(input("How long would be the burn time for this burn?: "))
        gimAngle = float(input("What would be the gimble angle during this burn only?: "))
        depNoz = float(input("How many nozzles should be acitve during this burn?: "))
        
        decNoz.append(depNoz)
        decBurnAlt.append(decBurn)
        decBurnTime.append(decTime)      
        decBurnGimbAngle.append(gimAngle)
        isBurnCom.append(0)
        
        j = j +  1

 
rocTilt = float(input("What is the tilt of the LV when it is about to deploy the kick stage?: "))
followVel = float(input("Do you want the tilt to follow the velocity vector?: "))

tiltVel = 0
isGimbling = 0
noOfNoz = iniNoz
gAcc = getGAcc(H, R0)/(1000**2)
mg = mCur * gAcc
dV2 = ((2*math.pi*((H0 + R0)*math.cos(math.radians(thetaL))))/86164.09) * math.sin(math.radians(launchAzi))/1000
centriForce = (mCur * (( horVel)**2)) / (H + R0) 
resAngle = getResAngle(horVel,verVel)
angOfAtt = rocTilt - resAngle
rocVel = math.sqrt((verVel**2) + (horVel**2))
sideSurfArea = (totalSurfArea)/2

    
while H > 0 and centriForce < mg:
    
    if noDecBurn > 0:
        if l <= noDecBurn - 1 and H <= decBurnAlt[l] and isBurnCom[l] == 0:      
            if m < decBurnTime[l] and paraDep == 0:
                print ("Going to part 3")
                THETAP = decBurnGimbAngle[l]
                noOfNoz = decNoz[l]
                isGimbling  = 1
                m = m + 0.001
            else:
                isGimbling = 0
        else:      
            noOfNoz = 0
            THETAP = 0
        
        
    if time1 > rotStartTime and isRotStart == 0:
        isRotStart = 1
        
    if rocTilt >= 270:
        isRotDone = 1
    
    if isRotDone == 1 and isSetUpDone == 0 :
        isIniSlowCom = 0
        isSetUpDone = 1

    if k < iniSlowTime and isIniSlowCom == 0:
        noOfNoz = iniNoz
        thetaP = 0
    elif k > iniSlowTime:
        isIniSlowCom = 1
        
    if mFDec < 0:
        noOfNoz = 0
        
       
    if isGimbling == 1:
        if resAngle > rocTilt:
            thetaP = 360 - THETAP
            thrustResAng = thetaP - (360 - rocTilt)
        else:
            thetaP = THETAP
            thrustResAng = thetaP + rocTilt     
    else:
        thetaP = 0
        thrustResAng = rocTilt    
    
    mDot = mDot1 * noOfNoz
     
    gAcc = getGAcc(H, R0)/(1000**2)
    
    atmosP, atmosD, atmostT = atmosCondition(H, gAcc)
    atmosP = atmosP/(1000**2)
    atmosD = 1.225*math.exp((-0.1354/1000)*H)
                           
    centriForce = (mCur*((horVel)**2))/(H + R0)
    mg = mCur * gAcc			# Calculating weight at current mass and altitude
    
    
        
    if angOfAtt > 90:
        angOfAtt = 360 - angOfAtt
    
    if isAerospike == 0:
        if vExType == 2:
            Ve = vExIni + ((vExFin - vExIni) - (vExFin - vExIni) * ((atmosP+0.00001)/101325)) 		      
        
    if isAerospike == 1:
        thrust = (thrustIni + ((thrustFin - thrustIni) - ((thrustFin - thrustIni) * ((atmosP * 1000**2)/101325)))) * noOfNoz
    else:
        thrust = getThrust(mDot1*1000, Ve*1000, exitP*(1000**2), atmosP*(1000**2), exitArea, noOfNoz)/1000**2
    
    frontDragForce = 0.5*atmosD*((rocVel * math.cos(math.radians(rocTilt - resAngle)))**2)*frontSurfArea * frontDragCoeff
    tiltDragForce = 0.5*atmosD*(tiltVel**2)*sideSurfArea*sideDragCoeff
    sideDragForce = 0.5*atmosD*((rocVel * math.sin(math.radians(rocTilt - resAngle)))**2)*sideSurfArea * sideDragCoeff
    # aeroShellDrag = 0.5*atmosD*((rocVel * math.cos(math.radians(rocTilt - resAngle)))**2)*25.13 * 1.65

    if sideDragForce > maxSideForce:
        sys.exit("The rocket has broken due to high side drag forces!")


    if depMeth == 1:
        if H <= depAlt:
            paraDep = 1
            depMeth = 0
    elif depMeth == 2:
        if rocVel <= depVel:
            paraDep = 1
            depMeth = 0
    else:
        paraDep = 0
        
    if time1 > forceTime:
        backForce = 0
        
    sameQuadrant = getQuadrant(thrustResAng, rocTilt)
    
    if horVel >= 0:
        horDir = -1
    else:
        horDir = 1
        
    if verVel >= 0:
        verDir = -1
    else:
        verDir = 1
        
    
    if paraDep == 1:
        paraDragForce = (0.5*atmosD*(rocVel**2)*paraArea*paraDragCoef) * paraNo 
    else:
        paraDragForce = 0

    if sameQuadrant == 1:    
        fVerRoc = (thrust*math.cos(math.radians(rocTilt - (360 - thetaP)))) - mg + abs(frontDragForce * math.cos(math.radians(rocTilt))) * verDir + centriForce + abs(sideDragForce * math.sin(math.radians(rocTilt))) * verDir - paraDragForce * math.cos(math.radians(resAngle))
        fHorRoc = (thrust*math.sin(math.radians(rocTilt - (360 - thetaP)))) + abs(frontDragForce * math.sin(math.radians(rocTilt))) * horDir + abs(sideDragForce * math.cos(math.radians(rocTilt))) * horDir                       
    else:
        fVerRoc = (thrust*math.cos(math.radians(thetaP - rocTilt))) - mg + abs(frontDragForce * math.cos(math.radians(rocTilt))) * verDir + centriForce + abs(sideDragForce * math.sin(math.radians(rocTilt))) * verDir 
        fHorRoc = (thrust*math.sin(math.radians(rocTilt - thetaP))) + abs(frontDragForce * math.sin(math.radians(rocTilt))) * horDir + abs(sideDragForce * math.cos(math.radians(rocTilt))) * horDir               

    resAngle = getResAngle(horVel,verVel)
    
    resForce = math.sqrt((fVerRoc**2)+(fHorRoc**2))
    
    if isRotStart == 1 and isRotDone == 0:
        resForce2 = coldGasThrust * coldGasNo
    else:
        resForce2 = abs(thrust*math.sin(math.radians(thetaP))) - tiltDragForce
      
    if coldGasThrust > 0:    
        coldGasThrust = coldGasThrust - (coldGasThrustDep/1000**2)
    
    tiltAcc = (resForce2)/(mCur)
    tiltVel = tiltVel + tiltAcc
    if rocTilt <= 10 and isIniSlowCom == 1 and followVel == 0:
        tiltVel = 0
        rocTilt = 0
    omega = tiltVel/(2*math.pi*distToCom)
    if isRotDone == 1 and isIniSlowCom == 0:
        omega = 0
    
    rocTilt = rocTilt + omega		# Calculating currect tilt of rocket
    
    if isRotDone == 1 and followVel == 1:
        rocTilt = 180 + resAngle

    
    if rocTilt > 360:
         rocTilt = 0
    
    if paraDep == 1:
        rocTilt = 0			
           
    if resForce <= 0: 
        print ("Your rocket is too heavy!")
     			
        
    verAcc = fVerRoc/mCur
    verVel = verVel + verAcc
            
    horAcc = fHorRoc/mCur
    horVel = horVel + horAcc
    
    if horVel < 0 and paraDep == 1:
        horVel = 0
        
    if verVel > 0 and paraDep == 1:
        verVel = 0
    
    if depMeth == 1:
        if (H + verVel) <= depAlt and paraVelInput == 0:
            paraDepVerVel = abs(verVel)
            paraDepHorVel = horVel
            paraVelInput = 1
            
    elif depMeth == 2:         
         if (H + verVel) <= depVel and paraVelInput == 0:
            paraDepVerVel = abs(verVel)
            paraDepHorVel = horVel
            paraVelInput = 1
            
    rocVel = math.sqrt((verVel**2) + (horVel**2))
    rocAcc = math.sqrt((verAcc**2) + (horAcc**2))
    rocG = rocAcc / 9.81
    			

    H = H + verVel
    horH = horH + horVel
    
    # Calculating the dynamic pressure
    q = atmosD * (rocVel*1000)**2 * 0.5
      
    if H > maxVerH:
        maxVerH = H
        
    if horH > maxHorH:
        maxHorH = horH
        	
            
    if rocVel > maxRocVel:
        maxRocVel = rocVel
    
    if horVel > maxHorVel:
        maxHorVel = horVel
     
    time1 = time1 + 0.001
    
    angOfAtt = rocTilt - resAngle
    if angOfAtt > 180 : 
        angOfAtt = 360 - angOfAtt
    
    
    mFDec = mFDec - mDot
    mCur = mCur - mDot
    
    xH.append(time1)
    yH.append(H)
    
    if k < iniSlowTime and isIniSlowCom == 0:
        k = k + 0.001
    
    if noDecBurn > 0:
        if m >= decBurnTime[l] and l < noDecBurn - 1:
            m = 0
            isBurnCom[l] = 1
    if noDecBurn > 0:
        if isBurnCom[l] == 1 and l < noDecBurn - 1:
            l = l + 1

    rcTi.append(rocTilt)
    hV.append(horVel*1000)
    vV.append(verVel*1000)
    # tP.append(THETAP)
    rF2.append(resForce2*1000**2)
    rsAg.append(resAngle)
    aoa.append(abs(rocTilt - resAngle))
    nZ.append(noOfNoz) 
    fdf.append(frontDragForce*1000**2)
    rcV.append(rocVel*1000)
    atp.append(rocAcc *(1000**2))
    atd.append(rocG*(1000**2))



    # print (time1)
    # print ("Stage: 3")
    # print ("No of Nozzle on is: ", noOfNoz)
    # print ("Thrust is: ", thrust)
    # print (atmosP)
    # print (atmosD)
    # print (resAngle)
    # print (fHorRoc)
    # print (horAcc)
    # print (horVel)
    # print (rocVel)
    # if paraDep == 1:
    #     print (paraDragForce)
    # print (fVerRoc)
    # print (verAcc)
    # print (verVel)
    # print (frontDragForce)
    # print (sideDragForce)
    # print (centriForce)
    # print (mg)
    # print ("Parachute status is:" , paraDep)
    # print (horH)
    # print (rocTilt)
    # print (H)
    # print (mFDec)
    # print (" ")


plt.plot(xH,yH)
plt.grid()
plt.show()



plt.plot(xH,rF2)
plt.grid()
plt.show()


plt.plot(xH,rsAg)
plt.grid()
plt.show()


plt.plot(xH, rsAg, 'r', xH, rcTi, 'g', xH, aoa, 'b')
plt.grid()
plt.show()


plt.plot(yH,hV)
plt.grid()
plt.show()


plt.plot(yH,rcV)
plt.grid()
plt.show()


plt.plot(yH,vV)
plt.grid()
plt.show()


plt.plot(yH,fdf)
plt.grid()
plt.show()


plt.plot(yH,atp)
plt.grid()
plt.show()


plt.plot(yH,atd)
plt.grid()
plt.show()


# plt.plot(xH,tP)
# plt.show()

plt.plot(xH,nZ)
plt.grid()
plt.show()
