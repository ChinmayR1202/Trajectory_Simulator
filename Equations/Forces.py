import math

def getWeight(mCur, H, R0, timeStep):
    gAcc = (((6.67e-11 * 5.972e24))/((H + R0)**2))/ timeStep**2
    mg = mCur * gAcc
    return gAcc, mg

def getThrust(thrustIniSL, thrustFinSL, thrustIniVA, thrustFinVA, atmosP, sLNoz, vacNoz, timeStep):
    thrust = (((thrustIniSL + ((thrustFinSL - thrustIniSL) - ((thrustFinSL - thrustIniSL) * ((atmosP*timeStep**2)/101325)))) * sLNoz) + ((thrustIniVA + ((thrustFinVA - thrustIniVA) - ((thrustFinVA - thrustIniVA) * ((atmosP*timeStep**2)/101325)))) * vacNoz))
    return thrust

def getCentriForce(mCur, horVel, dV2, H, R0):
    centriForce = (mCur*((horVel+dV2)**2))/(H + R0)
    return centriForce

def getMach(rocVel, a, timeStep):
    mach = (rocVel*timeStep)/a
    return mach


def getDir(horVel, verVel):
    if horVel >= 0:
        horDir = -1
    else:
        horDir = 1
        
    if verVel >= 0:
        verDir = -1
    else:
        verDir = 1
        
    return horDir, verDir

def getDragForce(rocVel, tiltVel, rocTilt, resAngle, atmosD, atmosP, frontSurfArea, sideSurfArea, frontDragCoeff, sideDragCoeff):
    frontDragForce = 0.5*atmosD*((rocVel * math.cos(math.radians(rocTilt - resAngle)))**2)*frontSurfArea * frontDragCoeff
    tiltDragForce = 0.5*atmosD*(tiltVel**2)*sideSurfArea*sideDragCoeff
    sideDragForce = 0.5*atmosD*((rocVel * math.sin(math.radians(rocTilt - resAngle)))**2)*sideSurfArea * sideDragCoeff
    # liftForce = 0.5*atmosD*((rocVel * math.cos(math.radians(rocTilt - resAngle)))**2)*sideSurfArea * liftCoeff
    liftForce = 0
    
    return frontDragForce, tiltDragForce, sideDragForce, liftForce

def getForce(horDir, verDir, rocTilt, thetaP, frontDragForce, sideDragForce, liftForce, sameQuadrant, thrust, mg, centriForce):

    if sameQuadrant == 1:    
        fVerRoc = (thrust*math.cos(math.radians(rocTilt - (360 - thetaP)))) - mg + abs(frontDragForce * math.cos(math.radians(rocTilt))) * verDir + centriForce + abs(sideDragForce * math.sin(math.radians(rocTilt))) * verDir + abs(liftForce * math.sin(math.radians(rocTilt))) * verDir 
        fHorRoc = (thrust*math.sin(math.radians(rocTilt - (360 - thetaP)))) + abs(frontDragForce * math.sin(math.radians(rocTilt))) * horDir + abs(sideDragForce * math.cos(math.radians(rocTilt))) * horDir + abs(liftForce * math.cos(math.radians(rocTilt))) * horDir                       
    else:
        fVerRoc = (thrust*math.cos(math.radians(thetaP - rocTilt))) - mg + abs(frontDragForce * math.cos(math.radians(rocTilt))) * verDir + centriForce + abs(sideDragForce * math.sin(math.radians(rocTilt))) * verDir + abs(liftForce * math.sin(math.radians(rocTilt))) * verDir 
        fHorRoc = (thrust*math.sin(math.radians(rocTilt - thetaP))) + abs(frontDragForce * math.sin(math.radians(rocTilt))) * horDir + abs(sideDragForce * math.cos(math.radians(rocTilt))) * horDir + abs(liftForce * math.cos(math.radians(rocTilt))) * horDir         
  
    return fVerRoc, fHorRoc

def getResForce(fVerRoc, fHorRoc):
    resForce = math.sqrt((fVerRoc**2)+(fHorRoc**2))
    return resForce

def getThrustByWeigth(thrust, mg):
    t2w = thrust/mg
    return t2w