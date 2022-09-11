import math   

def updateMDot(mDot1, mDot2, noOfNoz, vacNoz):
    mDot = (mDot1 * noOfNoz) + (mDot2 * vacNoz)			
    return mDot

def updateMass(mCur, Mf, mDot):	
    mCur = mCur - mDot
    Mf = Mf - mDot
    return mCur, Mf

def updateDistToCom(distToCom, comIncRate):
    distToCom = distToCom + comIncRate
    return distToCom


def updateVelocity(fVerRoc, verVel, fHorRoc, horVel, mCur):
    verAcc = fVerRoc/mCur
    verVel = verVel + verAcc
    
    horAcc = fHorRoc/mCur
    horVel = horVel + horAcc
    
    rocVel = math.sqrt((verVel**2) + (horVel**2))
    
    return verVel, horVel, rocVel

def updateDistance(H, verVel, horH, horVel):
    H = H + verVel
    horH = horH + horVel
    return H, horH


def GetMax(maxVerH, H, maxAoa, aoa, maxHorH, horH, maxHorVel, horVel, maxRocVel, rocVel):
    if H > maxVerH:
        maxVerH = H
    
    if aoa > maxAoa:
        maxAoa = aoa
    
    if horH > maxHorH:
        maxHorH = horH	
        
    if rocVel > maxRocVel:
        maxRocVel = rocVel
    
    if horVel > maxHorVel:
        maxHorVel = horVel
        
    return maxVerH, maxAoa, maxHorH, maxHorVel, maxRocVel


def updateColdGasThrust(coldGasThrust, coldGasThrustDep, timeStep):
    coldGasThrust = coldGasThrust - (coldGasThrustDep/timeStep)
    return coldGasThrust

def updateTime(time1, timeStep):
    time1 = time1 + (1/timeStep)
    return time1


def updateStageTimes(isGimCom, s1Time, s2Time, s3Time, timeStep, rocTilt):
    if isGimCom == False:
        s1Time = s1Time + (1/timeStep)
    elif isGimCom == True and rocTilt < 90:
        s2Time = s2Time + (1/timeStep)
    elif isGimCom == True and rocTilt >= 90:
        s3Time = s3Time + (1/timeStep)	
        
    return s1Time, s2Time, s3Time