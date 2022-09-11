import math

def StartDescent(Mf, isInitiateDescent, mFDec, isRotDone, iniSlowCom):    
    ' Descent '
    if Mf < 0:
        noOfNoz = 0  
        if not isInitiateDescent:
            rotStartTimer = True
            Mf = mFDec
            distToCom = 5
            isInitiateDescent = True
        if isRotDone and not iniSlowCom:
            iniSlowCom = True

    return iniSlowCom, Mf, distToCom, isInitiateDescent, rotStartTimer, noOfNoz

def StartRotation(rotTimer, rotStartTime):    
    if rotTimer >= rotStartTime:
        rotStartTimer = False
        isRotStart = True

    return rotStartTimer, isRotStart

def CheckRotationCom(isRotStart, rocTilt, isRotDone):    
    if isRotStart and rocTilt >= 270 and not isRotDone:
        rocTilt = 270
        isRotDone = True
        
    return isRotDone, rocTilt

def StartDeorbitBurn(isRotDone, iniSlowCom, iniNoOfNoz):        
    if isRotDone and not iniSlowCom:
        noOfNoz = iniNoOfNoz
    elif iniSlowCom:
        noOfNoz = 0
           
    return noOfNoz

def DescentRoctiltCorrection(isRotDone, iniSlowCom, resAngle):
   if isRotDone and iniSlowCom:
       rocTilt = 180 + resAngle
   return rocTilt

def CorrectDetatchRocTilt(rocTilt, desRocTilt, isInitiateDescent):          
   if rocTilt >= desRocTilt and not isInitiateDescent:
       tiltVel = 0
       rocTilt = desRocTilt
       
   return tiltVel, rocTilt


def updateDescentRotTimeer(rotStartTimer, rotTimer, timeStep):
    if rotStartTimer:
        rotTimer += (1/timeStep)
    return rotTimer