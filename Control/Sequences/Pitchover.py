import math

def checkGimbleStatus(i, THETAP, noOfNoz, pitchTime, isGimCom, iniNoOfNoz, H):    
    if i >= pitchTime and isGimCom == False:
        THETAP = 0
        isGimCom = True
        noOfNoz = iniNoOfNoz
        print("Gimbling comepleted at: " + str(H))
        # switchH = H        
    return THETAP, isGimCom, noOfNoz
    
        
def startSecondPitchover(i, H, pitch2H, isGimCom, pitchCount, pitchTime, pitchTime2, THETAP, THETAP2, isGravTurnCont):         
    if H >= pitch2H and isGimCom == True and pitchCount == 1:
        pitchTime = pitchTime2
        THETAP = THETAP2
        pitchCount = 2
        isGimCom = False
        i = 0
        isGravTurnCont = 0
        
    return pitchTime, THETAP, pitchCount, isGimCom, i, isGravTurnCont
        
def updatePitchTime(i, pitchTime, isMaxQ, timeStep):        
    if i <= pitchTime and isMaxQ == 1:
        isGimbling  = 1
        i = i + (1/timeStep)
    else:
        isGimbling = 0
 
    return i, isGimbling