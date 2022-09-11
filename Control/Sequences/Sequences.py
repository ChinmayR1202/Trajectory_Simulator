import math    

"Check to acitvate MECO"
def getMeco(noOfNoz, vacNoz, mecoType, H, hMeco, rocVel, vMeco, nozMeco, mecoStatus):
    if mecoType == 1:	
        if H >= hMeco and mecoStatus == 0:
            noOfNoz = nozMeco
            mecoStatus = 1
            
    elif mecoType == 2:
        if rocVel >= vMeco and mecoStatus == 0:
            noOfNoz = nozMeco
            vacNoz = nozMeco
            mecoStatus = 1 
            
    return mecoStatus, noOfNoz, vacNoz

"Check to activate SECO"             
def getSeco(noOfNoz, vacNoz, secoType, H, hSeco, rocVel, vSeco, secoStatus):             
    if secoType == 1:	
        if H >= hSeco and secoStatus == 0:
            noOfNoz = 0
            vacNoz = 0
            secoStatus = 1
    
    elif secoType == 2:
        if rocVel >= vSeco and secoStatus == 0:
            noOfNoz = 0 
            mDot = 0
            secoStatus = 1 
            
    return secoStatus, noOfNoz, vacNoz

"Check to activate reburn"    
def getReBurn(noOfNoz, vacNoz, reBurnInput, mecoStatus, secoStatus, reBurnStatus, reBurnType, H, hReBurn, nozReBurn, rocVel, vReBurn, rocTilt, mFReBurn, isAerospike, iniVacNoz):            
    if  reBurnInput == 1:
        if reBurnType == 1:	
            if H >= hReBurn and reBurnStatus == 0:
                noOfNoz = nozReBurn
                # mDot = mDot1 * noOfNoz
                reBurnStatus = 1
                
        elif reBurnType == 2:
            if rocVel >= vReBurn and reBurnStatus == 0:
                noOfNoz = nozReBurn
                # mDot = mDot1 * noOfNoz
                reBurnStatus = 1
                
        elif reBurnType == 3:
            # if (Mf/MfIni)*100 >= mFReBurn and reBurnStatus == 0:
            if rocTilt >= mFReBurn and reBurnStatus == 0:
                if isAerospike == 1:
                    noOfNoz = nozReBurn
                else:
                    noOfNoz = 0
                    vacNoz = iniVacNoz
                # mDot = mDot1 * noOfNoz
                mecoStatus = 1
                secoStatus = 1
                reBurnStatus = 1
                
    return noOfNoz, vacNoz, mecoStatus, secoStatus, reBurnStatus
 
"Checks if the conditions are right to start gimbling and whether max Q has been crossed"               
def checkMaxQ(isMaxQ, H, throttleAl, iniNoOfNoz, noOfNoz, vacNoz, isAerospike, rocNoz, rocTilt, thetaP, thrustResAng, maxQType, q, maxQ, qtype, maxQH):                
    if isMaxQ == 0:
        if H <= throttleAl:
            noOfNoz = iniNoOfNoz
            vacNoz = 0
        else:
            if isAerospike == 1:
                noOfNoz = rocNoz
            elif isAerospike == 2:
                noOfNoz = rocNoz
                vacNoz = 0
        thetaP = 0
        thrustResAng = rocTilt
        if maxQType == 1:
            if q > maxQ:
                maxQ = q
            elif q <= (3*maxQ)/4 and qtype == 1: #(3*maxQ)/4
                isMaxQ = 1
                print("Gravity Turn was initiated at: ", H)
                print("Aerodynamic Pressure is: ", q)
                # noOfNoz = iniNoOfNoz   
                VacNoz = 0
            elif q < maxQ and qtype==0:
                qtype = 1
                # print(rocVel*timeStep)
                # print(H)
                # print(q*timeStep**2)
                # print(time1)
                # print(atmosD)
                # sys.exit("reached")
        else:
            if H >= maxQH:
                isMaxQ = 1
                print("Gravity Turn was initiated at: ", H)
                print("Aerodynamic Pressure is: ", q)
                noOfNoz = iniNoOfNoz
                vacNoz = 0
                
    return noOfNoz, vacNoz, isMaxQ, qtype, thetaP, thrustResAng, maxQ

"Gets the rocket velocity at 100km"
def checkKarmanVelocity(H, alt, rocVel, rocTilt, timeStep):
    if H >= 100000 and alt == 0:
        print(rocVel*timeStep)
        print(rocTilt)
        alt = 1
    
    return alt

"Checks if conditions are met to switch the engines"
def switchEngine(H, noOfNoz, switchH, isAerospike, isSwitchDone, vacNoz, iniVacNoz):    
    if isAerospike == 2 and not isSwitchDone:
        if H >= switchH:
            noOfNoz = 0
            vacNoz = iniVacNoz
            isSwitchDone = True
    return noOfNoz, vacNoz, isSwitchDone

"Gets the current gimbling angle"
def getGimblingAngle(isGimbling, resAngle, rocTilt, THETAP):    
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
        
    return thrustResAng, thetaP

"Resets the tilt of the rocket"
def resetRocketTilt(rocTilt):
   if rocTilt > 360:
       rocTilt = 0
   
   return rocTilt   
 
"Resets the thrust of the cold gas thrusters to 0 if it goes negative to stop negative rotation"    
def resetColdGasThrust(coldGasThrust):           
   if coldGasThrust <= 0:
       coldGasThrust = 0
       
   return coldGasThrust

"Stops the increase in the distance to the center of mass"
def stopCOMIncrease(distToCom):   
   if distToCom >= 27:      
       comIncRate = 0
       
   return comIncRate