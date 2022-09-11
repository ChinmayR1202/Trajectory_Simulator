import math

def getAOA(rocTilt, resAngle):
    aoa = abs(rocTilt - resAngle)
    return aoa

def getMomOfInertia(mCur, length):
    MoOfInertia = ((1/12) * mCur * length**2 ) + ((1/4) * mCur * 1.5**2)
    return MoOfInertia

def getRotFromThrust(MoOfInertia, torque):
    tiltAcc = torque/MoOfInertia
    return tiltAcc

def getResAngle(horVel, verVel):
    if horVel >= 0 and verVel >= 0:
        resAngle = math.degrees(math.atan(abs(horVel)/abs(verVel + 0.0000001)))
    elif horVel > 0 and verVel <= 0:
        resAngle = 90 + math.degrees(math.atan(abs(verVel)/abs(horVel + 0.0000001)))
    elif horVel < 0 and verVel < 0:
        resAngle = 180 + math.degrees(math.atan(abs(horVel)/abs(verVel + 0.0000001)))
    elif horVel <= 0 and verVel > 0:
        resAngle = 270 + math.degrees(math.atan(abs(verVel)/abs(horVel + 0.0000001)))
        
    return resAngle


def getQuadrant(ThrustResAngle, rocTilt):
    global yDirThrust, xDirThrust, yDirTilt, xDirTilt
    
    if ThrustResAngle >= 0 and ThrustResAngle <= 90:
        yDirThrust = 1
        xDirThrust = 1
    elif ThrustResAngle > 90 and ThrustResAngle <= 180:
        yDirThrust = -1
        xDirThrust = 1
    elif ThrustResAngle > 180 and ThrustResAngle <= 270:
        yDirThrust = -1
        xDirThrust = -1
    elif ThrustResAngle > 270 and ThrustResAngle <= 180:
        yDirThrust = 1
        xDirThrust = -1

    if rocTilt >= 0 and rocTilt <= 90:
        yDirTilt = 1
        xDirTilt = 1
    elif rocTilt > 90 and rocTilt <= 180:
        yDirTilt = -1
        xDirTilt = 1
    elif rocTilt > 180 and rocTilt <= 270:
        yDirTilt = -1
        xDirTilt = -1
    elif rocTilt > 270 and rocTilt <= 180:
        yDirTilt = 1
        xDirTilt = -1

    if (yDirThrust/yDirTilt) > 0 and (xDirThrust/xDirTilt) > 0:
        sameQuadrant = 1
    else:
        sameQuadrant = 0
        
    return sameQuadrant

def Rigid_body_moment(t_c, zt, clo, cla, xcg, xcp, zcp, cdo, cda, ch, alp):
	## Aerodynamic coefficient of moment equations
	# Moment due to trust at 0 AOA
	M1 = t_c*zt;                
	# Moment due to zero life coeffcient and skin friction drag             
	cmo = clo*(((xcg-xcp)/ch)*math.cos(alp) - (zcp/ch)*math.sin(alp)) + cdo*(((xcg-xcp)/ch)*math.sin(alp) + (zcp/ch)*math.cos(alp)); 
	# Momnet due life coeffcient and drag at an AOA - must be multiplied with AOA
	cma = cla*(((xcg-xcp)/ch)*math.cos(alp) - (zcp/ch)*math.sin(alp)) + cda*(((xcg-xcp)/ch)*math.sin(alp) + (zcp/ch)*math.cos(alp)); 
	return M1, cmo, cma

def Tail_moment(v, rho, clat, it, Eo, Ea, Vv, Vh):
	# Tail Maoment equations
	# The velocity magnitude around the horizontal fin needs to be determined and studied
	vt = v-0.12*v;                        
	n = (0.5*rho*(v**2))/(0.5*rho*(v**2));
	cmot = n*Vh*clat*(Eo-it);
	cmat = n*Vh*clat*(Ea-1);
	return cmot, cmat

def flight_path_correction(cmo,cma, alp, cmat, cm1, rho ,v, M1, m_c, Aw,ro,ri, cmot, ch):
	cm = cmo + cma*alp + cmot + cmat*alp + cm1; 
	## Longitudinal Moment Equations
	# Finding the moment using cm and thrust/vectoring components
	M = (cm*0.5*rho*(v**2)*Aw*ch)+ M1; 
	# Finding hte moment of inertia f(mass)
	MI = (m_c*((ro**2)-(ri**2)))/4;            
	# Transient pitch due to moment    
	gam_m = M/MI;               
	return gam_m   

def CheckAoa(rocTilt, resAngle):
    angOfAtt = rocTilt - resAngle
    if angOfAtt > 180 : 
        angOfAtt = 360 - angOfAtt
        
    return angOfAtt


def correctRocketRotation(rocVel, atmosD, thrust, angOfAtt, mCur):        
    if rocVel > 0 and atmosD > 0:
    # Variable attriibbutes for the stability correction
        t_c = thrust
        alp = angOfAtt
        v   = rocVel
        rho = atmosD
        m_c = mCur
        # Cl,cd: can be added into the variable attribute section if supplied through SU2 
        #-------------------------------------------------------------------------------------------- 
        #------------------------- Correction in flight path angle due to stability -----------------
        M1, cmo, cma      = Rigid_body_moment(t_c, zt, clo, cla, xcg, xcp, zcp, cdo, cda, ch, alp)
        cmot, cmat        = Tail_moment(v, atmosD, clat, it, Eo, Vv, Vh)
        rocTilt_correction= flight_path_correction(cmo,cma, alp, cmat, cm1, rho ,v, M1, m_c, Aw,ro,ri)
           
    else:
        rocTilt_correction = 0
        
    return rocTilt_correction

def getTorque(isGimbling, tiltDragForce):
    
    if isGimbling:
        # resForce2 = abs(thrust*math.sin(math.radians(thetaP))) - tiltDragForce
        torque = 0.0005 
    else:
        torque = -tiltDragForce
    # elif isRotStart and isRotDone == False:
    #     resForce2 = coldGasThrust * coldGasNo
    
    return torque

def Rotate(torque, omega, rocTilt, mCur, isGimCom):

    tiltAcc = torque/mCur
    omega = omega + tiltAcc
    # tiltVel = tiltVel + tiltAcc
    # omega = tiltVel/(2*math.pi*distToCom)
    
    rocTilt = rocTilt + math.degrees(omega) #+ math.degrees(rocTilt_correction)
    
    
    # if not isGimCom and rocTilt > 20:
    #     omega = 0 
        
    # if isGimbling == 0:
    #     rocTilt = resAngle
    
    return rocTilt, omega

def checkRotationDone(isRotDone):
    if isRotDone:
        omega = 0
        
    return omega

   

    # if abs(angOfAtt) > 5 and isGimCom:
def checkAoa(isGimCom, rocTilt, resAngle):
    if isGimCom:
        rocTilt = resAngle # + ((angOfAtt/abs(angOfAtt))*5)
        
    return rocTilt
    