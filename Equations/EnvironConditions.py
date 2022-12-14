import math


def getDynamicPressure(atmosD, rocVel):
    q = atmosD * rocVel**2 * 0.5
    return q

def atmosCondition(altitude, gravAcc, length, rocVel, timeStep):   
    gravAcc = gravAcc * (timeStep**2)
    rocVel = rocVel * timeStep
    gamma = 1.4
    R = 286
    T0 = 288.15
    mu0 = 1.7332e-5
    
    if altitude >= 0 and altitude <= 11000:	
        atmosDe = 1.225*(288.15/(288.15+(-0.0065*(altitude))))**(1+((gravAcc*0.0286944)/(8.3144598*-0.0065)))
        atmosPr = 101325*(288.15/(288.15+(-0.0065*(altitude))))**(((gravAcc*0.0286944)/(8.3144598*-0.0065)))
        atmosTe = 288.15+(altitude*-0.0065)
		
    elif altitude > 11000 and altitude <= 20000:
        atmosDe = 0.36391*math.exp((-gravAcc*0.0286944*(altitude-11000))/(8.3144598*216.65))
        atmosPr = 22632.1*math.exp((-gravAcc*0.0286944*(altitude-11000))/(8.3144598*216.65)) 	
        atmosTe = 216.65		
			
    elif altitude > 20000 and altitude <= 32000:	
        atmosDe = 0.08803*((216.65)/(216.65+(0.001*(altitude-20000))))**(1+((gravAcc*0.0286944)/(8.3144598*0.001)))
        atmosPr = 5474.89*((216.65)/(216.65+(0.001*(altitude-20000))))**(((gravAcc*0.0286944)/(8.3144598*0.001))) 
        atmosTe = 216.65+((altitude-20000)*0.001)
								
    elif altitude > 32000 and altitude <= 47000:	
        atmosDe = 0.01322*((228.65)/(228.65+(0.0028*(altitude-32000))))**(1+((gravAcc*0.0286944)/(8.3144598*0.0028)))
        atmosPr = 868.02*((228.65)/(228.65+(0.0028*(altitude-32000))))**(((gravAcc*0.0286944)/(8.3144598*0.0028)))
        atmosTe = 228.65+((altitude-32000)*0.0028)
			
    elif altitude > 47000 and altitude <= 51000:	
        atmosDe = 0.00143*math.exp((-gravAcc*0.0286944*(altitude-47000))/(8.3144598*270.65))
        atmosPr = 110.90*math.exp((-gravAcc*0.0286944*(altitude-47000))/(8.3144598*270.65))
        atmosTe = 270.65	
        
    elif altitude > 51000 and altitude <= 71000:	
        atmosDe = 0.00086*((270.65)/(270.65+(-0.0028*(altitude-51000))))**(1+((gravAcc*0.0286944)/(8.3144598*-0.0028)))
        atmosPr = 66.94*((270.65)/(270.65+(-0.0028*(altitude-51000))))**(((gravAcc*0.0286944)/(8.3144598*-0.0028))) 
        atmosTe = 270.65+((altitude-51000)*-0.0028)	
			
    elif altitude > 71000 and altitude <= 86000:	
        atmosDe = 0.000064*((214.65)/(214.65+(-0.002*(altitude-71000))))**(1+((gravAcc*0.0286944)/(8.3144598*-0.002)))
        atmosPr = 3.96*((214.65)/(214.65+(-0.002*(altitude-71000))))**(((gravAcc*0.0286944)/(8.3144598*-0.002))) 
        atmosTe = 214.65+((altitude-71000)*-0.002)
        			
    elif altitude > 86000:	
        atmosPr = 0
        atmosDe = 0
        atmosTe = 184.65+(altitude*-0.002)
        
    if atmosTe <= 0:
        atmosTe = 0.0001
        
    a = math.sqrt(gamma * R * atmosTe)
    viscocity = mu0 * ((atmosTe / T0)**1.5) * ((T0 + 110.4) / (atmosTe + 110.4)) 
    reynolds = (atmosDe * length * rocVel)/ viscocity
    
    atmosPr = atmosPr/timeStep**2

    return atmosPr, atmosDe, atmosTe, a, reynolds

