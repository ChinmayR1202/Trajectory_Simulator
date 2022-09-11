import matplotlib.pyplot as plt

def PlotOutputs(xH, yH, maxHorH, maxVerH, time2, reAg, rcTi, aOA, nNoz, CD, CL, Q, MA, sDF, vVel, fDF):

    plt.plot(xH,yH)
    plt.axis([-50000, (maxHorH + 50000), 0, (maxVerH + 50000)])
    plt.show()
    
    
    plt.plot(time2,yH)
    plt.show()
    
    
    
    plt.plot(time2, reAg, 'r', time2, rcTi, 'g', time2, aOA, 'b')
    plt.show()    
    
    plt.plot(yH, reAg, 'r', yH, rcTi, 'g', yH, aOA, 'b')
    plt.show()    
    
    plt.plot(time2, nNoz) 
    plt.show()      
    
    plt.plot(time2, CD) 
    plt.show()
    
    plt.plot(time2, CL) 
    plt.show()
    
    plt.plot(MA, CD) 
    plt.show()
    
    plt.plot(yH, CL) 
    plt.show()
    
    plt.plot(yH, nNoz)
    plt.show()
     
    plt.plot(time2, Q) 
    plt.show()      
    
    plt.plot(yH, Q)
    plt.show()
    
    plt.plot(CL, Q)
    plt.show()
    
    
    plt.plot(time2,sDF)
    plt.show()
    
    plt.plot(time2,vVel)
    plt.show()
    
    plt.plot(time2,fDF)
    plt.show()
    
    plt.plot(yH,fDF)
    plt.show()
    
    return
