def startSU2(mach, angOfAtt, atmosD, atmosP, reynolds, doneOnce, itTimer, restartIter, sheet, sheetiter, timeStep):
    import os
    import csv
    import sys
    
    
    if mach>0.3 and atmosD > 0:
        itTimer += 1
    
    if mach > 0.3 and itTimer >= 25 and (atmosP*timeStep**2) > 500:
    
        'SU2 integration start'
    
        lines = open('Test_Config.cfg', 'r').readlines()
        lines[45] = ('MACH_NUMBER = ' + str(mach) + "\n" )
        lines[47] = ('AOA = ' + str(-angOfAtt) + "\n" )
        
        
        if not doneOnce:
            lines[27] = ('TIME_MARCHING = DUAL_TIME_STEPPING-1ST_ORDER' + "\n")
            lines[30] = ('TIME_STEP = 1e-2' + "\n")
            lines[38] = ('RESTART_SOL = NO' + "\n")
            lines[34] = ('TIME_ITER = 2' + "\n")
            lines[55] = ('FREESTREAM_PRESSURE = ' + str(atmosP*timeStep**2) + "\n" )
            lines[56] = ('FREESTREAM_DENSITY = ' + str(atmosD) + "\n")
            lines[53] = ('REYNOLDS_NUMBER = ' + str(reynolds) + "\n" )
            # lines[186] = ('OUTPUT_WRT_FREQ = 1' + "\n")
            
        else:        
            if mach >=0.5 :
                lines[27] = ('TIME_MARCHING = DUAL_TIME_STEPPING-2ND_ORDER' + "\n")
            else:
                lines[27] = ('TIME_MARCHING = DUAL_TIME_STEPPING-1ST_ORDER' + "\n")
            lines[30] = ('TIME_STEP = 1e-2' + "\n")
            lines[38] = ('RESTART_SOL = YES' + "\n")
            lines[34] = ('TIME_ITER = 3' + "\n")
            lines[39] = ('RESTART_ITER = 2' + "\n" )
            lines[55] = ('FREESTREAM_PRESSURE = ' + str(atmosP*timeStep**2) + "\n" )
            lines[56] = ('FREESTREAM_DENSITY = ' + str(atmosD) + "\n")
            lines[53] = ('REYNOLDS_NUMBER = ' + str(reynolds) + "\n" )
            # lines[186] = ('OUTPUT_WRT_FREQ = 1' + "\n")
         
            
        out = open('Test_Config.cfg', 'w')
        out.writelines(lines)
        out.close()
        itTimer = 0
        
        if not doneOnce:
            os.system('cmd /k "cd c:\ & cd SU2\SU2_Tutorials\compressible_flow & cd ROCKET_WT & SU2_CFD Test_Config.cfg"')
            with open('history.csv', 'r') as result:
                values = csv.reader(result)
                values = list(values)
                cd = values[-1][8]
                cl = values[-1][9]
            os.remove("history.csv")    
            # os.remove("restart_flow_00000.dat")
            # os.remove("restart_flow_00001.dat")
            # os.rename("restart_flow_00298.dat", "restart_flow_00000.dat")
            # os.rename("restart_flow_00299.dat", "restart_flow_00001.dat")
            doneOnce = True
        else:
            os.system('cmd /k "SU2_CFD Test_Config.cfg"')
            with open('history_00002.csv', 'r') as result:
                values = csv.reader(result)
                values = list(values)
                cd = values[-1][8]
                cl = values[-1][9]
            os.remove("history_00002.csv")
            os.remove("restart_flow_00000.dat")
            # 
            # os.remove("restart_flow_00009.dat")
            os.rename("restart_flow_00001.dat", "restart_flow_00000.dat")
            os.rename("restart_flow_00002.dat", "restart_flow_00001.dat")
                
        # print("One iteration done")
        if mach >= 0.6:
            frontDragCoeff = float(cd)
            sideDragCoeff = float(cd)
            liftCoeff = float(cl)
        restartIter += 1
            
        if sideDragCoeff < 0:
            sys.exit("Negative drag coefficient detected")
    
        return cd, cl, doneOnce, restartIter
    
def getCDFromCSV(sideDragCoeff, frontDragCoeff, mach, sheetiter, sheet):
    if mach <= sheet.cell_value(sheetiter+1, 0) and sheetiter < 370815:
        cd = sheet.cell_value(sheetiter, 1)
        sheetiter += 1
        
        sideDragCoeff = cd
        frontDragCoeff = cd
    return sideDragCoeff, frontDragCoeff, sheetiter