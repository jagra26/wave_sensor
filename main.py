from machine import Pin, I2C, Timer
import ADXL345
import time
from hmc5883l import HMC5883L
import math as m
import os

# Defitions

timer = Timer()
led = Pin('LED',Pin.OUT)
i2c = I2C(1,scl=Pin(19),sda=Pin(18), freq=10000)
adx = ADXL345.ADXL345(i2c)
sensor = HMC5883L(i2c_id=0,scl=1, sda=0)
LastMillis = time.ticks_ms()

NWmax = 50
AXOFF = 0
AYOFF = 0
AZOFF = 0
Acc = [0,0]
Vel = [0,0]
VelHPF = [0,0]
Pos = [0,0]
PosHPF = [0,0]
AccHPF = [0,0]   # Aux array for Digital_High_Pass_Filter
DHPFcoef = 0.990 # Digital_High_Pass_Filter Config @ 20ms #0.993, 0.9935, 0.994
ai = 0
Hs = [0]*NWmax
Ts = [0]*NWmax
IW = 0
NW = 0
Pmax = -10000
Pmin = 10000
t_start = 0

MinWaveHth =  0.1      # Minimun Wave Heigth (mts) 
MaxWaveHth =  5.00     # Maximun Wave Heigth (mts) 
MinWavePer =  1000     # Minimun Wave Period (millisec)  
MaxWavePer =  40000    # Maximun Wave Period (millisec)  

smpleTime = True
wdataTime = False
Return = False
Start = True


#########################################
# Get Accelarometer Data
as1 = 0
am = 0

def Zero_Cross(Vi, Pi, ti, WHCorr ):
    global Return,Pmax,Pmin,t_start,Hs,Ts,NW,IW, Start
    Pmax = max(Pi[1], Pmax)
    Pmin = min(Pi[1], Pmin)
    #print(f"v0 = {Vi[0]} / v1 = {Vi[1]}")
    
    if ( ( Vi[0] <=0 ) and ( Vi[1] > 0 ) ):
        if (Start):
          Start   = False
          t_start = ti
          Pmax    = -10000
          Pmin    =  10000

        else:
           # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            wheight  = (Pmax - Pmin)*WHCorr       
            wperiod  = (ti - t_start) 
            Pmax    = -10000
            Pmin    =  10000        
            t_start = ti

            if ( ( wheight >= MinWaveHth ) and ( wheight <= MaxWaveHth ) and ( wperiod >= MinWavePer ) and ( wperiod <= MaxWavePer ) ): 
                Return = True   

                #Global Vectors
                Hs[IW]    = wheight
                Ts[IW]    = wperiod/1000
                NW+=1
                IW+=1 
                if (IW > NWmax-1):
                    IW=0

                #To terminal
                print(NW, end='')
                print("\t", end='')                                
                print(ti/1000, end='')
                print("\t", end='') 
                print(wheight, end='')    
                print("\t", end='')                
                print(wperiod)                            
    
                with open('data.csv','a') as f:
                    f.write(str(NW) + "," + str(ti) + "," + str(wheight) + "," + str(wperiod)+"\n")
    
    
def Get_Accel_data():
    global am, as1

    x = (adx.xValue - AXOFF)*0.0038
    y = (adx.yValue - AYOFF)*0.0038
    z = (adx.zValue - AZOFF)*0.0038

    #Module
    am = m.sqrt(x*x + y*y + z*z)
    as1 = abs(x) + abs(y) + abs(z)
    return [x, y, z]
  
########################################
#########################################
# GET RAW WAVE
def Get_Raw_Wave():
    global ai0, ai, AccHPF, LastMillis, VelHPF, Vel, Acc, Pos, PosHPF
    #timer.deinit()
    ai0 = ai
    
    axis = Get_Accel_data()
    #print(axis)
    #print(am)
    if (axis[0] >= 0):
        ai = am 
    else:
        ai = -am 


    #Digital_High_Pass_Filter
    AccHPF[0] = AccHPF[1]
    AccHPF[1] = DHPFcoef * ( AccHPF[1] + ai - ai0 ) 

    dt = time.ticks_ms() - LastMillis
    LastMillis = time.ticks_ms()

    #G -> to m/s2 
    Acc[0] = Acc[1] 
    Acc[1] = AccHPF[1] * 9.80665  # by Gravity

    #Integral of Acceleration ( acceleration to velocity )
    
    Vel[0] = Vel[1] 
    Vel[1] = Vel[0] + (dt/2000)*(Acc[1] + Acc[0] )
      
    #Digital_High_Pass_Filter
    VelHPF[0] = VelHPF[1]
    VelHPF[1] = DHPFcoef * (VelHPF[1] + Vel[1] - Vel[0] )  
      
    #Pntegral of Velocity ( velocity to postition )
    Pos[0] = Pos[1] 
    Pos[1] = Pos[0] + (dt/2000)*(VelHPF[1] + VelHPF[0])
      
    #Digital_High_Pass_Filter
    PosHPF[0] = PosHPF[1]
    PosHPF[1] = DHPFcoef * (PosHPF[1] + Pos[1] - Pos[0] )
    
    #Zero Down Cross
    Zero_Cross( VelHPF, PosHPF, LastMillis, 0.9)
    
    #print(str(Acc[1]) + "\t" + str(Vel[1]) + "\t" + str(VelHPF[1]) + "\t" +
    #      str(Pos[1]) + "\t" + str(PosHPF[1]) + "\n")
    #print(str(dt) + "\t" + str(Acc[1]) + "\t" + str(VelHPF[1]) + "\t" + str(PosHPF[1]) + "\n")
    #timer.init()


print('\n Starting...\n')
#timer.init(mode=Timer.PERIODIC, period=50, callback=Get_Raw_Wave)
led.on()

while True:
    #global smpleTime, wdataTime

    #Main Funciton
    #if (smpleTime):
        #smpleTime = false
        #Get_Accel_data()
    time.sleep_ms(1)
    Get_Raw_Wave()
    
    # Write Data Waves
   #if (wdataTime):
        #wdataTime = false
        #WriteDataWaves() 
        #NW        = 0
        #LastWrite = Lastmillis
        #Nrest -= 1

    # x = adx.xValue 
    # y = adx.yValue
    # z = adx.zValue
    # print('The acceleration info of x, y, z are:%d,%d,%d'%(x,y,z))
    # roll,pitch = adx.RP_calculate(x,y,z)
    # print('roll=',roll)
    # print('pitch=',pitch)
    # print()
    # x, y, z = sensor.read()
    # print(sensor.format_result(x, y, z))
    # print()
    # time.sleep_ms(1000)
    




