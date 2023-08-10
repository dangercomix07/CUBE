import board
import busio
import adafruit_mpu6050
import time
import math
import digitalio
import pwmio

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

try:
    i2c0 = busio.I2C(board.GP19, board.GP18)
    mpu = adafruit_mpu6050.MPU6050(i2c0)
except:
    print("IMU failed to initialise")
    pass

try:
    #Creating BLDC objects
    bldc = pwmio.PWMOut(board.GP0, duty_cycle=2**15, frequency= 50)
    #calibrating ESCs
    bldc.duty_cycle = 1000
    time.sleep(3)
    #4840 is rest position in bidirectional mode
    bldc.duty_cycle = 4850
except:
    print("RWS initialisation failed")
    pass

#initialisations
prevtime=time.monotonic()
yaw =0
ie = 0
de = 0
eprev = 0

#PID Parameters
Kp = 2
Ki = 0  #0.001
Kd = 0

#Motor Cap Control
lcap = 4400 #Lower cap
ucap = 5300 #Upper cap
zp = 4850   #zero-point
pwmRange = 450


#GYRO-DRIFT-COMPENSATION
# Constants for calibration
CALIBRATION_TIME = 5  # Calibration time in seconds
CALIBRATION_SAMPLES = 200  # Number of samples for calibration

def calibrate_gyro():
    print("Calibrating gyro...")
    total_gyro_z = 0
    for _ in range(CALIBRATION_SAMPLES):
        total_gyro_z += mpu.gyro[2]  # Read gyro_z data
        time.sleep(0.025)  # Wait for 25ms between samples 40Hz sampling rate

    gyro_drift = total_gyro_z / CALIBRATION_SAMPLES
    print("Gyro drift:", gyro_drift)
    return gyro_drift

def apply_gyro_drift_compensation(gyro_z, drift):
    return gyro_z - drift

# Perform gyro drift calibration
gyro_drift = calibrate_gyro()

while True:
    # Read raw gyro data
    currtime = time.monotonic()
    dt = currtime - prevtime
    prevtime = currtime
    
    raw_gyro_z = mpu.gyro[2]
    # Apply gyro drift compensation
    gyro_z = apply_gyro_drift_compensation(raw_gyro_z, gyro_drift)
    
    #THRESHOLDING
    threshold = 0.009
    if(abs(gyro_z)<threshold):
        gyro_z = 0
    yaw += math.degrees(gyro_z*dt)

    #Cyclic relation (><360 resets to 0)
    while yaw>360:
        yaw = yaw - 360
    while yaw<-360:
        yaw = yaw + 360

    #Implement moving average filter here
    #print(dt) #need to be almost constant
    #print(raw_gyro_z)
    #print(gyro_z)
    print(yaw)
    
    desired_angle = 0
    #+-0.5 range in desired angle
    if(yaw>0):
        e = yaw - 0.5
    elif(yaw<0):
        e = yaw + 0.5
    else:
        e = 0
        
    #Computing integral and derivative    
    ie += e*dt
    #de = (e-eprev)/dt
    
    #PID Controller
    u = Kp*(e) + Ki*(ie) + Kd*de #Control Input
    
    if u>ucap:
        u_max =ucap
    elif u<lcap:
        u_max = lcap
        
    if abs(u)<pwmRange:
        bldc.duty_cycle = int(4850 + u)
    else:
        bldc.duty_cycle = int(4850 + u_max)
    
    eprev = e
    time.sleep(0.025) #40Hz sampling rate
    
###### Led Blink at perpendicular angles ########
    
#     if(yaw>86 and yaw<94):
#         led.value = 1
#     elif(yaw<-86 and yaw>-94):
#         led.value =1
#     else:
#         led.value = 0
        
################################################




