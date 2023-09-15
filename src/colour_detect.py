import RPi.GPIO as GPIO
import time
#set pin numbers for sensors
S2 = 1 
S3 = 2
OUT = 25
NUM_CYCLES = 10
readCount = 0
maxRead = 10
diffThreshold = 30 #difference in average frequency values 

def setup():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(OUT,GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(S2,GPIO.OUT)
  GPIO.setup(S3,GPIO.OUT)
  print("\n")

def find_frequency():
        start = time.time()
        #see how long it takes to read NUM_CYCLES
        for impulse_count in range(NUM_CYCLES):
            GPIO.wait_for_edge(OUT, GPIO.FALLING)
        duration = time.time() - start
        frequency = NUM_CYCLES/duration
        return frequency


def detect_loop():
    detect_colour = True
    redArr, greenArr, blueArr = [], [], []
    while(detect_colour):  

        # set pins to read red
        GPIO.output(S2,GPIO.LOW)
        GPIO.output(S3,GPIO.LOW)
        time.sleep(0.3) #don't know if we need delay

        red  = find_frequency()   
    
        #set pins to read blue
        GPIO.output(S2,GPIO.LOW)
        GPIO.output(S3,GPIO.HIGH)
        time.sleep(0.3)


        blue = find_frequency()
        
        #set pins to read green
        GPIO.output(S2,GPIO.HIGH)
        GPIO.output(S3,GPIO.HIGH)
        time.sleep(0.3)

        green = find_frequency()

        readCount += 1 #add one to count of frequencies 

        #append freq values to array
        redArr.append(red)
        blueArr.append(blue)
        greenArr.append(green)

        #update moving average of readings
        redAvg = sum(redArr)/len(redArr)
        blueAvg = sum(blueArr)/len(blueArr)
        greenAvg = sum(greenArr)/len(greenArr)
        
        #find differences between all values
        minDiff = min([abs(redAvg-blueAvg),abs(redAvg - greenAvg),abs(blueAvg-greenAvg)])

        if (readCount > maxRead and minDiff < diffThreshold):
            detect_colour = False
    
    return min([redAvg, blueAvg, greenAvg])

    
def endprogram():
    GPIO.cleanup()     
    
if __name__=='__main__':
    
    setup()

    try:
        detect_loop()

    except KeyboardInterrupt:
        endprogram()

