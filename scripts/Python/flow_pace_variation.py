# -*- coding: utf-8 -*-

"""

sample macro where a pump and the pacer are controlled in a coordinated way
all commands are logged with timestamps to explog.log to possibly sync events
ports are manually inserted, so make sure to know where the pacer is connected and adjust it accordingly
to check all ports call `print(serial.tools.list_ports)

various stages are displayed:
- vary pacing parameters, switch paced channel
- start/ stop pump
- vary pumprates
- start/ stop pump with combined pacing variation
"""

import serial
from time import sleep
import logging

#### setup logger and constants

logging.basicConfig(filename='explog.log', level=logging.DEBUG, format='%(asctime)s %(message)s')# encoding='utf-8', 

pump = serial.Serial()
pump.timeout = 0.2
pump.baudrate = 9600 # make sure to set same baudrate to pump
pump.port = 'COM8'
pump.open()

pacer = serial.Serial()
pacer.timeout = 0.2
pacer.baudrate = 9600
pacer.port = 'COM7'
pacer.open()

TSLEEP = 5
WAITCMD = 0.2

### define helper functions

def waitmin(min):
    logging.warning("waiting for " + str(min) + " min")
    print("### waiting for " + str(min) + " min ###")
    sleep (min*60)
    
def init_pump():
    """ inits pump to infusing + running """
    print("initing pump, set to infuse and start")
    logging.warning("initing pump, set to infuse and start")    
    
    writepump("STP")    
    writepump("DIR INF") # make sure pump is infusing
    # pump is already set to 50 µl/h
    # if new value is set pump has to be on stop before setting value with
    # RAT 50 UH (50 µl/h)
    writepump("RUN")

def init_pacer():
    """ after serial connection pacer does reboot ... wait until available """
    print("initing pacer, waiting 10 s for connection")
    logging.warning("initing pacer, waiting 10 s for connection")
    sleep(10)

def writepump(cmd):
    """
        writes cmd to pump, attaches \r and encodes message as byte
        reads response until pump.timeout occurs and logs + prints resonse
        response indicated by >
    """

    print(cmd)
    logging.warning(cmd)
    cmd = str.encode(cmd + '\r')
    pump.write(cmd)
    
    rawresp = pump.readline() # take care that timeout is specified above, will block otherwise
    resp = rawresp.decode("ascii").strip('\x02').strip('\x03')
    resp = "> " + resp
    print(resp)
    logging.warning(resp)
    
    sleep(WAITCMD)  # waits a short period after sending signal

def writepacer(cmd):
    """
        writes cmd to pacer, attaches \n and encodes messages as byte
        reads response
        response indicated by >
        
        communicating with the pacer just needs a call to `writepacer("SETON ON ON")`
    """

    print(cmd)
    logging.warning(cmd)
    cmd = str.encode(cmd + '\n')
    pacer.write(cmd)
        
    rawresp = pacer.read(1000) # reads up to 1000 bytes or until pacer.timeout occurs
    # expected answers short for setting values
    # longest answer if GETPAR sent
    
    resplines = rawresp.decode("ascii").splitlines()
    
    for line in resplines:
        print(line)
        logging.warning(line)
    
    sleep(WAITCMD)  # waits a short period after sending signal 

### main macro function

def controlmacro():
    logging.warning("### starting macro ###")
    print("### starting macro ###")
    
    # stage 1:     
    # subsequent pacing variation
    # pace chan1 for PACER_MIN, then rest REST_MIN, 
    # then pace chan2 for PACER_MIN, then rest REST_MIN
    # perform everything 3x
    
    # init pacer to 1 Hz, 50 ms, 10 V, negpol on both chan
    
    writepacer("SETREM ON")
    writepacer("SETF 1.0 1.0")
    writepacer("SETW 50.0 50.0")
    writepacer("SETU 10.0 10.0")
    writepacer("SETPOL NEG NEG")
    writepacer("GETPAR")
    
    PACER_MIN = 10
    REST_MIN = 10
    
    for i in range(3):
        writepacer("SETON ON OFF") # pace channel 1 for PACER_MIN
        waitmin(PACER_MIN)
        writepacer("SETON OFF OFF") # rest for REST_MIN
        waitmin(REST_MIN)
        writepacer("SETON OFF ON") # pace channel 2 for PACER_MIN
        waitmin(PACER_MIN)        
        writepacer("SETON OFF OFF") # rest for REST_MIN
        waitmin(REST_MIN)
    
    writepacer("GETPAR")
    waitmin(20) # wait for 20 more minute to stabilize signal
    # ~ 140 min elapsed
    
    # stage 2:
    # pumping rate variation
    # stop pump NOFLOW_MIN, start pump for ONFLOW_MIN
    # stop and restart 3x
     
    NOFLOW_MIN = 30
    ONFLOW_MIN = 30
    
    # 3 cycles of off-on
    for i in range(3):
        writepump("STP")
        waitmin(NOFLOW_MIN)
        writepump("RUN")
        waitmin(ONFLOW_MIN)    
    
    # ~ 320 min elapsed
    
    # stage 3:
    # pumping rate variation
    # flowrates [µl/h]: 200-400-600-400-200-50
    # keep pump for FLOWSTEP_MIN at specified pumping rate    
    
    FLOWSTEP_MIN = 10    
    
    writepump("STP")
    writepump("RAT 200 UH")
    writepump("RUN")
    waitmin(FLOWSTEP_MIN)
    
    writepump("STP")
    writepump("RAT 400 UH")
    writepump("RUN")
    waitmin(FLOWSTEP_MIN)
    
    writepump("STP")
    writepump("RAT 600 UH")
    writepump("RUN")
    waitmin(FLOWSTEP_MIN)    

    writepump("STP")
    writepump("RAT 400 UH")
    writepump("RUN")
    waitmin(FLOWSTEP_MIN)

    writepump("STP")
    writepump("RAT 200 UH")
    writepump("RUN")
    waitmin(FLOWSTEP_MIN)

    writepump("STP")
    writepump("RAT 50 UH")
    writepump("RUN")
    # set back to 50 µl/h when finished
 
    # ~ 320 min elapsed

    
    # stage 4:
    # combine pacing with pumping rate variation
    # stop pump + pace 30 min, enable pump again wo. pacing for 30 min
    
    # set basic pacer config
    writepacer("SETREM ON")
    writepacer("SETF 1.0 1.0")
    writepacer("SETW 50.0 50.0")
    writepacer("SETU 10.0 10.0")
    writepacer("SETPOL NEG NEG")
    writepacer("GETPAR")

    #3 cycles: stop pupmp + pace both channels for NOFLOW_MIN, then stop pacing, start pump for ONFLOW_MIN
    for i in range(3):
        writepacer("SETON ON ON")
        writepump("STP")
        waitmin(NOFLOW_MIN)
        writepacer("SETON OFF OFF")
        writepump("RUN")
        waitmin(ONFLOW_MIN)
    
    # ~ 500 min elapsed
    
    # repeat with different frequency
    # set f for both chan to 1.3 Hz
    writepacer("SETF 1.3 1.3")
    #3 cycles: pace both channels + stop pump -> stop pacing, start pump
    for i in range(3):
        writepacer("SETON ON ON")
        writepump("STP")
        waitmin(NOFLOW_MIN)
        writepacer("SETON OFF OFF")
        writepump("RUN")
        waitmin(ONFLOW_MIN)
    # ~ 680 min elapsed
    
    writepacer("GETPAR")    


    logging.warning("### macro finished ###")
    print("### macro finished ###")
    
if __name__ == "__main__":
    init_pump()
    init_pacer()
    controlmacro()
    
    pump.close()
    pacer.close()