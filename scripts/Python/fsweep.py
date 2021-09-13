# -*- coding: utf-8 -*-

"""

sample macro where the pacing frequency is stepwise ramped up
all commands are logged to explog.log
ports are manually inserted, so make sure to know where the pacer is connected and adjust it accordingly
to check all ports call `print(serial.tools.list_ports)
same/ combined sweeps are of course also feasible for other parameters like U/w/polarity

"""

import serial
from time import sleep
import logging

#### setup logger and constants

logging.basicConfig(filename='explog.log', level=logging.DEBUG, format='%(asctime)s %(message)s')

# insert ports manually so far

pacer = serial.Serial()
pacer.timeout = 0.2
pacer.baudrate = 9600
pacer.port = 'COM7'
pacer.open()

WAITCMD = 0.2

### define helper functions

def waitmin(min):
    logging.warning("waiting for " + str(min) + " min")
    print("### waiting for " + str(min) + " min ###")
    sleep (min*60)
    
def waitsec(sec):
    logging.warning("waiting for " + str(sec) + " sec")
    print("### waiting for " + str(sec) + " sec ###")
    sleep (sec)

def init_pacer():
    """ after serial connection pacer does reboot ... wait until available """
    print("initing pacer, waiting 10 s for connection")
    logging.warning("initing pacer, waiting 10 s for connection")
    sleep(10)

def writepacer(cmd):
    """
        writes cmd to pacer, attaches \n and encodes messages as byte
        reads response
        response indicated by >
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
      
    # subsequent pacing variation
    # init pacer to 1 Hz, 50 ms, 10 V, negpol on both chan
    
    waitsec(15)
    
    writepacer("SETREM ON")
    writepacer("SETF 0.3 0.3")
    writepacer("SETW 50.0 50.0")
    writepacer("SETU 10.0 10.0")
    writepacer("SETPOL NEG NEG")
    writepacer("SETON OFF OFF")
    writepacer("GETPAR")
        
    waitsec(10)
    
    # turn on pacing
    writepacer("SETON ON ON")
    
    # sweep frequencies
    fs = ["0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0", "1.1", "1.2", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0"]
    for freq in fs:
        writepacer("SETF " + freq + " " + freq)
        waitsec(10)

    # place back to default frequency of 1.0 and turn off
    # check param for log
    writepacer("SETF 1.0 1.0")
    writepacer("SETON OFF OFF")
    writepacer("GETPAR")    

    logging.warning("### macro finished ###")
    print("### macro finished ###")
    
if __name__ == "__main__":
    init_pacer()
    controlmacro()
    
    pacer.close()