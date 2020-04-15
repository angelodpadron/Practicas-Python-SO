from hardware import *
from so import *
import log


##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size to 20 "cells"
    HARDWARE.setup(20)
    
    ## new create the Operative System Kernel
    kernel = Kernel()

    ##  create a program
    prg0 = Program("app0.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(3)])
    prg1 = Program("app1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO()])
    prg2 = Program("app2.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(4)])

    batch = [prg0, prg1, prg2]
    
    ## ejecucion individual

    kernel.run(prg0)

    ## ejecucion de lote

    kernel.executeBatch(batch)



