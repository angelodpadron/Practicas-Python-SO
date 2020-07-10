from hardware import *
from so import *
import log


##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size to 32 "cells"
    HARDWARE.setup(32, 4)

    ## Switch on computer
    HARDWARE.switchOn()

    ## new create the Operative System Kernel

    # schedulers:
    # - FCFS
    # - Priority
    # - PreemptivePriority
    # - RoundRobin(quantum) -> Round Robin es el scheduler por omision.
    kernel = Kernel()

    # programas en formato file system

    prg1 = Program("prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(2)])
    prg2 = Program("prg2.exe", [ASM.CPU(7)])
    prg3 = Program("prg3.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])

    # carga con file system

    kernel.fileSystem.write("/home/wine/prg1.exe", prg1)
    kernel.fileSystem.write("/home/wine/prg2.exe", prg2)
    kernel.fileSystem.write("/home/wine/prg3.exe", prg3)

    # execute all programs "concurrently"
    kernel.run("/home/wine/prg1.exe", 5)
    kernel.run("/home/wine/prg2.exe", 3)
    kernel.run("/home/wine/prg3.exe", 1)




