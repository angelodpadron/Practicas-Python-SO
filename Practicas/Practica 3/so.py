#!/usr/bin/env python

from hardware import *
import log



## emulates a compiled program
class Program():

    def __init__(self, name, instructions):
        self._name = name
        self._instructions = self.expand(instructions)

    @property
    def name(self):
        return self._name

    @property
    def instructions(self):
        return self._instructions

    def addInstr(self, instruction):
        self._instructions.append(instruction)

    def expand(self, instructions):
        expanded = []
        for i in instructions:
            if isinstance(i, list):
                ## is a list of instructions
                expanded.extend(i)
            else:
                ## a single instr (a String)
                expanded.append(i)

        ## now test if last instruction is EXIT
        ## if not... add an EXIT as final instruction
        last = expanded[-1]
        if not ASM.isEXIT(last):
            expanded.append(INSTRUCTION_EXIT)

        return expanded

    def __repr__(self):
        return "Program({name}, {instructions})".format(name=self._name, instructions=self._instructions)


## emulates an Input/Output device controller (driver)
class IoDeviceController():

    def __init__(self, device):
        self._device = device
        self._waiting_queue = []
        self._currentPCB = None

    def runOperation(self, pcb, instruction):
        pair = {'pcb': pcb, 'instruction': instruction}
        # append: adds the element at the end of the queue
        self._waiting_queue.append(pair)
        # try to send the instruction to hardware's device (if is idle)
        self.__load_from_waiting_queue_if_apply()

    def getFinishedPCB(self):
        finishedPCB = self._currentPCB
        self._currentPCB = None
        self.__load_from_waiting_queue_if_apply()
        return finishedPCB

    def __load_from_waiting_queue_if_apply(self):
        if (len(self._waiting_queue) > 0) and self._device.is_idle:
            ## pop(): extracts (deletes and return) the first element in queue
            pair = self._waiting_queue.pop(0)
            #print(pair)
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)


    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)


## emulates the system loader - PRACTICA 3
class Loader():
    def __init__(self):
        self._nextBaseDir = 0           ##inicialmente no hay ningun programa en memoria
    
    @property
    def setNextBaseDir(adress)          
        self._nextBaseDir = adress

    def nextBaseDir(self)
        return self._nextBaseDir

    def load(self, pcb, program):
        programSize = len(program.instructions)
        
        newBaseDir = self.nextBaseDir
        newBound = newBaseDir + programSize

        pcb.setBaseDir(newBaseDir)
        pcb.setBound(newBound)

        for i in range(0, programSize):
            HARDWARE.memory.write(i+newBaseDir, program.instruction[i])
            self.setNextProgramCell(newBound)
            log.logger.info(HARDWARE.memory)

        return newBaseDir

## emulates a dispatcher controller - PRACTICA 3
class Dispatcher():
    ## configura el pc de la cpu y direccion base 
    def load(self, pcb):
        HARDWARE.mmu.baseDir = pcb.baseDir
        HARDWARE.cpu.pc = pcb.pc
    ## "foto" del proceso en cpu + idle
    def save(self, pcb):
        pcb.pc = HARDWARE.cpu.pc
        HARDWARE.cpu.pc = -1



## process states - PRACTICA 3
NEW = "NEW"
READY = "READY"
RUNNING = "RUNNING"
WAITING = "WAITING"
TERMINATED = "TERMINATED"

## emulates a process control block - PRACTICA 3
class PCB():
    def __init__(self, pID, baseDir):        
        self._pID = pID
        self._baseDir = _baseDir

        self._pc = 0
        self._state = NEW
                
        #self._limit = 0        necesario? terminated deriba a un killhandler
        
        
    @property
    def pID(self):
        return self._pID
    
    @property
    def baseDir(self):
        return self._baseDir    
    
    @property
    def pc(self):
        return self._pc

    @property
    def state(self):
        return self._state
 
    ##setters
    @property
    def setState(self, state):
        self._setState = state

    @property
    def setBaseDir(self, baseDir):
        self._setBaseDir = _baseDir


## emulates a ready queue - PRACTICA 3
class ReadyQueue():
    def __init__(self):
        self._queue = _queue.Queue(50)

    @property
    def queue(self):
        return self._queue
    
    def put(self, pcb):
        self.queue.put(pcb)

    def get(self):
        return self.queue.get()


## emulates a PCB table -- PRACTICA 3
class PCBTable():
    def __init__(self):
        self._pcbTable = []
        self._runningPCB = None
        self._newPID = 0

    @property
    def pcbTable(self):
        return self._pcbTable

    @property
    def runningPCB(self):
        return self._runningPCB

    @property
    def newPID(self):
        return self._newPID    
    
    

    def add(self, pcb):
        self._PCBTable.add(pcb)

    def remove(self, pID):
        aux = self._PCBTable
        tableSize = len(self._PCBTable)
        for i in range(0, tableSize):
            if (self._PCBTable[i].pID() != pID):
                aux.add(self._PCBTable[i])
        self._PCBTable = aux

    def get(self, pID):
        for i in range (0, self._PCBTable):
            if (self._PCBTable[i].pID() == pID):
                return self._PCBTable[i]
        log.logger.error("THE PCB WITH PID {pID} IS NOT PRESENT IN TABLE")

    def getNewPID(self):
        aux = self.newPID
        self.newPID = self.newPID + 1
        return aux






## emulates the  Interruptions Handlers
class AbstractInterruptionHandler():
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    # setea el nuevo pcb dependiendo el estado de runningPCB - PRACTICA 3
    def setPCB(pcb):
         if (self.kernel.pcbTable.runningPCB == None):
            pcb.setState(RUNNING)
            self.kernel.pcbTable.runningPCB = pcb
            self.kernel.dispatcher.load(pcb)
        else:
            pcb.setState(READY)
            self.kernel.ReadyQueue.put(pcb)

    # planificar el siguiente proceso - PRACTICA 3
    def setNextPCB():
        #verificar ready queue
        if (not self.kernel.readyQueue.queue.isEmpty):
            #tomar un pcb con estado ready de la cola
            nextPCB = self.kernel.readyQueue.get
            nextPCB.setState(RUNNING)
            self.kernel.dispatcher.load(nextPCB)
            self.kernel.pcbTable.runningPCB = nextPCB




# - PRACTICA 3
class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        newProgram = irq.parameters                 ## guardar intrucciones de programa
        
        newPC = self.kernel.loader.load(newProgram)
        newPID = pcbTable.getNewPID()               ## obtener nuevo ID de proceso -- podria hacerlo el pc table al agregar
        newPCB = PCB(newPID, newPC)        
        
        self.kernel.pcbTable.add(newPCB)            ## añadir a tabla PCB el nuevo bloque de control

        newPCB.baseDir = self.kernel.loader.load(program)   ##obtener direccion

        self.setPCB(newPCB)
    


# - PRACTICA 3
class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        finishedPCB = self.kernel.pcbTable.runningPCB ## obtener pcb en ejecucion con state terminated
        finishedPCB.setState(TERMINATED)  ## proceso finalizado
        self.kernel.dispatcher.save(finishedPCB)
        self.setNextPCB()

    


    
# - PRACTICA 3
class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        operation = irq.parameters
        # guardar estado actual del proceso en ejecucion
        actualPCB = self.kernel.pcbTable.runningPCB
        actualPCB.state(WAITING)
        self.kernel.dispatcher.save(actualPCB)

        self.kernel.ioDeviceController.runOperation(actualPCB, operation)
        log.logger.info(self.kernel.ioDeviceController)

        self.setNextPCB(pcb)


# - PRACTICA 3
class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()        
        log.logger.info(self.kernel.ioDeviceController)
        self.setPCB(pcb)


# emulates the core of an Operative System
class Kernel():

    def __init__(self):
        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        # - PRACTICA 3
        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        ## setup loader - PRACTICA 3
        self._loader = Loader(self)

        ## setup dispatcher - PRACTICA 3
        self._dispatcher = Dispatcher()

        ## setup PCB table - PRACTICA 3
        self._pcbTable = PCBTable()

        ## setup Ready Queue - PRACTICA 3
        self._readyQueue = ReadyQueue()


    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    @property
    def loader(self):
        return self._loader
    
    @property
    def dispatcher(self):
        return self._dispatcher

    @property
    def pcbTable(self):
        return self._pcbTable

    @property
    def readyQueue(self):
        return self._readyQueue   
    
    ## emulates a "system call" for programs execution - PRACTICA 3
    def run(self, program):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, program)
        HARDWARE.interruptVector.handle(newIRQ)       


    def __repr__(self):
        return "Kernel "
