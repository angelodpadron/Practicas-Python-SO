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
class IoDeviceController:

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


## emulates the  Interruptions Handlers
class AbstractInterruptionHandler():
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    # toma un proceso en estado ready de la cola
    def fetchReadyPCB(self):
        if self.kernel.readyQueue.lenght() > 0:
            nextPCB = self.kernel.readyQueue.pop(0)
            self.kernel.dispatcher.load(nextPCB)
            nextPCB.changeStateTo(RUNNING)
            self.kernel.pcbTable.setRunningPCB(nextPCB)

    # setea el pcb en running o ready dependiendo si la cpu esta atendiendo un proceso o no.
    def stagePCB(self, pcb):
        if self.kernel.pcbTable.runningPCB is None:
            pcb.changeStateTo(RUNNING)
            self.kernel.pcbTable.setRunningPCB(pcb)
            self.kernel.dispatcher.load(pcb)
        else:
            pcb.changeStateTo(READY)
            self.kernel.readyQueue.add(pcb)


class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        program = irq.parameters
        newPCB = PCB(program.name)
        self.kernel.pcbTable.add(newPCB)
        self.kernel.loader.load(newPCB, program)
        self.stagePCB(newPCB)


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        killPCB = self.kernel.pcbTable.runningPCB
        self.kernel.dispatcher.save(killPCB)
        killPCB.changeStateTo(TERMINATED)
        self.kernel.pcbTable.setRunningPCB(None)
        log.logger.info("{killPCB.path} finished correctly")
        self.fetchReadyPCB()


class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        operation = irq.parameters
        inPCB = self.kernel.pcbTable.runningPCB
        self.kernel.dispatcher.save(inPCB)
        inPCB.changeStateTo(WAITING)
        self.kernel.pcbTable.setRunningPCB(None)
        self.kernel.ioDeviceController.runOperation(inPCB, operation)
        log.logger.info(self.kernel.ioDeviceController)
        self.fetchReadyPCB()


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        outPCB = self.kernel.ioDeviceController.getFinishedPCB()
        log.logger.info(self.kernel.ioDeviceController)
        self.stagePCB(outPCB)


class Dispatcher:

    def __init__(self):
        pass

    def save(self, pcb):
        pcb.setPc(HARDWARE.cpu.pc)
        HARDWARE.cpu.pc = -1

    def load(self, pcb):
        HARDWARE.cpu.pc = pcb.pc
        HARDWARE.mmu.baseDir = pcb.baseDir


class Loader:

    def __init__(self):
        self._nextProgram = 0

    def load(self, pcb, program):
        newBaseDir = self._nextProgram
        progSize = len(program.instructions)
        newLimit = newBaseDir + progSize
        pcb.setBaseDir(newBaseDir)
        pcb.setLimit(newLimit)
        for i in range(0, progSize):
            HARDWARE.memory.write(i+newBaseDir, program.instructions[i])
        self.setNextProgram(newLimit)
        log.logger.info(HARDWARE.memory)


    @property
    def nextProgram(self):
        return self._nextProgram

    # error con annotation setter 
    def setNextProgram(self, value):
        self._nextProgram = value


class ReadyQueue:

    def __init__(self):
        self._ready_queue = []

    def add(self, pcb):
        self._ready_queue.append(pcb)

    def pop(self, n):
        return self._ready_queue.pop(n)

    def lenght(self):
        return len(self._ready_queue)
        

# Estados posibles de un proceso
NEW = "NEW"
READY = "READY"
WAITING = "WAITING"
RUNNING = "RUNNING"
TERMINATED = "TERMINATED"


class PCB:
    def __init__(self, path):
        self._pid = -1
        self._baseDir = -1
        self._limit = -1
        self._pc = 0
        self._state = NEW
        self._path = path           #nombre de programa como direccion

    @property
    def pid(self):
        return self._pid

    @property
    def baseDir(self):
        return self._baseDir

    @property
    def limit(self):
        return self._limit

    @property
    def pc(self):
        return self._pc

    @property
    def state(self):
        return self._state

    @property
    def path(self):
        return self._path

    # setters manuales (error con annotation @setter)

    def setBaseDir(self, baseDir):
        self._baseDir = baseDir

    def setPid(self, pid):
        self._pid = pid

    def setLimit(self, limit):
        self._limit = limit

    def setPc(self, pc):
        self._pc = pc

    def changeStateTo(self, state):
        self._state = state


class PCBTable:
    def __init__(self):
        self._PCBTable = dict()
        self._runningPCB = None
        self._PIDCounter = 0

    def get(self, pid):
        return self._PCBTable[pid]

    def add(self, pcb):
        pcb.setPid(self._PIDCounter)
        self._PCBTable[self._PIDCounter] = pcb
        self.getNewPID()

    def remove(self, pid):
        del self._PCBTable[pid]

    @property
    def runningPCB(self):
        return self._runningPCB

    # error con annotation @setter
    def setRunningPCB(self, pcb):
        self._runningPCB = pcb

    def getNewPID(self):
        self._PIDCounter += 1


# emulates the core of an Operative System
class Kernel:

    def __init__(self):
        ## setup interruption handlers
        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        # setup controladora de dispositivo IO
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        # setup componentes del sistema
        self._readyQueue = ReadyQueue()
        self._dispatcher = Dispatcher()
        self._loader = Loader()
        self._pcbTable = PCBTable()

    @property
    def readyQueue(self):
        return self._readyQueue

    @property
    def dispatcher(self):
        return self._dispatcher

    @property
    def loader(self):
        return self._loader

    @property
    def pcbTable(self):
        return self._pcbTable

    @property
    def ioDeviceController(self):
        return self._ioDeviceController


    # funcion principal
    def run(self, program):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, program)
        HARDWARE.interruptVector.handle(newIRQ)

    def __repr__(self):
        return "Kernel "
