#!/usr/bin/env python
import heapq

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
            # print(pair)
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)

    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(
            deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)


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
        if not self.kernel.scheduler.emptyRQ():
            nextPCB = self.kernel.scheduler.next()
            self.kernel.dispatcher.load(nextPCB)
            nextPCB.changeStateTo(RUNNING)
            self.kernel.pcbTable.setRunningPCB(nextPCB)

            log.logger.info("{prg_name} will start running".format(prg_name=nextPCB.path))

    # setea el pcb en running o ready dependiendo si la cpu esta atendiendo un proceso o no.
    def stagePCB(self, pcb):
        if self.kernel.pcbTable.runningPCB is None:
            pcb.changeStateTo(RUNNING)
            self.kernel.pcbTable.setRunningPCB(pcb)
            self.kernel.dispatcher.load(pcb)

            log.logger.info("{prg} will start running".format(prg=pcb.path))

        # update: stagePCB debe verificar si el scheduler implementado requiere expropiar el pcb en ejecucion

        elif self.kernel.scheduler.expropriationRequired(pcb, self.kernel.pcbTable.runningPCB):
            previousPCB = self.kernel.pcbTable.runningPCB
            self.kernel.dispatcher.save(previousPCB)
            previousPCB.changeStateTo(READY)
            self.kernel.scheduler.add(previousPCB)

            log.logger.info("{prg1} has been added to the ready queue due a lower priority than {prg2}".format(prg1=previousPCB.path, prg2=pcb.path))

            pcb.changeStateTo(RUNNING)
            self.kernel.pcbTable.setRunningPCB(pcb)
            self.kernel.dispatcher.load(pcb)

            log.logger.info("{prg} will start running".format(prg=pcb.path))

        else:
            pcb.changeStateTo(READY)
            self.kernel.scheduler.add(pcb)

            log.logger.info("{prg_name} has been added to the ready queue".format(prg_name=pcb.path))


class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        # refactor: new debe handlear el par con la prioridad
        entryParameters = irq.parameters
        programPath = entryParameters['path']
        if programPath in self.kernel.fileSystem.allPaths():
            priority = entryParameters['priority']
            newPCB = PCB(programPath, priority)
            self.kernel.pcbTable.add(newPCB)
            self.kernel.loader.load(newPCB)
            log.logger.info("{prg} loaded onto pcb table".format(prg=newPCB.path))
            self.stagePCB(newPCB)
        else:
            log.logger.info("Error: No such directory")


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        killPCB = self.kernel.pcbTable.runningPCB
        self.kernel.dispatcher.save(killPCB)
        killPCB.changeStateTo(TERMINATED)
        self.kernel.pcbTable.setRunningPCB(None)
        log.logger.info("{finished_prg} finished correctly.".format(finished_prg=killPCB.path))
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
        log.logger.info("{prg} is waiting now...".format(prg=inPCB.path))
        self.fetchReadyPCB()


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        outPCB = self.kernel.ioDeviceController.getFinishedPCB()
        log.logger.info(self.kernel.ioDeviceController)
        self.stagePCB(outPCB)


class TimeOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        timedOutPCB = self.kernel.pcbTable.runningPCB
        self.kernel.dispatcher.save(timedOutPCB)
        timedOutPCB.changeStateTo(READY)
        self.kernel.scheduler.add(timedOutPCB)

        log.logger.info("{prg} reached the CPU burst limit and has been moved to the ready queue".format(prg=timedOutPCB.path))

        self.kernel.pcbTable.setRunningPCB(None)
        self.fetchReadyPCB()

        HARDWARE.timer.reset()



class Dispatcher:

    def __init__(self, memoryManager):
        self._memoryManager = memoryManager

    def save(self, pcb):
        pcb.setPc(HARDWARE.cpu.pc)
        HARDWARE.cpu.pc = -1

    def load(self, pcb):
        HARDWARE.cpu.pc = pcb.pc
        HARDWARE.timer.reset()
        HARDWARE.mmu.resetTLB()
        currentPageTable = self._memoryManager.getPageTable(pcb.pid)
        for i in currentPageTable.getPagesData():
            HARDWARE.mmu.setPageFrame(i, currentPageTable.getFrame(i))


class Loader:

    def __init__(self, memoryManager, fileSystem):
        # self._nextProgram = 0
        self._frameSize = HARDWARE.mmu.frameSize
        self._memoryManager = memoryManager
        self._fileSystem = fileSystem

    def load(self, pcb):
        # fetch program from file system
        program = self._fileSystem.read(pcb.path)
        framedInstructions = self.chunkify(program.instructions)
        availableFrames = self._memoryManager.allocFrames(len(framedInstructions))
        pageTable = PGTable()
        for i in range(0, len(availableFrames)):
            pageTable.linkPageTable(i, availableFrames[i])
        self._memoryManager.putPageTable(pcb.pid, pageTable)
        self.loadPages(framedInstructions, availableFrames)
        log.logger.info(HARDWARE.memory)

    def chunkify(self, instructions):
        return list(self.chunks(instructions, self._frameSize))

    def chunks(self, instructions, frameSize):
        for i in range(0, len(instructions), frameSize):
            yield instructions[i:i + frameSize]

    def loadPages(self, framedInstructions, availableFrames):
        for n in range(0, len(framedInstructions)):
            frameID = availableFrames[n]
            self.auxLoadPages(framedInstructions[n], frameID)

    def auxLoadPages(self, instructionPage, frameID):
        frameBaseDir = frameID * self._frameSize
        for i in range(0, len(instructionPage)):
            HARDWARE.memory.write(frameBaseDir + i, instructionPage[i])


# Schedulers
class AbstractScheduler:
    def __init__(self):
        self._readyQueue = []

    def add(self, pcb):
        pass

    def next(self):
        pass

    def emptyRQ(self):
        return len(self._readyQueue) == 0

    def expropriationRequired(self, pcb1, pcb2):
        return False

    def calcPriority(self, defaultPriority, arrivalTime):
        if arrivalTime < 5:
            return defaultPriority
        elif arrivalTime < 10 and defaultPriority > 1:
            return defaultPriority - 1
        elif arrivalTime < 15 and defaultPriority > 2:
            return defaultPriority - 2
        elif arrivalTime < 20 and defaultPriority > 3:
            return defaultPriority - 3
        else:
            return 1


class FCFS(AbstractScheduler):

    def add(self, pcb):

        self._readyQueue.append(pcb)

    def next(self):
        return self._readyQueue.pop(0)


class Priority(AbstractScheduler):

    def __init__(self):
        self._arrivalTime = 0 # segunda variable para handlear prioridades iguales
        super(Priority, self).__init__()

    def add(self, pcb):
        pcb.setAge(0)
        heapq.heappush(self._readyQueue, (self.calcPriority(pcb.priority, pcb.age), self._arrivalTime, pcb))
        self._arrivalTime += 1

    def next(self):
        self.updateAllAges()
        return heapq.heappop(self._readyQueue)[-1]   # pendiente algo mas prolijo

    def updateAllAges(self):
        for i in self._readyQueue:
            i[-1].incrementAge()


class PreemptivePriority(Priority):

    def expropriationRequired(self, pcb1, pcb2):
        return self.calcPriority(pcb1.priority, pcb1.age) < self.calcPriority(pcb2.priority, pcb2.age)


class RoundRobin(FCFS):
    def __init__(self, quantum):
        super(RoundRobin, self).__init__()
        HARDWARE.timer.quantum = quantum


# Estados posibles de un proceso
NEW = "NEW"
READY = "READY"
WAITING = "WAITING"
RUNNING = "RUNNING"
TERMINATED = "TERMINATED"


# File system
class FileSystem:
    def __init__(self):
        self._programDictionary = {}

    def write(self, path, program):
        self._programDictionary[path] = program
        # log.logger.info("{program} location is {path}".format(program=program[0], path=path))

    def read(self, path):
        return self._programDictionary[path]

    def allPaths(self):
        return self._programDictionary.keys()


# Memory manager
class MemoryManager:
    def __init__(self):
        self._frameSize = HARDWARE.mmu.frameSize
        self._pageTables = {}
        self._freeFrames = []
        self.initializeArray()

    def initializeArray(self):
        for i in range(0, ((HARDWARE.memory.size // self._frameSize) - 1)):
            self._freeFrames.append(i)

    def allocFrames(self, n):
        frames = []
        for i in range(0, n):
            frames.append(self._freeFrames.pop(0))
        return frames

    # liberar frames usados por el proceso de PID
    def freeFrames(self, PID):
        for i in self._pageTables.pop(PID).framesData():
            self._freeFrames.append(i)

    def getPageTable(self, PID):
        return self._pageTables[PID]

    def putPageTable(self, PID, pageTable):
        self._pageTables[PID] = pageTable

    # consultar memoria disponible
    def freeMemory(self):
        return len(self._freeFrames) * self._frameSize


# Page-Frame table
class PGTable:
    def __init__(self):
        self._pgTable = {}

    def linkPageTable(self, page, frame):
        self._pgTable[page] = frame

    def getFrame(self, page):
        return self._pgTable[page]

    def getPagesData(self):
        return self._pgTable.keys()

    def getFramesData(self):
        return self._pgTable.values()


class PCB:
    def __init__(self, path, priority):
        self._pid = -1
        self._baseDir = -1
        self._limit = -1
        self._pc = 0
        self._state = NEW
        self._path = path

        # priority
        self._age = 0
        self._priority = priority

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

    # Priority scheduler fields
    @property
    def age(self):
        return self._age

    @property
    def priority(self):
        return self._priority

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

    # Priority schedulers functions
    def incrementAge(self):
        if self._state != RUNNING or self._state != WAITING:
            self._age = self._age + 1
            log.logger.info("{prg} arrival time is {age}".format(prg=self.path, age=self.age))

    def setPriority(self, priority):
        self._priority = priority

    def setAge(self, age):
        self._age = age


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

    def __init__(self, scheduler=None):
        ## setup interruption handlers
        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        timeOutHandler = TimeOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(TIMEOUT_INTERRUPTION_TYPE, timeOutHandler)

        # setup controladora de dispositivo IO
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        # setup componentes del sistema
        self._memoryManager = MemoryManager()
        self._dispatcher = Dispatcher(self._memoryManager)
        self._fileSystem = FileSystem()
        self._loader = Loader(self._memoryManager, self._fileSystem)
        self._pcbTable = PCBTable()


        if scheduler is None:
            self._scheduler = RoundRobin(3)
            log.logger.info("No scheduling algorithm was specified. The kernel will schedule with {scheduler} algorithm".format(
                scheduler=self.scheduler.__class__.__name__))
        else:
            self._scheduler = scheduler
            log.logger.info("The system will schedule with {scheduler} algorithm".format(scheduler=self.scheduler.__class__.__name__))

    @property
    def scheduler(self):
        return self._scheduler

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

    @property
    def fileSystem(self):
        return self._fileSystem

    # funcion principal
    # refactor: run debe armar un par con el programa y su prioridad
    def run(self, path, priority):
        newParameters = {
            'path': path,
            'priority': priority
        }
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, newParameters)
        HARDWARE.interruptVector.handle(newIRQ)

    def __repr__(self):
        return "Kernel "
