import serial
import time
import threading
import json
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 50000
UDP_PORT_IN = 50002
lidarPort = '\\.\COM5'
servoPort = '\\.\COM1'
TurnRate = 3 #Degrees per turn

class rplidar(object):

    def __init__(self, servo):
        self.servo = servo
        self.angle = 0
        servo.setAngle(0)
        self.connected = False
        self.running = False
        self.stopSem = threading.Semaphore(0)

    def connect(self):
        if (not self.connected ):
            self.serial = serial.Serial(port=lidarPort, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
            self.connected = True

    
    def disconnect(self):
        if (self.connected):
            self.serial.close()
            self.connected = False
        
    def startMotor(self):
        self.serial.dtr = False

    def stopMotor(self):
        self.serial.dtr = True;

    def getStatus(self):
        data = bytearray([0xa5,0x52])
        self.serial.write(data)
        data = self.serial.read(3)
        print(data)

    def stopScan(self):
        data = bytearray([0xa5, 0x25])
        self.serial.write(data);

    def reset(self):
        data = bytearray([0xa5, 0x40])
        self.serial.write(data)

    def scan(self):
        data = bytearray([0xa5, 0x20])
        self.serial.write(data)

    def readData(self, length = 1):
        self.serial.timeout = 0.5
        self.serial.write_timeout = 0.5
        self.serial.inter_byte_timeout = 0.5
        return self.serial.read(length)

    def startReadThread(self):
        self.thread = threading.Thread(target=self.recFunc)
        self.running = True
        self.thread.start()

    def stopReadThread(self):
        self.running = False
        self.semUDP.release()
        self.sem.release()

    def startServer(self):
        self.runningControl = True
        self.serverThread = threading.Thread(target=self.controlThread)
        self.serverThread.start()

    def controlThread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_IN))
        while(self.runningControl):
            data, addr = sock.recvfrom(100)
            s = data.decode("utf8")
            print(data)
            if (s == "start"):
                self.connect()
                self.startMotor()
                self.scan()
                self.startReadThread()
                
                print("Starting")
            elif (s == "stop"):
                self.stopReadThread()
                self.stopMotor()
                self.stopScan()
                self.stopSem.acquire()
                self.disconnect()
                print("Stopping")
                  

    def sendFunc(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while(self.running):
            self.semUDP.acquire(1)
            j = json.dumps(self.proc.__dict__)
            sock.sendto(j.encode("utf8"), (UDP_IP, UDP_PORT))

    def recFunc(self):
        self.sem = threading.Semaphore(0)
        self.semUDP = threading.Semaphore(0)
        turnCount = 0;
        self.processThread = threading.Thread(target=self.processFunc)
        self.processThread.start()

        self.sendThread = threading.Thread(target=self.sendFunc)
        self.sendThread.start()
        while (self.running):
            self.rawdata = bytearray()
            self.rawdata = bytearray(self.readData(1500))
            if (len(self.rawdata) != 1500):
                print("restart Scan")
                self.disconnect()
                time.sleep(0.5)
                self.connect()
                self.scan()
                self.startMotor()
                time.sleep(1)
                
            self.sem.release()
            self.angle = servo.getAngle();
            next = self.angle + TurnRate;
            if (next > 180):
                turnCount = turnCount + 1
                next = 0;
            if (not servo.setAngle(next)):
                print("Reset Device: Turn " + str(turnCount))
                self.servo.disconnect()
                time.sleep(1)
                self.servo.connect()
        self.stopSem.release()
            
            

    def processFunc(self):
        while(self.running):
            self.sem.acquire(1)
            if (not self.running):
                return
            self.proc = self.processData(self.rawdata)
            self.semUDP.release();


    def processData(self, data):
        l = len(data)
        out = SampleGroup()
        index = 0;
        while(l > 5):
            tmp = data[index] & 0x3
            if (tmp == 0 or (tmp & (tmp -1)) != 0): #look for sync bits
                index = index + 1
                l = l - 1
                continue


            quality = (data[index] & 0x03) >> 2
            angle = (((data[index + 1] & 0xFE) >> 1) | (data[index + 2] << 7)) / 64
            range = ((data[index+3]) | (data[index + 4] << 8)) / 4

            l = l - 5
            index = index + 5
            if (range != 0):
                out.addSample(range, self.angle, angle, quality)
        return out

    def printProcess(self, data):
        print(json.dumps(data.__dict__))


class SampleGroup(object):
    def __init__(self):
        self.quality  =  []
        self.range = []
        self.tilt = []
        self.pan = []

    def addSample(self, range, pan, tilt, quality):
        self.range.append(range)
        self.pan.append(pan)
        self.tilt.append(tilt)
        self.quality.append(quality)


class ServoControl(object):
    def __init__(self):
        self.connected = False
        self.angle = 90
        
    def connect(self):
        self.serial = serial.Serial(port=servoPort, baudrate=19200, write_timeout = 0.5)

    def disconnect(self):
        self.serial.close()

    def setAngle(self, angle):
        self.angle = angle
        s = "servo " + str(angle) + "\n"
        data = bytearray(s.encode('utf8'))
        try:
            w = self.serial.write(data)
            return True
        except serial.serialutil.SerialTimeoutException:
            return False

    def getAngle(self):
        return self.angle
try:
    servo = ServoControl()
    servo.connect()

    lidar = rplidar(servo)
    #lidar.startMotor()
    #allow lidar to spin up to speed
    #time.sleep(5)
    lidar.startServer()
    #lidar.startReadThread()
    #lidar.scan()
    #time.sleep(100)
    #lidar.stopReadThread()
    #lidar.stopMotor()
    while(lidar.running or lidar.runningControl):
        time.sleep(1)
except KeyboardInterrupt:
    lidar.stopReadThread()
    lidar.stopMotor()

