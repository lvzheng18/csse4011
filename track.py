#!/usr/bin/python

import socket
import math
import time
import datetime
import struct
import StringIO
from threading import Thread
import sys
import json
import numpy
import pylab
from matplotlib.animation import FuncAnimation

UDP_TIMESYNC_PORT = 3000 # node listens for timesync packets on port 4003
UDP_REPLY_PORT = 3001 # node listens for reply packets on port 7005

isRunning = True 

'''

sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, 0)
sock.bind(('', UDP_REPLY_PORT))
sock.settimeout(1.0)

'''

unit_size = 68.0
'''
def calculate_position1(converted, positions):
  b=[]
  A=[]

  k = converted[-1]
  kp = positions[-1]
  for i in range(len(positions) - 1):
    
    con = converted[i]
    pos = positions[i]
    if con[1] == -1:
      continue


    b1 = con[1]**2 - k[0]**2 - pos[0]**2 - pos[1]**2 + kp[0]**2 + kp[1]**2
    a1 = 2 * (kp[0] - pos[0])
    a2 = 2 * (kp[1] - pos[1])
    b.append(b1)
    A.append([a1, a2])

  x,y = numpy.linalg.lstsq(A, b)[0]
  return [x, y]

'''

def calculate_position(r):
  
  p = [(0 ,0 ), (2 ,0 ), (2 ,2 ), (0 ,2 )]
  b=[]
  A=[]
  if (r[-1] == -1):
    rk = r[2]
    k = 2
  else:
    rk = r[-1]
    k  = 3

  for i in range(3):
    if (r[i] == -1):
      continue

    b1 = r[i]**2 - rk*rk - p[i][0]**2 - p[i][1]**2 + p[k][0]**2+ p[k][1]**2
    a1 = 2*(p[k][0] - p[i][0])
    a2 = 2*(p[k][1] - p[i][1])
    b.append(b1)
    A.append([a1, a2])

  At = numpy.transpose(A)
  Ainv = numpy.linalg.inv(numpy.dot(At, A))
  AinvAt = numpy.dot(Ainv, At)
  
  out = numpy.dot(AinvAt,b)
  return out


  '''
  b = numpy.array([r1*r1 - rk*rk - p[0][0]**2 - p[0][1]**2 + p[3][0]**2+ p[3][1]**2,
  r2*r2 - rk*rk - p[1][0]**2 - p[1][1]**2 + p[3][0]**2+ p[3][1]**2,
  r3*r3 - rk*rk - p[2][0]**2 - p[2][1]**2 + p[3][0]**2+ p[3][1]**2])

  A = numpy.array([(2*(p[3][0] - p[0][0]), 2*(p[3][1] - p[0][1])),
  (2*(p[3][0] - p[1][0]), 2*(p[3][1] - p[1][1])),
  (2*(p[3][0] - p[2][0]), 2*(p[3][1] - p[2][1]))])
  
  At = numpy.transpose(A)
  

  Ainv = numpy.linalg.inv(numpy.dot(At, A))
  AinvAt = numpy.dot(Ainv, At)
  
  out = numpy.dot(AinvAt,b)
  return out
  '''



class Kalman:
    def __init__(self):

		self.n = 1
		self.m = 2

		self.x_hat = 0;
		self.A = numpy.array([(1, 0), (0, 1)]);
		self.P = 1;
		self.I = 1;
		self.C = numpy.array([(1) ,(1)]);
		self.R = numpy.array([(0.64, 0) ,(0, 0.64)]);
		self.Q = 0.05;
		self.G = numpy.array([(1, 1)]);
		
		
        

    def update(self, obs):
		self.obs1 = numpy.transpose(obs)
		self.P = self.P + self.Q;
		self.buffer1 = self.P * numpy.transpose(self.C)
		self.buffer3 = numpy.array([(self.P, self.P) ,(self.P, self.P)]) + self.R
		self.buffer2 = numpy.linalg.inv(self.buffer3)
		self.G = numpy.dot(self.buffer1, self.buffer2)
		self.P = (1 - numpy.dot(self.G, self.C)) * self.P 

		self.buffer4 = self.obs1 - self.C * self.x_hat

		self.x_hat = self.x_hat + numpy.dot(self.G, self.buffer4)



init_error = 1;
cov_init=init_error*numpy.eye(4)
x_init = numpy.array( [0, 0, 0, 0] )
k1 = Kalman()
k2 = Kalman()

inputs = numpy.zeros(4)
vx = 0.1
vy = 0.1
x_cor = 0
y_cor = 0


def plot():
  data = numpy.array( [(0),(0)] ) #change
  fig = pylab.figure()
  f1 = pylab.subplot(111)
  f2 = f1.scatter(data[0], data[1], s=10,c='red',marker='s', edgecolors='red', label="estimated loc")
  #f.xlabel('x [m]')
  #f.ylabel('y [m]')

  def update(self):
    data[0] = x_cor
    data[1] = y_cor
    
    f2.set_offsets(data)
  pylab.xlabel('x') 
  pylab.xlabel('y') 
  pylab.axis([-1, 14, -1, 12])
  animation = FuncAnimation(fig, update, interval=300)
  
  pylab.show()

# start plot as a thread
t1 = Thread(target=plot)
t1.start()


#create some dimmy data
a1  = [0 , 152.0526 , 245.1775 , 384.6661 , 626.9290 , 435.4124 , 435.4124 , 435.4124 , 435.4124]
b1 = [136.0000 , 152.0526  ,204.0000 , 304.1052 , 531.0970 , 366.1912 , 366.1912 , 366.1912 , 366.1912]
c1 = [192.3330  , 68.0000  , 68.0000 , 192.3330 , 435.4124 , 245.1775 , 245.1775 , 245.1775 , 245.1775]
d1 = [136.0000 ,  68.0000  ,152.0526 , 304.1052 , 548.2335 , 340.0000, 340.0000, 340.0000, 340.0000]
count = -1
#dimmy data


while True:
  track = "aaaa::212:4b00:7b5:4f81"
  ips = ["aaaa::212:4b00:7b5:7585", "aaaa::212:4b00:7b5:1e84", "aaaa::212:4b00:7b5:3c86", "aaaa::212:4b00:7b5:3881"]
  p = [(0 ,0 ), (2 ,0 ), (2 ,2 ), (0 ,2 )] #in mm ideally 
  recieved = [] 
  converted = [] #[(RSSI mm, Sonar mm), () ...]

  
  '''

  try:
    for ip in ips:
      #send rssi request
      sock.sendto(track, (ip, UDP_TIMESYNC_PORT))
      #recieve request
      data, addr = sock.recvfrom(UDP_REPLY_PORT)
      print data

      #decode data
      j = json.loads(data)
      recieved.append((addr, int(j["NODE RSSI"]), int(j["RSSI"]), int(j["SONAR"])))

  except socket.timeout:
    continue #retry

  
  



  #have all data, convert rssi to distance
  for block in recieved:
    r1 = block[1] * -32.99 - 1033
    r2 = block[2] * -32.99 - 1033
    if (block[3] < 0):
      converted.append(((r1 + r2)/ 2 / unit_size, block[3]))
    else:
      converted.append(((r1 + r2)/ 2 / unit_size, block[3]/unit_size))
  
  #least squares
  #b = np.array()
  #A = np.array()
  '''
  count = count + 1
  if count == len(a1):
    break

  converted.append((a1[count] / unit_size  + 1, -1) )
  converted.append((b1[count] / unit_size   - 0.3, -1))
  converted.append((c1[count] / unit_size  + 0.4, c1[count] / unit_size) )
  converted.append((d1[count] / unit_size  + 0.5, d1[count] / unit_size ))
  
  time.sleep(1)
  
  negone = 0
  for q in converted:
    if(q[1] == -1) :
      negone = negone + 1




  if (negone > 1) :
    try:
      xy_ssr = calculate_position([converted[0][0], converted[1][0], converted[2][0], converted[3][0]])

    except  numpy.linalg.linalg.LinAlgError:
      print "singluar matrix"
      continue  
    
    inputs[2] = k1.x_hat + vx
    inputs[3] = k2.x_hat + vy
    inputs[0] = xy_ssr[0]
    inputs[1] = xy_ssr[1]

    k1.update([inputs[0], inputs[2]])
    k2.update([inputs[1], inputs[3]])
    print k1.x_hat, k2.x_hat
    print "issr error ",  math.sqrt((k1.x_hat - xy_ssr[0])**2 + (k2.x_hat - xy_ssr[0])**2)
    #print k.x_hat
    #print vx, vy
  else :
    try:
      
      xy_ssr = calculate_position([converted[0][0], converted[1][0], converted[2][0], converted[3][0]])
      
      xy_ult = calculate_position([converted[0][1], converted[1][1], converted[2][1], converted[3][1]])
      
    except  numpy.linalg.linalg.LinAlgError:
      print "singluar matrix"
      continue
    inputs[0] = xy_ult[0]
    inputs[1] = xy_ult[1]
    inputs[2] = xy_ssr[0]
    inputs[3] = xy_ssr[1]

    k1.update([inputs[0], inputs[2]])
    k2.update([inputs[1], inputs[3]])
    print xy_ssr, xy_ult
    print k1.x_hat, k2.x_hat
    print "issr error ",  math.sqrt((k1.x_hat - xy_ssr[0])**2 + (k2.x_hat - xy_ssr[0])**2)
    print "ult error ",  math.sqrt((k1.x_hat - xy_ult[0])**2 + (k2.x_hat - xy_ult[0])**2)
    
  

  vx = k1.x_hat - x_cor
  vy = k2.x_hat - y_cor
  x_cor = k1.x_hat
  y_cor = k2.x_hat







  
