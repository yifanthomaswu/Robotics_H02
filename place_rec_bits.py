#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os

import time
import math
import brickpi

interface=brickpi.Interface()
interface.initialize()

motor = 2

interface.motorEnable(motor)

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 20.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 400
motorParams.pidParameters.k_i = 100
motorParams.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motor,motorParams)

sonar = 2
interface.sensorEnable(sonar, brickpi.SensorType.SENSOR_ULTRASONIC);


# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 180):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                s = (s[1:len(s) - 1]).split(',')[0]
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."
        
        return ls
    
def readSonar():
    readings = []
    i = 0
    while (i < 10):
        r = interface.getSensorValue(sonar)[0]
        readings.append(r)
        i += 1
    #print(readings)
    return max(set(readings), key = readings.count) + 2.5

def stop():
    interface.setMotorRotationSpeedReference(motor,0.01)
    interface.setMotorPwm(motor,0)
    return

class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);
            
canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();
        
# FILL IN: spin robot or sonar to capture a signature and store it in ls
def characterize_location(ls):
    no = len(ls.sig)
    angle = interface.getMotorAngle(sonar)[0]
    for i in range(no):
        if (i == no / 2):
            interface.increaseMotorAngleReference(motor, -(math.pi * 2))
            while not interface.motorAngleReferenceReached(motor) :
                time.sleep(0.1)
            time.sleep(2)
            newangle = interface.getMotorAngle(sonar)[0]
            print math.degrees(abs(angle - newangle))
            angle = newangle
        ls.sig[i] = (readSonar(), i * (math.pi * 2) / no)
        interface.increaseMotorAngleReference(motor, (math.pi * 2) / no)
        while not interface.motorAngleReferenceReached(motor) :
            time.sleep(0.1)
        #stop()
        time.sleep(1)
        newangle = interface.getMotorAngle(sonar)[0]
        print math.degrees(abs(angle - newangle))
        print ls.sig[i][0]
        angle = newangle
        x = 138
        y = 54
        l, a = ls.sig[i][0], ls.sig[i][1]
        X = (math.cos(a) * l) + x
        Y = (math.sin(a) * l) + y
        canvas.drawLine((x, y, X, Y))
    #interface.increaseMotorAngleReference(motor, -(math.pi * 2))
    #while not interface.motorAngleReferenceReached(motor) :
        #time.sleep(0.01)
    return

def print_loc(ls, x, y):
    for i in range(len(ls.sig)):
        l, a = ls.sig[i][0], ls.sig[i][1]
        X = (math.cos(a) * l) + x
        Y = (math.sin(a) * l) + y
        canvas.drawLine((x, y, X, Y))
    return
    

# FILL IN: compare two signatures
def compare_signatures(ls1, ls2):
    dist = 0
    for i in range(len(ls2.sig)):
        dist += (ls1.sig[i][0] - ls2.sig[i][0])**2
    return dist

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    #print_loc(ls, 84, 30)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = LocationSignature();
    characterize_location(ls_obs);
    lowest_dist = 30000000
    lowest_idx = -1

    # FILL IN: COMPARE ls_read with ls_obs and find the best match
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx);
        dist    = compare_signatures(ls_obs, ls_read)
        if(dist < lowest_dist):
            lowest_dist = dist
            lowest_idx = idx
    return lowest_idx
            

# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files(). 
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

signatures = SignatureContainer(5);
#signatures.delete_loc_files()

learn_location();
#recognize_location();


