
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()
        
        self.readSensors()
        self.ini_gps_x = self.measures.x
        self.ini_gps_y = self.measures.y
        print(self.measures)
        while not self.measures.start:
            self.readSensors()

        while True:

            orientation = self.checkWalls()
            if orientation == 'front':
                self.driveMotors(0.5, 0.5)
            if orientation == 'turn_left':
                self.driveMotors(0.0, 0.1)
            if orientation == 'turn_right':
                self.driveMotors(0.1, 0.0)
            if orientation == 'front_wall':
                self.driveMotors(0.0, 0.0)
                self.driveMotors(-0.5, 0.5)    
            if orientation == 'Nothing':
                self.driveMotors(0.0, 0.0)




    def sensorValues(self):
        self.readSensors()
        return [1/ value if value != 0 else 0 for value in self.measures.irSensor]

    def checkWalls(self):
        front, left, right, back = self.sensorValues()
        print("--------------------")

        print(self.measures.compass)
        print("--------------------")
        print(self.measures.compassReady)
        print('front', front, '\n', 'left', left, '\n', 'right' ,right, '\n', 'back', back)
        compass = self.measures.compass
        if front <= 2 and right <= 0.4:
            return 'front_wall'
        elif right > 0.45 and right < 0.35:
            return 'front'
        elif right > 0.45 :
            return 'turn_right'
        elif right < 0.35 :
            return 'turn_left'
        

        if front > 0.6 and left > 0.6 and right > 0.6 and back > 0.6:
            return 'Nothing'
        
        return 'front'
        
        

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
