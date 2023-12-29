
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.init_fuzzy_controller()

    def init_fuzzy_controller(self):
        # Antecedents (Inputs)
        universe = np.arange(0, 11, 0.01)
        power_universe = np.arange(-0.3, 0.3, 0.01)
        self.sensor_right = ctrl.Antecedent(universe,'right_sensor')
        self.sensor_front = ctrl.Antecedent(universe,'front_sensor')

        # Auto-membership function population
        

        self.sensor_right['close'] = fuzz.trimf(self.sensor_right.universe,[0,0,0.6])
        self.sensor_right['good'] = fuzz.trimf(self.sensor_right.universe,[0.5,0.5,0.7])
        self.sensor_right['far'] = fuzz.trimf(self.sensor_right.universe,[0.6,10,11])

        self.sensor_front['close'] = fuzz.trimf(self.sensor_front.universe,[0,0.5,0.5])
        self.sensor_front['good'] = fuzz.trimf(self.sensor_front.universe,[0.4,0.6,0.7])
        self.sensor_front['far'] = fuzz.trimf(self.sensor_front.universe,[0.65,10,11])

        # Consequents (Outputs)
        self.speed_left = ctrl.Consequent(power_universe, 'left_speed')
        self.speed_right = ctrl.Consequent(power_universe, 'right_speed')

        self.speed_right['inverse'] = fuzz.trimf(self.speed_right.universe, [-0.5, -0.15, 0])
        self.speed_right['stop'] = fuzz.trimf(self.speed_right.universe, [-0.01, 0, 0.01])
        self.speed_right['front'] = fuzz.trimf(self.speed_right.universe, [0, 0.15, 0.5])

        self.speed_left['inverse'] = fuzz.trimf(self.speed_left.universe, [-0.5, -0.15, 0])
        self.speed_left['stop'] = fuzz.trimf(self.speed_left.universe, [-0.01, 0, 0.01])
        self.speed_left['front'] = fuzz.trimf(self.speed_left.universe, [0, 0.15, 0.5])

        # Fuzzy rules
        rule1 = ctrl.Rule(self.sensor_front['close'], (self.speed_left['inverse'], self.speed_right['front']))

        rule2 = ctrl.Rule(self.sensor_right['close'], (self.speed_right['front'], self.speed_left['stop']))

        rule3 = ctrl.Rule(self.sensor_right['far'], (self.speed_left['front'], self.speed_right['stop']))

        rule4 = ctrl.Rule(self.sensor_right['good'] & self.sensor_front['far'], (self.speed_left['front'], self.speed_right['front']))
        rule5 = ctrl.Rule(self.sensor_right['good'] & self.sensor_front['good'], (self.speed_left['front'], self.speed_right['front']))
        rule6 = ctrl.Rule(self.sensor_right['good'] & self.sensor_front['close'], (self.speed_left['inverse'], self.speed_right['front']))




        # Control system
        self.fuzzy_control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
        self.fuzzy_control = ctrl.ControlSystemSimulation(self.fuzzy_control_system)

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

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()
            front = self.measures.irSensor[0]
            right = self.measures.irSensor[2]


            if front > 0:
                front_dist = 1/(front)
            else:
                front_dist = 10

            if right > 0:
                right_dist = 1/(right)
            else:
                right_dist = 10
            



            # Fuzzy logic for wall following and obstacle avoidance
            self.fuzzy_control.input['right_sensor'] = right_dist  # right sensor
            self.fuzzy_control.input['front_sensor'] = front_dist  # front sensor
            self.fuzzy_control.compute()

            left_wheel_speed = self.fuzzy_control.output['left_speed']
            right_wheel_speed = self.fuzzy_control.output['right_speed']

            self.driveMotors(left_wheel_speed, right_wheel_speed)
            

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
