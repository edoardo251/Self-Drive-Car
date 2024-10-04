#python

import math
import heapq
import csv
   
 
def shortest_paths(adjacencies, dummies, start, stop):
    def dijkstra(start, stop):
        distances = {point: float('inf') for point in adjacencies}
        distances[start] = 0
        queue = [(0, start)]
        previous = {point: None for point in adjacencies}
         
        while queue:
            current_distance, current_point = heapq.heappop(queue)
            if current_point == stop:
                break
            
            for neighbor in adjacencies[current_point]:
                new_distance = current_distance + distance(dummies[current_point], dummies[neighbor])
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current_point
                    heapq.heappush(queue, (new_distance, neighbor))
        
        path = []
        current_point = stop
        while current_point is not None:
            path.insert(0, current_point)
            current_point = previous[current_point]
        
        return path, distances[stop]
    
    # Shortest path calculate in both directions
    path_start_to_stop, distance_start_to_stop = dijkstra(start, stop)
    path_stop_to_start, distance_stop_to_start = dijkstra(stop, start)
    
    if distance_start_to_stop <= distance_stop_to_start:
        return path_start_to_stop, distance_start_to_stop
    else:
        return path_stop_to_start[::-1], distance_stop_to_start


def distance(point1, point2):
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5


def loadGlobalVariables():
    # From/to point
    self.startPoint = ""
    self.stopPoint = ""
    
    # Vehicle properties
    self.coordZ = 0.08328
    self.velocity = 6.0
    
    # Static global variables
    self.pointRangeValue = 0.08
    
    # Global variables
    self.index = 0
    self.initFinished = False
    self.changeDirection = False 
    self.destinationReached = False
    self.orientation = []
    self.newOrientation = []
    self.orientationName = ''
    self.newOrientationName = ''
    self.isChanging = False
    self.retro = False
    self.pathAlreadyRemoved = False
    self.proximitySensorEnabled = True
    self.dati = []
    
    # Get object handles
    self.carObject = sim.getObject(".")
    self.motorLeft = sim.getObject("./DynamicLeftJoint")
    self.motorRight = sim.getObject("./DynamicRightJoint")
    self.proximitySensor = sim.getObject("./Proximity_sensor")
    
    self.alpha = {
        "0": "A",
        "1": "B",
        "2": "C",
        "3": "D",
        "4": "E",
        "5": "F",
        "6": "G",
        "7": "H",
        "8": "I",
        "9": "J",
        "10": "H",
        "11": "L",
        "12": "M",
        "13": "N",
        "14": "O",
        "15": "P"
    }
    
    self.adjacencies = {
        "A": ["B"],
        "B": ["C", "E"],
        "C": ["B", "D", "H"],
        "D": ["C"],
        "E": ["B", "F"],
        "F": ["E", "J"],
        "G": ["H"],
        "H": ["C", "G", "I", "L"],
        "I": ["H", "O", "J"],
        "J": ["F", "I", "P"],
        "K": ["L"],
        "L": ["H", "K", "N"],
        "M": ["N"],
        "N": ["M", "L"],
        "O": ["I", "P"],
        "P": ["O", "J"]
    }

    self.dummies = {
        "A": [2.250, 2.250],
        "B": [2.250, 0.250],
        "C": [2.250, -1.250],
        "D": [2.250, -2.350],
        "E": [1.250, 0.250],
        "F": [1.250, 2.250],
        "G": [0.250, -2.350],
        "H": [0.250, -1.250],
        "I": [0.250, 0.250],
        "J": [0.250, 2.250],
        "K": [-1.250, -2.350],
        "L": [-1.250, -1.250],
        "M": [-2.250, -2.350],
        "N": [-2.250, -1.250],
        "O": [-2.250, 0.250],
        "P": [-2.250, 2.250]
    }
    
    # Initial conditions
    setVelocity(0.0, 0.0)
        
    
def calculateDijkstra():
    self.points, self.distances = shortest_paths(self.adjacencies, self.dummies, self.startPoint, self.stopPoint)
    
    # Initial vehicles properties (position and velocity)
    initial_x = self.dummies[self.points[0]][0]
    initial_y = self.dummies[self.points[0]][1]
    first_point_x = self.dummies[self.points[1]][0]
    first_point_y = self.dummies[self.points[1]][1]
    
    self.initialPosition = [initial_x, initial_y, self.coordZ]
    setPosition(self.carObject, self.initialPosition)
    self.orientation = getNextOrientation([initial_x, initial_y], [first_point_x, first_point_y])[0]
    self.orientationName = getNextOrientation([initial_x, initial_y], [first_point_x, first_point_y])[1]
    
    setOrientation(self.carObject, self.orientation)


def getPosition(object):
    [x, y, z] = sim.getObjectPosition(object)
    return [x, y, z]
    
    
def setPosition(object, positions):
    sim.setObjectPosition(object, positions)


def getVelocity(object):
    velocity = sim.getJointTargetVelocity(object)
    return velocity


def setVelocity(vLeft, vRight):
    sim.setJointTargetVelocity(self.motorLeft, vLeft)
    sim.setJointTargetVelocity(self.motorRight, vRight)
    
    
def getOrientation(object):
    orientation = sim.getObjectOrientation(object, -1)
    return orientation
    

def setOrientation(object, angle):
    orientation = getOrientation(object)
    new_orientation = [math.radians(angle[0]), math.radians(angle[1]), math.radians(angle[2])]
    sim.setObjectOrientation(object, -1, new_orientation)
    
    
def checkOrientationRange(coords, orientation):    
    if(coords[1]+0.550 > orientation[1] and coords[1] -0.550 < orientation[1]):
        return True
    else:
        return False
    

def getNextOrientation(point, newPoint):
    x = newPoint[0] - point[0]
    y = newPoint[1] - point[1]
    if(x == 0):
        if(y > 0):
            return [90.00, 0, 90.00], "top" # Sopra
        else:
            return [-90.00, 0, -90.00], "bottom" # Sotto
    elif(y == 0):
        if(x > 0):
            return [90, -90, 90], "right" # Destra
        else:
            return [90, 90, 90.00], "left" # Sinistra
            

def oppositeOrientation(orientation):
    opposite = {
        "top": [-90.00, 0, -90.00],
        "bottom": [90.00, 0, 90.00],
        "right": [0.00, 90, 180.00],
        "left": [0.00, -90, 0.00]
    }
    return opposite[orientation]


def checkRange(x, y, point):
    if(x <= point[0]+self.pointRangeValue and x >=point[0]-self.pointRangeValue and y <= point[1]+self.pointRangeValue and y >=point[1]-self.pointRangeValue):
        return True
    else:
        return False
        
        
def checkRotationWheels(currentOrientation, nextOrientation, velocity, retro):
    if nextOrientation == currentOrientation:
        return setVelocity(self.velocity, self.velocity)
    elif currentOrientation == "top":
        if nextOrientation == "left":
            if retro:
                return setVelocity(-velocity, velocity)
            else:
                return setVelocity(0.1, velocity)
        else:
            if retro:
                return setVelocity(velocity, -velocity)
            else:
                return setVelocity(velocity, 0.1)
    elif currentOrientation == "right":
        if nextOrientation == "top":
            if retro:
                return setVelocity(-velocity, velocity)
            else:
                return setVelocity(0.1, velocity)
        else:
            if retro:
                return setVelocity(velocity, -velocity)
            else:
                return setVelocity(velocity, 0.1)
    elif currentOrientation == "bottom":
        if nextOrientation == "right":
            if retro:
                return setVelocity(-velocity, velocity)
            else:
                return setVelocity(0.1, velocity)
        else:
            if retro:
                return setVelocity(velocity, -velocity)
            else:
                return setVelocity(velocity, 0.1)
    elif currentOrientation == "left":
        if nextOrientation == "bottom":
            if retro:
                return setVelocity(-velocity, velocity)
            else:
                return setVelocity(0.1, velocity)
        else:
            if retro:
                return setVelocity(velocity, -velocity)
            else:
                return setVelocity(velocity, 0.1)
        
        
def appendPoint(coordinates, pathPoints):
    pathPoints.append(coordinates[0])
    pathPoints.append(coordinates[1])
    pathPoints.append(0)
    pathPoints.append(0)
    pathPoints.append(0)
    pathPoints.append(0)
    pathPoints.append(1)
    return pathPoints


def generatePath():
    pathPoints = []
    for element in self.points:
        coordinate = self.dummies[element]
        appendPoint(coordinate, pathPoints)
    self.pathHandle = sim.createPath(pathPoints, 0, 100, 0)
    self.pathAlreadyRemoved = False
    

def removePath():
    sim.removeObjects([sim.getObject('../Path')])
    for k in reversed(range(0, len(self.points))):
        nodeName = f"../ctrlPt[{k}]"
        sim.removeObjects([sim.getObject(nodeName)])
    self.pathAlreadyRemoved = True
    

def appendDataRow(precPoint, nextPoint, position, orientation, nextOrientation, event):
    position = [round(position[0], 5), round(position[1], 5), round(position[2], 5)]
    orientation = [round(orientation[0], 5), round(orientation[1], 5), round(orientation[2], 5)]
    self.dati.append({
        "PrecPoint": precPoint,
        "NextPoint": nextPoint,
        "Position": position,
        "Orientation": orientation,
        "NextOrientation": nextOrientation,
        "Event": event
    })


def saveDataCSV():
    CSVFileName = 'selfDriveAutoData.csv'
    headers = ["PrecPoint", "NextPoint", "Position", "Orientation", "NextOrientation", "Event"]

    CSVFileNotExists = not os.path.exists(CSVFileName) or os.path.getsize(CSVFileName) == 0
    with open(CSVFileName, mode='a+', newline='') as file_csv:
        writer = csv.DictWriter(file_csv, fieldnames=headers)
        if CSVFileNotExists:
            writer.writeheader()
        writer.writerows(self.dati)


def sysCall_init():
    sim = require('sim')


def sysCall_actuation():
    sim = require('sim')
    
    startSimulation = sim.getStringSignal("startSimulation").decode("utf-8")
    
    # Init
    if startSimulation == 'True' and self.initFinished == False:
        self.startPoint = sim.getStringSignal("startPoint").decode("utf-8")
        self.stopPoint = sim.getStringSignal("stopPoint").decode("utf-8")
        calculateDijkstra()
        generatePath()
        setVelocity(self.velocity, self.velocity)
        self.initFinished = True
    
    # Actuation
    if startSimulation == 'True':    
        if self.destinationReached == False:
            precPoint = self.points[self.index]
            nextPoint = self.points[self.index+1]

            coords_prec = self.dummies[precPoint]
            coords_next = self.dummies[nextPoint]   
       
            [x, y, z] = getPosition(self.carObject)  
            [xPos, yPos, zPos] = getOrientation(self.carObject)
            [a, b, c] = getNextOrientation(coords_prec, coords_next)[0]
            oppositeDirection = getNextOrientation(coords_prec, coords_next)[1]
            
            # Proximity Sensor
            wallDetected,_,_,_,_ = sim.handleProximitySensor(self.proximitySensor)
            if wallDetected == 1 and self.proximitySensorEnabled:
                print(">> Wall detected")
                self.proximitySensorEnabled = False
                if self.pathAlreadyRemoved == False:
                    removePath()
                setVelocity(0.0, 0.0)
                appendDataRow(precPoint, nextPoint, [x, y, z], [xPos, yPos, zPos], [a, b, c], "Wall detected")
                if(checkOrientationRange([xPos, math.degrees(yPos), zPos], self.newOrientation)):
                    setVelocity(-self.velocity, -self.velocity)
                    self.retro = True
                
            # Retro
            if self.retro and checkRange(x,y, coords_prec):
                self.proximitySensorEnabled = True
                setVelocity(0.0, 0.0)
                self.retro = False
                self.index = 0
               
                if nextPoint in self.adjacencies[precPoint]:
                    self.adjacencies[precPoint].remove(nextPoint)
                if precPoint in self.adjacencies[nextPoint]:
                    self.adjacencies[nextPoint].remove(precPoint)
                self.points, self.distances = shortest_paths(self.adjacencies, self.dummies, precPoint, self.stopPoint)
                print(self.points)
                generatePath()
                
                precPoint = self.points[self.index]
                nextPoint = self.points[self.index+1]

                coords_prec = self.dummies[precPoint]
                coords_next = self.dummies[nextPoint]  
                
                self.orientation = self.newOrientation
                self.orientationName = self.newOrientationName  
                self.newOrientation = getNextOrientation(coords_prec, coords_next)[0]
                self.newOrientationName = getNextOrientation(coords_prec, coords_next)[1]
                checkRotationWheels(self.orientationName, self.newOrientationName, 0.78, True)
                self.isChanging = True

            # A dummy was reached
            if checkRange(x, y, coords_next):
                print(f">> Intersection: ({precPoint}, {nextPoint})")
                self.orientation = getNextOrientation(coords_prec, coords_next)[0]
                self.orientationName = getNextOrientation(coords_prec, coords_next)[1]
                self.index += 1
                self.changeDirection = True
                appendDataRow(precPoint, nextPoint, [x, y, z], [xPos, yPos, zPos], [a, b, c], "Point acrossed")
                
                # Is destination reached?
                if self.index == len(self.points)-1:
                    print(">> Final destination reached")
                    setVelocity(0.0, 0.0)
                    self.destinationReached = True
                    if self.pathAlreadyRemoved == False:
                        removePath()
                    appendDataRow(precPoint, nextPoint, [x, y, z], [xPos, yPos, zPos], [a, b, c], "Final destination reached")
                    saveDataCSV()
                    sim.pauseSimulation()
                    
            elif self.changeDirection == True:
                self.newOrientation = getNextOrientation(coords_prec, coords_next)[0]
                self.newOrientationName = getNextOrientation(coords_prec, coords_next)[1]
                checkRotationWheels(self.orientationName, self.newOrientationName, 0.8, False)
                self.isChanging = True
                self.changeDirection = False
                
            if self.isChanging and checkOrientationRange([xPos, math.degrees(yPos), zPos], self.newOrientation):
                setVelocity(self.velocity, self.velocity)
                self.isChanging = False


def sysCall_nonSimulation():
    pass

def sysCall_beforeSimulation():
    loadGlobalVariables()
    
def sysCall_afterSimulation():
    if self.pathAlreadyRemoved == False:
        removePath()
    
def sysCall_cleanup():
    pass
    
def sysCall_cleanup():
    pass