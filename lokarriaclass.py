"""
Commands a robot to follow a given path in Microsoft Robotic Developer
Studio 4 with the Lokarria interface.

Author: Adam Kavanagh Coyne

Quaternion functions and additional formatting taken from example code by Erik Billing given in the AI class from Umea University.

NOTE: This is the Python 3 version. To work in Python 2, all instances of the "http.client" package should be replaced by "httplib".
"""

import http.client, json, time
from math import sin,cos,pi,atan2

URL = 'localhost:50000'
HEADERS = {"Content-type": "application/json", "Accept": "text/json"}
FILENAME = 'Path-to-bed.json'

#goToPoint() constants
BEARING_THRESHOLD = 0.1
DISTANCE_THRESHOLD = 0.1
GOTOPOINT_ANG_SPEED = 1
GOTOPOINT_LIN_SPEED = 0.5

class UnexpectedResponse(Exception): pass

class Robot(object):
    """Represents the robot to facilitate function calls and code readability.

    Attributes:
        server: a string for the url of the MRDS server (running the simulation)
        path: the series of coordinates read from JSON file dictating path
        x: the x spatial coordinate for the position of the robot
        y: the y spatial coordinate
        z: the z spatial coordinate
        heading: the heading of the robot (angle)
    """

    def __init__(self, server, pathFilename):
        """Return a Robot after reading and storing both server URL and JSON path"""
        self.server = server
        self.path = openJsonTrajectory(pathFilename)
        self.updateAttributes()

    def serverGet(self, command):
        """Sends a GET request to the MRDS server
        :param command: the specific directory requested
        """
        mrds = http.client.HTTPConnection(self.server)
        mrds.request('GET',command)
        return mrds.getresponse()

    def serverJsonPost(self, json):
        """Sends a JSON POST command to the MRDS server (used for defining speeds)"""
        mrds = http.client.HTTPConnection(self.server)
        mrds.request('POST','/lokarria/differentialdrive',json,HEADERS)
        return mrds.getresponse()
    
    def setAngularSpeed(self, speed):
        """Sets angular speed of the bot by sending a command to the server"""
        param = json.dumps({'TargetAngularSpeed':speed})
        response = self.serverJsonPost(param)
        status = response.status
        if status == 204:
            return response
        else:
            return UnexpectedResponse(response)

    def setLinearSpeed(self, speed):
        """Sets linear speed of the bot by sending a command to the server"""
        param = json.dumps({'TargetLinearSpeed':speed})
        response = self.serverJsonPost(param)
        status = response.status
        if status == 204:
            return response
        else:
            return UnexpectedResponse(response)

    def setSpeed(self,angularSpeed,linearSpeed):
        """Sets speed of the bot by sending a command to the server"""
        param = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
        response = self.serverJsonPost(param)
        status = response.status
        if status == 204:
            return response
        else:
            return UnexpectedResponse(response)

    def getLaser(self):
        """Returns the laser properties from the server as a dictionary"""
        response = self.serverGet('/lokarria/laser/properties')
        if (response.status == 200):
            laserData = response.read()
            response.close()
            properties = json.loads(laserData)
            beamCount = int((properties['EndAngle']-properties['StartAngle'])/properties['AngleIncrement'])
            a = properties['StartAngle']#+properties['AngleIncrement']
            angles = []
            while a <= properties['EndAngle']:
                angles.append(a)
                a+=pi/180 #properties['AngleIncrement']
            #angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
            return angles
        else:
            return UnexpectedResponse(response)

    def getLaserAngles(self):
        #Do we need this?
        pass


    def getPose(self):
        """Reads the current position and orientation from the MRDS"""
        response = self.serverGet('/lokarria/localization')
        if (response.status == 200):
            poseStr = response.readall().decode('utf-8')
            response.close()
            return json.loads(poseStr)
        else:
            return UnexpectedResponse(response)
        
    def updateAttributes(self):
        """Updates the robot pose attributes"""
        #Attributes to update: x, y, z, heading. After, laser?
        try:
            pose = self.getPose()
            self.heading = toHeading(pose['Pose']['Orientation'])
            self.x = pose['Pose']['Position']['Y']
            self.y = pose['Pose']['Position']['Y']
        except (ConnectionRefusedError):
            print("Connection to server refused. Are you sure you have the right address?")

    def getBearing(self, coordinates):
        dX = coordinates['X']-self.x
        dY = coordinates['Y']-self.y
        angle = atan2(dY/dX)
        return (angle-self.heading)

    def distanceTo(self, coordinates):
        """Returns the distance between the robot and a given point
        :param p1: The point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively
        """
        dx = coordinates['X']-self.x
        dy = coordinates['Y']-self.y
        return sqrt(dx^2+dy^2)
        
    def lookAhead(self, distance):
        """Returns the coordinates of a point at a given distance in front of the robot (as a dict)"""
        yp = self.y - distance * cos(self.heading)
        xp = self.x - distance * sin(self.heading)
        point={'X':xp,'Y':yp}
        return point
        
    def goToPoint(self, coordinates):
        """Moves the robot in a straight line to a given point. Follows a 2-step system; 
        1. Orient heading towards point. 2. Move in straight line; Once in close range of point, stop.
        May be inefficient; no new process, spams server requests for data...
        Will stop if it starts moving away from point.
        Returns 0 if point reached (with a tolerance of DISTANCE_THRESHOLD), or 1 if robot was detected moving away from point.
        :param p1: The point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively.
        """
        #Part 1: Turn to aim at point
        self.setSpeed(0,0)
        self.updateAttributes()
        bearing = self.getBearing(coordinates)
        if (bearing > BEARING_THRESHOLD or bearing < 2*pi()-BEARING_THRESHOLD):
            if (bearing > pi()):
                self.setAngularSpeed(GOTOPOINT_ANG_SPEED)
            else:
                self.setAngularSpeed(-GOTOPOINT_ANG_SPEED)
            while (self.getBearing(coordinates) > BEARING_THRESHOLD or self.getBearing(coordinates) < 2*pi()-BEARING_THRESHOLD): #While the robot isn't pointing at dest
                self.updateAttributes()
        #Is pointing ok, stop turning
        self.setSpeed(0,0)
            
        #Part 2: Move forward until point is reached
        if (self.distanceTo(coordinates) > DISTANCE_THRESHOLD):
            self.setLinearSpeed(GOTOPOINT_LIN_SPEED)
            while (self.distanceTo(coordinates) > DISTANCE_THRESHOLD and previousDistance >= self.distanceTo(coordinates)):
                #Wait 100ms here maybe?
                previousDistance = self.distanceTo(coordinates)
                self.updateAttributes()
            if (previousDistance < self.distanceTo(coordinates)):
                print("Started moving away from point, stopping")
                result = 1
            else:
                print("Distance threshold reached, stopping")
                result = 0
        else:
            print("Already close to point")
            result = 0
        #Stop moving
        self.setSpeed(0,0)
        return result


    
def distanceBetween(p1,p2):
    """Returns the distance between two points
    :param p1: The first point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively 
    :param p2: The second point (identical system to p1)
    """
    dx = p1['Y']-p2['Y']
    dy = p1['X']-p2['X']
    return sqrt(dx^2+dy^2)
    
def toHeading(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q

def openJsonTrajectory(file):
    with open(file) as f:
        content = f.read()
        return json.loads(content)
    
print("Compiled!")

r1 = Robot(URL,FILENAME)
print("Robot made.")
print("Initial coordinate read from file:")

print(r1.path[1])

# Hey man, here's an example of how to go through the path data structure ;)
# print(r1.path[2]['Pose']['Position']['X'])

