"""
Commands a robot to follow a given path in Microsoft Robotic Developer
Studio 4 with the Lokarria interface.
Authors: Adam Kavanagh Coyne & Shanmuganathan Sabarisathasivam
Quaternion functions and additional formatting taken from example code by Erik Billing given in the AI class from Umea University.
NOTE: This is the Python 2.7 version. To work in Python 3, all instances of the "httplib" package should be replaced by "httplib".
"""

import httplib, json, time
from math import sin,cos,pi,atan2,asin,sqrt,acos,exp

## Interfacing constants
URL = 'localhost:50000'
HEADERS = {"Content-type": "application/json", "Accept": "text/json"}
FILENAME = 'Path-around-table-and-back.json'

## Physical constants
DISTANCE_THRESHOLD = 0.5 # The distance the robot has to be from a point for it to be considered "reached".
GOTOPOINT_ANG_SPEED = 3 # Maximum angular speed the robot can move at (is scaled down based on bearing, see coefficientfunc)
GOTOPOINT_LIN_SPEED = 1 # Maximum linear speed the robot can move at (is reduced as angular speed increases)
TICK_DURATION=0.01 # Time between each algorithm loop. Too short a tick can saturate the socket used to communicate with the robot.
LOOK_AHEAD_DISTANCE = 1.5 # Distance in front of robot to search for next carrot point (see report for details)
LA_THRESHOLD=400 # Look-ahead threshold: maximum range of points in front of current in which to search for carrot point (to avoid overzealous corner cuttinig)


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
        """Return a Robot after reading and storing both server URL and JSON path, then updating internal attributes"""
        self.server = server
        self.path = openJsonTrajectory(pathFilename)
        self.updateAttributes()

    def serverGet(self, command):
        """Sends a GET request to the MRDS server (used for getting attributes)
        :param command: the specific directory requested
        """
        mrds = httplib.HTTPConnection(self.server)
        mrds.request('GET',command)
        return mrds.getresponse()

    def serverJsonPost(self, json):
        """Sends a JSON POST command to the MRDS server (used for setting speeds)"""
        mrds = httplib.HTTPConnection(self.server)
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
        """Reads the current position and orientation from the MRDS
        Returns a dictionary of pose data.
        """
        response = self.serverGet('/lokarria/localization')
        if (response.status == 200):
            poseStr = response.read()
            response.close()
            return json.loads(poseStr)
        else:
            return UnexpectedResponse(response)
        
    def updateAttributes(self):
        """Updates the robot pose attributes, heading and coordinates, not speeds."""
        pose = self.getPose()
        self.heading = toHeading(pose['Pose']['Orientation'])
        self.x = pose['Pose']['Position']['X']
        self.y = pose['Pose']['Position']['Y']

    def getBearing(self, coordinates):
        """Gets the angle in radians between robot's current heading and heading defined by point
        :param p1: The point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively
        """
        dX = coordinates['X']-self.x
        dY = coordinates['Y']-self.y
        angle = atan2(dY,dX)
        bearing = angle-self.heading
        if bearing > pi:
            return bearing-2*pi
        elif bearing < -pi:
            return bearing+2*pi
        else:
            return bearing

    def distanceTo(self, coordinates):
        """Returns the distance between the robot and a given point
        :param p1: The point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively
        """
        self.updateAttributes()
        dx = coordinates['X']-self.x
        dy = coordinates['Y']-self.y
        return sqrt(pow(dx,2)+pow(dy,2))
        
    def lookAhead(self, distance):
        """Returns the coordinates of a point at a given distance in front of the robot (as a dict)"""
        yp = self.y + distance * cos(self.heading-pi/2)
        xp = self.x - distance * sin(self.heading-pi/2)
        point={'X':xp,'Y':yp}
        return point
        
    def goToPoint(self, coordinates):
        """Adjusts the robot's trajectory to head towards a point; a continuous adaptation of the previous 2-step version goToPoint
        Made to be looped for every point.
        :param p1: The point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively.
        """
        self.updateAttributes()
        bearing = self.getBearing(coordinates)
        distance = self.distanceTo(coordinates)
        if bearing>0: 
            angularSpeed=coefficient(bearing)*GOTOPOINT_ANG_SPEED
        else:
            angularSpeed=-coefficient(bearing)*GOTOPOINT_ANG_SPEED
        self.setSpeed(angularSpeed,GOTOPOINT_LIN_SPEED)
        

def distanceBetween(p1,p2):
    """Returns the distance between two points
    :param p1: The first point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively 
    :param p2: The second point (identical system to p1)
    """
    dx = p1['Y']-p2['Y']
    dy = p1['X']-p2['X']
    return sqrt(pow(dx,2)+pow(dy,2))

def heading(q):
    """Derives angle cosines from quaternion orientation (which is what is given by MRDS)
    Functions rotate, vector, qmult, quaternion, conjugate are all used here for this task.
    All quaternion functions were provided in sample code for assignment.
    """
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
    """Opens a JSON file (trajectory) and returns data as Python object"""
    with open(file) as f:
        content = f.read()
        return json.loads(content)
        
def toHeading(q):
    """Converts quaternion to XY-plane Euler angle in radians"""
    v=heading(q)
    angle=atan2(v['Y'],v['X'])
    # Angle normalisation
    if angle>0:
        return angle
    else:
        return 2*pi+angle

def coefficient(bearing):
    """Returns a ratio used to scale down angular speed based on bearing"""
    return 1-exp(-2*bearing**2)
       
    
    
    
if __name__ == "__main__":
    
    r1 = Robot(URL,FILENAME)
    print("Robot made.")

    r1.setSpeed(0,0)                    #Set intial angular and linear velocity to zero
    print('Robot intial coordinates and heading: X: ',r1.x,' Y: ',r1.y,' Heading: ',r1.heading)

    # Follow the predefined path
    print "Timer started"
    time_start=time.time() 
    
    prevPointIndex=0 # Index of first point is zero
    atDestination=False
    carrotPoint=r1.path[prevPointInd]['Pose']['Position'] # First point
    print "Path following started with path: ", FILENAME
    
    while (not atDestination):
        lookAheadPoint=r1.lookAhead(LOOK_AHEAD_DISTANCE) # The point a certain distance ahead of the robot
        time.sleep(TICK_DURATION)

        prevPointDistance=distanceBetween(r1.path[prevPointIndex]['Pose']['Position'], lookAheadPoint)
        carrotPoint=r1.path[prevPointIndex]['Pose']['Position']
        currentPointIndex=prevPointIndex
        
        # Get range of path points to scan (as many as path will allow up to LA_THRESHOLD points ahead of current point)
        if len(r1.path)<prevPointIndex+LA_THRESHOLD:
            lastIndex = len(r1.path)-1 # Scan points until end of path
            if r1.distanceTo(r1.path[-1]['Pose']['Position'])<DISTANCE_THRESHOLD: # if we are within DISTANCE_THRESHOLD from the final point
                atDestination=True
        else:
            lastIndex = prevPointIndex+LA_THRESHOLD # Scan up to LA_THRESHOLD points ahead of current point
            
        # Scan points for closest in path to lookAheadPoint, track its coordinates and index
        prevPointDistance=distanceBetween(r1.path[prevPointIndex]['Pose']['Position'], lookAheadPoint)
        carrotPoint=r1.path[prevPointIndex]['Pose']['Position']
        carrotPointIndex=prevPointIndex
        
        for i in range(prevPointIndex,lastIndex):
            distance=distanceBetween(r1.path[i]['Pose']['Position'], lookAheadPoint)
            if distance < prevPointDistance:
                carrotPoint=r1.path[i]['Pose']['Position']
                carrotPointIndex=i
                prevPointDistance=distance
                
        prevPointIndex=carrotPointIndex
        
        # Adjust trajectory towards this point
        r1.goToPoint(carrotPoint)
    
    # Destination Reached
    time_traversed = time.time()- time_start                    # Stop the Timer
    print "Path following completed successfully"
    print "Time taken for following the path = ",time_traversed
    r1.setSpeed(0,0)                                            # Stop the robot

