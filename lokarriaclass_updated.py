"""
Commands a robot to follow a given path in Microsoft Robotic Developer
Studio 4 with the Lokarria interface.

Authors: Adam Kavanagh Coyne & Shanmuganathan Sabarisathasivam

Quaternion functions and additional formatting taken from example code by Erik Billing given in the AI class from Umea University.

NOTE: This is the Python 2.7 version. To work in Python 3, all instances of the "httplib" package should be replaced by "httplib".
"""

import httplib, json, time
from math import sin,cos,pi,atan2,asin,sqrt,acos


URL = 'localhost:50000'
HEADERS = {"Content-type": "application/json", "Accept": "text/json"}
FILENAME = 'Path-to-bed.json'


#goToPoint() constants
BEARING_THRESHOLD = 0.1
DISTANCE_THRESHOLD = 0.1
GOTOPOINT_ANG_SPEED = 0.1
GOTOPOINT_LIN_SPEED = 0.1

##px=[0.5,0.5,0,0.5,0,0.5,0,0]
##py=[0,0.5,1,1.5,1.5,1,0.5,0]

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
##        print self.path
        self.updateAttributes()

    def serverGet(self, command):
        """Sends a GET request to the MRDS server
        :param command: the specific directory requested
        """
        mrds = httplib.HTTPConnection(self.server)
        mrds.request('GET',command)
        return mrds.getresponse()

    def serverJsonPost(self, json):
        """Sends a JSON POST command to the MRDS server (used for defining speeds)"""
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
        """Reads the current position and orientation from the MRDS"""
        response = self.serverGet('/lokarria/localization')
        if (response.status == 200):
            poseStr = response.read()
##            print('Response ok')
            response.close()
            return json.loads(poseStr)
        else:
            return UnexpectedResponse(response)
        
    def updateAttributes(self):
        """Updates the robot pose attributes"""
        #Attributes to update: x, y, z, heading. After, laser?
##        pose = self.getPose()
##        self.heading = toHeading(pose['Pose']['Orientation'])
####            self.heading = pose['Pose']['Orientation']
##        self.x = pose['Pose']['Position']['X']
##        self.y = pose['Pose']['Position']['Y']
        try:
            pose = self.getPose()
            self.heading = toHeading(pose['Pose']['Orientation'])
##            self.heading = pose['Pose']['Orientation']
            self.x = pose['Pose']['Position']['X']
            self.y = pose['Pose']['Position']['Y']
        except:
            print("updateAttributes Error: Connection to server refused. Are you sure you have the right address?")

    def getBearing(self, coordinates):
        dX = coordinates['X']-self.x
        dY = coordinates['Y']-self.y
        angle = atan2(dY,dX)
        if angle > 0:
##            if abs(angle-self.heading)< abs(2*pi-(angle-self.heading)):
                return (angle)
##            else:
##                return -(2*pi-(angle-self.heading))
        else:
##            if abs(2*pi+angle-self.heading)< abs(2*pi-(2*pi+angle-self.heading)):
                return(2*pi+angle)
##            else:
##                return -(2*pi-(2*pi+angle-self.heading))


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
        print coordinates
        self.setSpeed(0,0)
        self.updateAttributes()
        bearing = self.getBearing(coordinates)
        if abs(bearing - self.heading) > BEARING_THRESHOLD:
            if ((bearing > self.heading) and ((bearing-self.heading) < (2*pi-(bearing-self.heading)))) or ((bearing < self.heading)and ((self.heading-bearing) > (2*pi-(self.heading-bearing)))):
                self.setAngularSpeed(GOTOPOINT_ANG_SPEED)
            else:
                self.setAngularSpeed(-GOTOPOINT_ANG_SPEED)
            while (abs(self.getBearing(coordinates)-self.heading) > BEARING_THRESHOLD):  #While the robot isn't pointing at dest
                time.sleep(0.1)
                self.updateAttributes()
        #Is pointing ok, stop turning
        print 'Bearing matched'
##        print self.getBearing(coordinates)
        self.setSpeed(0,0)
        self.updateAttributes()
##        print(self.x,self.y)
##        print self.distanceTo(coordinates)
        #Part 2: Move forward until point is reached
        previousDistance = self.distanceTo(coordinates)
        if (self.distanceTo(coordinates) > DISTANCE_THRESHOLD):
            self.setLinearSpeed(GOTOPOINT_LIN_SPEED)
            while (self.distanceTo(coordinates) > DISTANCE_THRESHOLD):
##                print self.distanceTo(coordinates)
                #Wait 100ms here maybe?
                time.sleep(0.1)
                self.updateAttributes()
##            if (previousDistance < self.distanceTo(coordinates)):
##                print("Started moving away from point, stopping")
##                result = 1
##            else:
##                print("Distance threshold reached, stopping")
##                result = 0
        else:
            print("Already close to point")
        result = 0
        #Stop moving
        self.setSpeed(0,0)
        print 'carrot point reached'
        return result

    def chooseCarrotPoints(self):
        self.path[Pose][Position]
    

        


def distanceBetween(p1,p2):
    """Returns the distance between two points
    :param p1: The first point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively 
    :param p2: The second point (identical system to p1)
    """
    dx = p1['Y']-p2['Y']
    dy = p1['X']-p2['X']
    return sqrt(pow(dx,2)+pow(dy,2))

def heading(q):
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
    
def toHeading(q):
##        phi=(atan2((q['X']*q['Z']+q['Y']*q['W']), -((q['Y']*q['Z'])-(q['X']*q['W']))))*180/pi
##        thta=acos(-q['X']**2-q['Y']**2+q['Z']**2+q['W']**2)
##        psi=(atan2((q['X']*q['Z']-q['Y']*q['W']), ((q['Y']*q['Z'])+(q['X']*q['W']))))*180/pi
##        heading=q['Y']/sqrt(1-q['W']*q['W'])
##        bank=atan2(2*q['X']*q['W']-2*q['Y']*q['Z'],1-2*(q['X']**2)-2*(q['Z']**2))
##        if (quat >= 1):
##            quat=0.999999
##        elif (quat <= -1):
##            quat=-0.999999
##        heading=[phi,thta,psi]
    v=heading(q)
    angle= atan2(v['Y'],v['X'])
    if angle>0:
        return angle
    else:
        return 2*pi+angle

    
##def dist_Error():

##def asin2(a)
##    if (a-2*pi)

   
if __name__ == "__main__":
    
    r1 = Robot(URL,FILENAME)
    print("Robot made.")
    
    r1.setSpeed(0.1,0)                    #Set intial angular and linear velocity to zero
##    while(1):
##        r1.updateAttributes()
##        print r1.heading*180/pi
##    
##    print r1.heading*180/pi
    print('Robot intial coordinates and heading: X: ',r1.x,' Y: ',r1.y,' Heading: ',r1.heading)
    
    # Point to point navigation test
##    r1.goToPoint({'X':0.5,'Y':1})
##    print(r1.x,r1.y)
    
    # Follow the predefined path
    for i in range(0,len(px)):
        print('Next carrot point:',px[i],py[i])
        r1.goToPoint({'X':px[i],'Y':py[i]})
    print "Path completed successfully"
    
    # Take first n points from the path
    n=10
    px=list()
    py=list()
    for i in range(0,n):
        px[i]=r1.path['Pose']['Position']['X'][i]
        py[i]=r1.path['Pose']['Position']['Y'][i]
    print px,py
# Hey man, here's an example of how to go through the path data structure ;)
# print(r1.path[2]['Pose']['Position']['X'])

