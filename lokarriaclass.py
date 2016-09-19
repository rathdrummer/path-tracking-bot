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
FILENAME = 'Path-around-table-and-back.json'


BEARING_THRESHOLD = 0.1
DISTANCE_THRESHOLD = 0.7
GOTOPOINT_ANG_SPEED = 1
GOTOPOINT_LIN_SPEED = 0.7
DISTANCE_BETWEEN_CARROT_POINTS=0.3
TICK=0.05

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
        linearSpeed: self-explanatory
        angularSpeed: idem
        heading: the heading of the robot (angle)
    """

    def __init__(self, server, pathFilename):
        """Return a Robot after reading and storing both server URL and JSON path"""
        self.server = server
        self.path = openJsonTrajectory(pathFilename)
        self.linearSpeed = 0.0
        self.angularSpeed = 0.0
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
            self.angularSpeed = speed
            return response
        else:
            return UnexpectedResponse(response)

    def setLinearSpeed(self, speed):
        """Sets linear speed of the bot by sending a command to the server"""
        param = json.dumps({'TargetLinearSpeed':speed})
        response = self.serverJsonPost(param)
        status = response.status
        if status == 204:
            self.linearSpeed = speed
            return response
        else:
            return UnexpectedResponse(response)

    def setSpeed(self,angularSpeed,linearSpeed):
        """Sets speed of the bot by sending a command to the server"""
        param = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
        response = self.serverJsonPost(param)
        status = response.status
        if status == 204:
            self.linearSpeed = linearSpeed
            self.angularSpeed = angularSpeed
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
        """Updates the robot pose attributes, heading and coordinates, but not speeds."""
        try:
            pose = self.getPose()
            self.heading = toHeading(pose['Pose']['Orientation'])
            self.x = pose['Pose']['Position']['X']
            self.y = pose['Pose']['Position']['Y']
        except:
            print("updateAttributes Error: Connection to server refused. Are you sure you have the right address?")

    def getBearing(self, coordinates):
        dX = coordinates['X']-self.x
        dY = coordinates['Y']-self.y
        angle = atan2(dY,dX)
##        if angle < 0:
##                angle=2*pi+angle
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
        yp = self.y - distance * cos(self.heading)
        xp = self.x - distance * sin(self.heading)
        point={'X':xp,'Y':yp}
        return point
        
    def goToPoint(self, coordinates):
        """Adjusts the robot's trajectory to head towards a point; a continuous adaptation of the previous 2-step version goToPoint
        Made to be looped in rapid succession (every 100ms).
        If point reached (as per DISTANCE_THRESHOLD standard) then returns 1, else returns 0
        :param p1: The point, dictionary containing at least the x and y coordinates under indices 'X' and 'Y' respectively.
        """
        result=True
        while (result):
            self.updateAttributes()
            time.sleep(TICK)
            
            bearing = self.getBearing(coordinates)
            distance = self.distanceTo(coordinates)

            if (bearing>BEARING_THRESHOLD):
                angularSpeed = GOTOPOINT_ANG_SPEED
                
            elif (bearing<-BEARING_THRESHOLD):
                angularSpeed = -GOTOPOINT_ANG_SPEED
                
            else:
                angularSpeed = 0
           
            if (distance>DISTANCE_THRESHOLD):
                result=True
            else:
                result=False

            self.setSpeed(angularSpeed,GOTOPOINT_LIN_SPEED)





    def chooseCarrotPoints(self,distance):
        px=[]
        py=[]
        px.append(self.path[0]['Pose']['Position']['X'])
        py.append(self.path[0]['Pose']['Position']['Y'])
        for i in range(0,len(self.path)):
            x=self.path[i]['Pose']['Position']['X']
            y=self.path[i]['Pose']['Position']['Y']
            if (sqrt((px[len(px)-1]-x)**2+(py[len(py)-1]-y)**2)>distance):
                px.append(x)
                py.append(y)
        return [px,py]
            
            

        


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
    print('Robot intial coordinates and heading: X: ',r1.x,' Y: ',r1.y,' Heading: ',r1.heading)

    coordinates = {'X':0.5,'Y':0.5}

##    while (True):
##        r1.updateAttributes()
##        time.sleep(0.1)
##            
##        bearing = r1.getBearing(coordinates)
##        if (bearing>BEARING_THRESHOLD or bearing<-BEARING_THRESHOLD):
##            print "not facing coords"
##        else:
##            print "Facing coords!"
    
    # Choose carrot points at a particular distance 'DISTANCE_BETWEEN_CARROT_POINTS' from each point
    [Cx,Cy]=r1.chooseCarrotPoints(DISTANCE_BETWEEN_CARROT_POINTS)
    print Cx
    print Cy


    # Follow the predefined path
    print "Timer started"
    time_start=time.time()
    print "Path following started"
    for i in range(0,len(Cx)):
        print('Next carrot point:',Cx[i],Cy[i])
        r1.goToPoint({'X':Cx[i],'Y':Cy[i]})
    print "Path following completed successfully"
    r1.setSpeed(0,0)
    time_traversed = time.time()- time_start
    print "Time taken for following the path = ",time_traversed

