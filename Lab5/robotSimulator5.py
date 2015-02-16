# robotSimulator5.py
# Uses different waypoints

# TODO: Better random spawning of start and end points

# Dimensions of robot
# 15.5cm wide, 12.5 from axle to back, 16.5 to wires, 3cm from axle to front
# 6.1" wide, 5" from axle to back
# 5.86" longest dimension from centre to edge

# Size of map: 96" x 48"
motorB = "motorB"
motorC = "motorC"


from math import *
import time, random
#random.seed(5)

from Tkinter import *
from eventBasedAnimationClass import EventBasedAnimationClass

mapWidth = 96
mapHeight = 48
gridWidth = 6

height = mapHeight * 10

outOfBoundsDistance = 6 # Minimum inches away from obstacle for start and end points
robotWidth = 6.1
#robotWidth = 5.86 * 2

legalStartingAndGoalPoints = [(1,1),(1,2),(2,1),(2,2),(3,1),(4,1),(5,1),
                              (5,2),(6,1),(7,1),(8,1),(8,2),(12,1),(12,2),
                              (12,3),(13,1),(13,2),(13,3),(14,1),(14,2),
                              (14,3),(15,1),(15,2),(15,3),(15,4),(15,5),
                              (10,4),(10,5),(11,4),(11,5),(1,6),(1,7),(2,6),
                              (2,7),(3,5),(3,6),(3,7),(4,7),(5,7),(6,7),
                              (7,6),(7,7),(8,7),(9,7)]



'''
class Waypoint(object):
    def __init__(self, index):
        self.index = index
        self.neighbours = []

    def addNeighbour(self, neighbourIndex):
        self.neighbours.append(neighbourIndex)

    def getNeighbours(self):
        return self.neighbours
'''


class Point(object):
    def __init__(self, x, y):
        # Takes in floats x and y, coordinates in inches
        self.x = x * 1.0
        self.y = y * 1.0

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __repr__(self):
        return "Point(%f, %f)" % (self.x, self.y)

    def __str__(self):
        return repr(self)

    def __abs__(self):
        return (self.x**2 + self.y**2)**0.5

    def __mul__(self, other):
        # Takes in float other, returns self divided by other
        return Point(self.x*1.0*other, self.y*1.0*other)

    def __rmul__(self, other):
        # Takes in float other, returns self divided by other
        return self * other

    def __div__(self, other):
        # Takes in float other, returns self divided by other
        return self * (1.0 / other)

    def unitVector(self):
        return self/abs(self)

    def getTheta(self):
        return atan2(self.y, self.x)

class RectangularObstacle(object):
    def __init__(self, point1, point2, point3, point4):
        # Takes in Points point1-4
        assert(type(point1) == Point)
        assert(type(point2) == Point)
        assert(type(point3) == Point)
        assert(type(point4) == Point)
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.point4 = point4

    def getDrawingCoordinates(self):
        # 10* because 1 inch = 10 pixels
        return [10 * self.point1.x, -10 * self.point1.y + height,
                10 * self.point2.x, -10 * self.point2.y + height,
                10 * self.point3.x, -10 * self.point3.y + height,
                10 * self.point4.x, -10 * self.point4.y + height]

    def getOutOfBoundsCoordinates(self):
        v1 = self.point2 - self.point1
        v2 = self.point3 - self.point2
        n1 = v1.unitVector()
        n2 = v2.unitVector()

        u1 = outOfBoundsDistance * n1
        u2 = outOfBoundsDistance * n2

        point1 = self.point1 - u1 - u2
        point2 = self.point2 + u1 - u2
        point3 = self.point3 + u1 + u2
        point4 = self.point4 - u1 + u2

        return [10 * point1.x, -10 * point1.y + height,
                10 * point2.x, -10 * point2.y + height,
                10 * point3.x, -10 * point3.y + height,
                10 * point4.x, -10 * point4.y + height]

    def getExpandedCoordinates(self):
        v1 = self.point2 - self.point1
        v2 = self.point3 - self.point2
        n1 = v1.unitVector()
        n2 = v2.unitVector()

        u1 = robotWidth/2 * n1
        u2 = robotWidth/2 * n2

        point1 = self.point1 - u1 - u2
        point2 = self.point2 + u1 - u2
        point3 = self.point3 + u1 + u2
        point4 = self.point4 - u1 + u2

        return [10 * point1.x, -10 * point1.y + height,
                10 * point2.x, -10 * point2.y + height,
                10 * point3.x, -10 * point3.y + height,
                10 * point4.x, -10 * point4.y + height]



class CircularObstacle(object):
    def __init__(self, centre, radius):
        # Takes in Point centre and float radius, in inches
        assert(type(centre) == Point)
        self.centre = centre
        self.r = radius

    def getDrawingCoordinates(self):
        # 10* because 1 inch = 10 pixels
        x0 = 10 * (self.centre.x - self.r)
        y0 =-10 * (self.centre.y - self.r) + height
        x1 = 10 * (self.centre.x + self.r)
        y1 =-10 * (self.centre.y + self.r) + height
        return [x0,y0,x1,y1]

    def getOutOfBoundsCoordinates(self):
        # 10* because 1 inch = 10 pixels
        r = self.r + outOfBoundsDistance
        x0 = 10 * (self.centre.x - r)
        y0 =-10 * (self.centre.y - r) + height
        x1 = 10 * (self.centre.x + r)
        y1 =-10 * (self.centre.y + r) + height
        return [x0,y0,x1,y1]

    def getExpandedCoordinates(self):
        # 10* because 1 inch = 10 pixels
        r = self.r + robotWidth/2
        x0 = 10 * (self.centre.x - r)
        y0 =-10 * (self.centre.y - r) + height
        x1 = 10 * (self.centre.x + r)
        y1 =-10 * (self.centre.y + r) + height
        return [x0,y0,x1,y1]













class EventBasedAnimationDemo(EventBasedAnimationClass):
    def __init__(self):

        self.width = mapWidth * 10 # 10 pixels is one inch
        self.height = height

        super(EventBasedAnimationDemo, self).__init__(self.width, self.height)

        self.timerDelay = 1 # Tweaking this value creates inaccuracies
        self.time = 0


        self.cx, self.cy = self.width/2, self.height/2  
        x,y = self.cx/10.0, self.cy/10.0 # In inches
        markerDistance = 1.18
        startX = x-markerDistance # In inches
        startY = y # In inches

        theta = 0
        L = 4.2 # inches
        r = 1.078 # inches

        # markerDistance = L/3 # What does marker distance mean now?
        robotWidth = 6.1 # inches
        leadingWidth = 1.18 # inches
        trailingWidth = 5 # inches

        startX, startY = random.choice(legalStartingAndGoalPoints)
        goalX, goalY = random.choice(legalStartingAndGoalPoints)
        startX, startY = 6/6, 6/6
        goalX, goalY = 18/6, 30/6

        # 0-1-2-3-4-7-5 3-6
        self.waypoints = [(18.2,39.2),(40.2,39.8),(52.4,41.1),(64.6,25.7),(40.9,7.2),(14.8,7.7),(80.0,10.6),(28.7,7.1)]
        self.neighbourList = [(1,),   (0,2),      (1,3),      (2,4,6),    (3,7),     (7,),      (3,),		(5,4)]
        self.robot = Robot(Point(startX*6, startY*6), Point(goalX*6, goalY*6), theta, L, r, markerDistance, robotWidth, leadingWidth, trailingWidth, self.waypoints, self.neighbourList)
        self.markerDistance = markerDistance


        r1 = RectangularObstacle(Point( 0,24), Point( 2.32, 29.69), Point(24.25,20.20), Point(21.85, 14.685))
        r2 = RectangularObstacle(Point(32.756,18), Point(51.69, 32.25), Point(55.45,27.53), Point(36.26, 13.19))
        r3 = RectangularObstacle(Point(78,24), Point(72.77, 27), Point(84,48), Point(89.5, 45.09))
        c1 = CircularObstacle(Point(30,30), 3)
        c2 = CircularObstacle(Point(60,12), 3)
        c3 = CircularObstacle(Point(66,39), 3)

        self.obstacles = [r1,r2,r3,c1,c2,c3]

        # Display settings
        self.showDangerZones = False

        

    
    def onMousePressed(self, event):
        #self.waypoints.append((event.x, event.y))
        print event.x*1.0/10, (height-event.y)*1.0/10

    def onKeyPressed(self, event):
        '''
        if event.keysym == "space":
            self.robot.x += 10
            self.robot.y += 10
        '''
        if event.char == "n":
            self.startTime = time.time() * 1000

            robot = self.robot
            goalX, goalY = random.choice(legalStartingAndGoalPoints)
            robot.start = Point(robot.x, robot.y)
            robot.goal = Point(goalX*6, goalY*6)
            # Possibly illegal positions:
            '''
            startX, startY = random.randint(0,96), random.randint(0,48)
            goalX, goalY = random.randint(0,96), random.randint(0,48)
            robot.start = Point(startX, startY)
            robot.goal = Point(goalX, goalY)
            robot.x = startX
            robot.y = startY
            '''
            robot.path = robot.findPath()
            robot.state = -1

            self.canvas.delete("endpoints")
            self.drawStartAndEndPoints()
        elif event.char == "s":
            self.showDangerZones = not self.showDangerZones
            self.canvas.delete("dangerZone")
            self.drawObstacles()
            self.drawWaypoints()
            self.drawOutOfBounds()
            self.drawGrid()
            self.drawStartAndEndPoints()


    def onTimerFired(self):
        currTime = time.time() * 1000
        self.time = currTime - self.startTime

        '''
        if self.time >= 9000:
            self.robot.powerLeft = 0
            self.robot.powerRight = 0
        elif self.time >= 6000:
            self.robot.powerLeft = -200
            self.robot.powerRight = -175
        elif self.time >= 3000:
            self.robot.powerLeft = 100
            self.robot.powerRight = 100
        else:
            self.robot.powerLeft = 200
            self.robot.powerRight = -200
            '''


        self.robot.makeDecision()
        self.robot.move((currTime - self.prevTime))

        #self.graphPoints.append(self.getIdealPosition(self.time))

        self.prevTime = currTime

    def setRobotPower(self):
        # Calculate where we need to be
        # Calculate power needed for powerLeft and powerRight of robot
        robot = self.robot
        L,r,theta = robot.L, robot.r, robot.theta
        xGoal, yGoal = self.getIdealPosition(self.time+17) # Look ahead in trajectory by 17ms
        xCurr, yCurr = robot.getMarkerPosition()

        xDiff = (xGoal - xCurr)
        yDiff = (- yGoal + yCurr) # Because of negative y in graphics

        kp = 20
        kt = 30
        vFinal = kp*(cos(theta) * xDiff + sin(theta) * yDiff)
        omega  = kt*((cos(theta) * yDiff - sin(theta) * xDiff)/L)

        robot.powerLeft = 9/pi * (2*vFinal - L*omega) / r
        robot.powerRight = 9/pi * (2*vFinal + L*omega) / r




    def getIdealPosition(self, t):
        t = t/1000.0
        if self.graphOption == 1:
            x = 500 * cos(t/10) * sin(t/10)
            y = 200 * sin(t/10) * sin(t/5)
        elif self.graphOption == 2:
            x = 200 * sin(3*t/5)
            y = 200 * cos(2*(t/5 + pi/4))
        elif self.graphOption == 3:
            x = 200 * cos(t/10) * cos(t/5)
            y = 200 * cos(3*t/10) * sin(t/10)
        elif self.graphOption == 4:
            x = 200 * (1.0/2*cos(3*t/10)-3.0/4*cos(t/5))
            y = 200 * (-3.0/4*sin(t/5)-1.0/2*sin(3*t/10))
        elif self.graphOption == 5:
            x = 100 * (-2 * cos(t/5)**2 - sin(t/10) + 1) * sin(t/5)
            y = 100 * cos(t/5) * (-2 * cos(t/5)**3 - sin(t/10) + 1)
        elif self.graphOption == 6:
            x = 100 * (2 * cos(t/12)**3 + 1)*sin(t/4)
            y = 100 * cos(t/4) * (1-2*sin(t/4)**4)
        elif self.graphOption == 7:
            x = 40*(5*cos(9*t/20)-4*cos(t/4))
            y = 40*(-4*sin(t/4)-5*sin(9*t/20))
        elif self.graphOption == 8: # Heart
            x = 160 * sin(t/5)**3
            y = 10*(13*cos(t/5) - 5 * cos(2*t/5) - 2*cos(3*t/5) - cos(4*t/5))
        elif self.graphOption == 9: # Batman curve?
            if t < 2:
                # At t = 0, x = -1, y = 1
                x = t/8-1
                y = t + 1
            elif 2 <= t < 3.5:
                # At t = 2, x = -0.75, y = 3
                x = t/6 - 13/12
                y = -t/2 + 4
            elif 3.5 <= t < 4.5:
                # At t = 3.5, x = -1/2, y = 2.25
                x = t - 4
                y = 2.25
            elif 4.5 <= t < 6:
                # At t = 4.5, x = 0.5, y = 2.25
                x = t/6 - 0.25
                y = t/2
            elif 6 <= t < 8:
                # At t = 6, 0.75, y = 3
                x = t/8
                y = -t + 9
            elif 8 <= t < 16:
                # At t = 8, x = 1, y = 1
                x = t/4 - 1
                y = (6*sqrt(10)/7 + 1.5 - 0.5*x) - (6*sqrt(10)/14)*sqrt(4-(x-1)**2)
            elif 16 <= t < 2.0/7*(56 + 6*sqrt(10) + 3*sqrt(33)):
                # At t=16, x=3, y=6*sqrt(10)/7
                y = -t/2 + 8 + 6*sqrt(10)/7
                x = 7 * sqrt(1-(y/3)**2)
            elif 2.0/7*(56 + 6*sqrt(10) + 3*sqrt(33)) <= t < 38.345:
                # At t = 2.0/7*(56 + 6*sqrt(10) + 3*sqrt(33)), x = 4, y = -3*sqrt(33)/7
                x = -2.0/3*t + 4.0/21*(77 + 6*sqrt(10) + 3*sqrt(33))
                #y = abs(x/2)-(3*sqrt(33)-7)/112*x**2 - 3 + sqrt((1-abs(abs(x)-2)-1)**2)
                y = abs(x/2) - (3*sqrt(33)-7)/112*x**2 - 3 + sqrt(1-(abs(abs(x)-2)-1)**2)
            elif 38.345 <= t < 48.69:
                # At t = 38.345, x = -4, y = -3*sqrt(33)/7
                y = t/2 - 38.345/2 - 3*sqrt(33)/7
                x = -7 * sqrt(1-(y/3)**2)
            elif 48.69 <= t < 56.69:
                # At t = 48.69, x = -3, y = 6*sqrt(10)/7
                x = t/4 - 48.69/4 - 3
                y = (6*sqrt(10)/7 + 1.5 - 0.5*abs(x) - (6*sqrt(10)/14)*sqrt(4-(abs(x)-1)**2))
            elif 56.69 <= t < 58.69:
                # x = -1, y = 1
                x = -1
                y = -t + 57.69
            elif 58.69 <= t < 60.19:
                # x = -1, y = -1
                x = t/2 -29.345 - 1
                y = t/2 -29.345 - 1
            elif 60.19 <= t < 61.69:
                # x = -0.75, y = -0.75
                x = -t/2 + 30.095 - 0.25
                y = t/2 -29.345 - 1
            else:
                x = 0
                y = 0

            x *= 30
            y *= 30
            '''
            x = 25 * sin(t/2) + 4
            y = 7 * t + 2
            '''


        return (self.cx + x, self.cy - y)



    def drawRobot(self):
        robot = self.robot
        x, y = robot.x, robot.y
        theta = robot.theta
        L = robot.L
        r = robot.r
        f,w,b = robot.front, robot.width, robot.back

        # Draw rectangle surrounding robot
        x0 = x - b * cos(theta) - w/2 * sin(theta)
        y0 = y - b * sin(theta) + w/2 * cos(theta)
        x1 = x + f * cos(theta) - w/2 * sin(theta)
        y1 = y + f * sin(theta) + w/2 * cos(theta)
        x2 = x + f * cos(theta) + w/2 * sin(theta)
        y2 = y + f * sin(theta) - w/2 * cos(theta)
        x3 = x - b * cos(theta) + w/2 * sin(theta)
        y3 = y - b * sin(theta) - w/2 * cos(theta)

        fill = "" if self.showDangerZones else "turquoise"
        self.canvas.create_polygon(x0*10, -y0*10+self.height, 
                                   x1*10, -y1*10+self.height, 
                                   x2*10, -y2*10+self.height, 
                                   x3*10, -y3*10+self.height, tags="redrawables",fill=fill,outline="black",width=2)

        # Draw axle
        x0 = x + L/2 * cos(theta + pi/2)
        y0 = y + L/2 * sin(theta + pi/2)
        x1 = x + L/2 * cos(theta - pi/2)
        y1 = y + L/2 * sin(theta - pi/2)
        self.canvas.create_line(x0*10,y0*-10+self.height,x1*10,y1*-10+self.height,tags="redrawables",width=5)

        # Draw robot wheels

        # Draw line from centre of axle to marker
        x2 = x + self.markerDistance * cos(theta)
        y2 = y + self.markerDistance * sin(theta)
        self.canvas.create_line(x*10,y*-10+self.height,x2*10,y2*-10+self.height,tags="redrawables",width=2)


    def drawRobotTrajectory(self):
        if len(self.robot.trajectoryPoints) > 1:
            x0,y0 = self.robot.trajectoryPoints[-2]
            x0 *= 10
            y0 *= -10
            y0 += self.height
            x1,y1 = self.robot.trajectoryPoints[-1]
            x1 *= 10
            y1 *= -10
            y1 += self.height
            self.canvas.create_line(x0,y0,x1,y1,fill="red",width=1)

    def drawGraph(self):
        if len(self.graphPoints) > 1:
            x0,y0 = self.graphPoints[-2]
            x0 *= 10
            y0 *= -10
            y0 += self.height
            x1,y1 = self.graphPoints[-1]
            x1 *= 10
            y1 *= -10
            y1 += self.height
            self.canvas.create_line(x0,y0,x1,y1,fill="blue",width=2)
    
    def drawTimer(self):
        self.canvas.create_text(10,10,text="%f" % self.time, anchor=NW,tags="redrawables")

    def drawActualTimer(self):
        self.canvas.create_text(self.width-10,10,text="%.3f" % (time.time()-self.startTime/1000), anchor=NE,tags="redrawables")

    def drawRobotPower(self):
        self.canvas.create_text(self.width-10,self.height-10,text="%d, %d" % (self.robot.motor[motorB], self.robot.motor[motorC]), anchor=SE,tags="redrawables")

    def drawRobotStats(self):
        self.canvas.create_text(10,self.height-10,text=self.robot.getStats(), anchor=SW, tags="redrawables")

    def drawGrid(self):
        for r in xrange(mapHeight/gridWidth-1):
            y = (r+1)*gridWidth*10
            self.canvas.create_line(0,y, self.width+20,y,tags="dangerZone")

        for c in xrange(mapWidth/gridWidth-1):
            x = (c+1)*gridWidth*10
            self.canvas.create_line(x,0, x,self.height+20,tags="dangerZone")

    def drawWaypoints(self):
        r = 4
        for x,y in self.robot.waypoints:
            x *= 10
            y *= -10
            y += height
            self.canvas.create_oval(x-r,y-r,x+r,y+r,fill="blue",outline="black",tags="dangerZone")

    def drawLinesConnectingWaypoints(self):
        if len(self.robot.waypoints) > 1:
            for i in xrange(len(self.robot.waypoints)-1):
                x0,y0 = self.robot.waypoints[i]
                x0 *= 10
                y0 *= -10
                y0 += height
                x1,y1 = self.robot.waypoints[i+1]
                x1 *= 10
                y1 *= -10
                y1 += height
                self.canvas.create_line(x0,y0,x1,y1)

    def drawStartAndEndPoints(self):
        robot = self.robot
        r = 10
        startX, startY = robot.start.x, robot.start.y
        self.canvas.create_oval(10*startX-r, -10*startY+height-r, 10*startX+r, -10*startY+height+r, fill="green",width=1,tags=("endpoints","dangerZone"))
        endX, endY = robot.goal.x, robot.goal.y
        self.canvas.create_oval(10*endX-r, -10*endY+height-r, 10*endX+r, -10*endY+height+r, fill="yellow",width=1,tags=("endpoints","dangerZone"))

    def redrawAll(self):
        self.canvas.delete("redrawables")
        
        self.drawRobot()

        #self.drawRobotTrajectory()

        self.drawRobotPower()
        self.drawRobotStats()

        #self.drawTimer()
        self.drawActualTimer()

        #self.drawLinesConnectingWaypoints()


    def drawObstacles(self):
        for obstacle in self.obstacles:
            if type(obstacle) == CircularObstacle:
                # Draw boundary
                if self.showDangerZones: self.canvas.create_oval(obstacle.getOutOfBoundsCoordinates(),fill="dark grey",dash=(5,5),tags="dangerZone")
            elif type(obstacle) == RectangularObstacle:
                # Draw boundary
                if self.showDangerZones: self.canvas.create_polygon(obstacle.getOutOfBoundsCoordinates(),fill="dark grey",dash=(5,5),outline="black",tags="dangerZone")

        for obstacle in self.obstacles:
            if type(obstacle) == CircularObstacle:
                # Draw boundary
                if self.showDangerZones: self.canvas.create_oval(obstacle.getExpandedCoordinates(),fill="pink",outline="pink",tags="dangerZone")
                self.canvas.create_oval(obstacle.getDrawingCoordinates(),fill="red",width=2,tags="dangerZone")
            elif type(obstacle) == RectangularObstacle:
                # Draw boundary
                if self.showDangerZones: self.canvas.create_polygon(obstacle.getExpandedCoordinates(),fill="pink",width=0,tags="dangerZone")
                self.canvas.create_polygon(obstacle.getDrawingCoordinates(),fill="red",tags="dangerZone",width=2,outline="black")

    def drawOutOfBounds(self):
        margin = 60
        self.canvas.create_rectangle(margin, margin, self.width-margin, self.height-margin, dash=(5,5),width=2,tags="dangerZone")

        if self.showDangerZones:
            margin = robotWidth/2*10
            self.canvas.create_rectangle(0,0,self.width+20,margin,fill="pink",width=0,tags="dangerZone")
            self.canvas.create_rectangle(0,0,margin,height+20,fill="pink",width=0,tags="dangerZone")
            self.canvas.create_rectangle(self.width-margin,0,self.width+20,height+20,fill="pink",width=0,tags="dangerZone")
            self.canvas.create_rectangle(0,height-margin,self.width+20,height+20,fill="pink",width=0,tags="dangerZone")

    def drawVoronoiDiagram(self):
        colours = ("red", "orange", "yellow", "green", "blue", "purple", "violet","black")
        cellSize = 5
        for r in xrange(0,self.height,cellSize):
            for c in xrange(0,self.width,cellSize):
                # Find nearest
                distances = []
                for x,y in self.waypoints:
                    distances.append(((x*10-c)**2+(y*-10+self.height-r)**2)**0.5)
                i = distances.index(min(distances))
                self.canvas.create_rectangle(c,r,c+cellSize,r+cellSize,fill=colours[i],width=0)

    def initAnimation(self):
        self.startTime = self.prevTime = time.time() * 1000

        #self.canvas.create_rectangle(0,0,self.width+20,self.height+20,fill="grey")
        #self.drawVoronoiDiagram()

        self.drawObstacles()
        self.drawWaypoints()
        self.drawGrid()
        self.drawStartAndEndPoints()
        self.drawOutOfBounds()
        self.redrawAll()

    




class Robot(object):
    def __init__(self, initialPosition, goal, theta, wheelbase, wheelRadius, markerDistance, width, leading, trailing, waypoints, neighbourList):
        assert(type(initialPosition) == Point)
        assert(type(goal) == Point)
        self.start = initialPosition
        self.goal = goal

        self.x = initialPosition.x
        self.y = initialPosition.y
        self.theta = theta

        self.L = wheelbase
        self.r = wheelRadius

        self.motor = {motorB: 0, motorC: 0}

        markerX = self.x + self.L/3 * cos(self.theta)
        markerY = self.y - self.L/3 * sin(self.theta)
        self.trajectoryPoints = [(markerX, markerY)]

        self.markerDistance = markerDistance

        self.width = width
        self.front = leading
        self.back = trailing

        self.waypoints = waypoints
        self.neighbourList = neighbourList
        #self.findNearestWaypoint()

        self.path = self.findPath()
        self.state = -1 # -1 = moving to first waypoint, len(n) = moving to goal, everything else is moving to ith waypoint

    def findPath(self):
        self.firstWaypoint = self.findNearestWaypoint()
        self.lastWaypoint = self.findNearestWaypointToGoal()
        if self.firstWaypoint == self.lastWaypoint: return [self.firstWaypoint]

        visited, queue = set(), [[self.firstWaypoint]]
        while queue:
            path = queue.pop(0)
            vertex = path[-1]
            if vertex not in visited:
                visited.add(vertex)
                for neighbour in self.neighbourList[vertex]:
                    if neighbour == self.lastWaypoint: return path + [neighbour]
                    if neighbour not in visited:
                        queue.append(path + [neighbour])

    def findNearestWaypointToGoal(self):
        distances = []
        for x,y in self.waypoints:
            distances.append(((self.goal.x-x)**2 + (self.goal.y-y)**2)**0.5)
        return distances.index(min(distances))

    def findNearestWaypoint(self):
        distances = []
        for x,y in self.waypoints:
            distances.append(((self.x-x)**2 + (self.y-y)**2)**0.5)
        return distances.index(min(distances))

    def findNearestWaypointsToStartAndEnd(self):
        distancesFromStart = []
        distancesFromGoal = []
        for x,y in self.waypoints:
            distancesFromStart.append(((self.x-x)**2 + (self.y-y)**2)**0.5)
            distancesFromGoal.append(abs(self.goal-Point(x,y)))

        self.nearestWaypointIndex = distancesFromStart.index(min(distancesFromStart))
        self.finalWaypointIndex = distancesFromGoal.index(min(distancesFromGoal))

    def move(self, timerDelay):
        # At 100 power, motors move 2.105 revolutions per second
        # Or 13.23 radians/s
        # v = r * omega

        vLeft = self.motor[motorB]/100.0 * 13.23 * self.r * 1.27
        vRight = self.motor[motorC]/100.0 * 13.23 * self.r * 1.27
        self.vFinal = (vLeft + vRight)/2
        self.omega = (vRight - vLeft)/self.L

        timerDelayInSeconds = timerDelay/1000.0;
        self.x += (self.vFinal * cos(self.theta)) * timerDelayInSeconds
        self.y += (self.vFinal * sin(self.theta)) * timerDelayInSeconds
        self.theta += self.omega * timerDelayInSeconds
        self.theta %= 2*pi
        if self.theta > pi: self.theta -= 2*pi

        self.trajectoryPoints.append(self.getMarkerPosition())

    def getMarkerPosition(self):
        x = self.x + self.markerDistance * cos(self.theta)
        y = self.y + self.markerDistance * sin(self.theta)
        return (x,y)

    def getStats(self):
        return "%.1f, %.1f, %.1f" % (self.x, self.y, self.theta/pi*180)

    def distanceToGoal(self):
        return abs(self.goal - Point(self.x, self.y))

    def makeDecision(self):
        if self.state == -1: # Move to first waypoint
            goal = self.waypoints[self.path[0]]
        elif self.state == len(self.path): # Move to last waypoint # Should turn off at here
            goal = (self.goal.x, self.goal.y)
        else: goal = self.waypoints[self.path[self.state]]


        #print "Moving to", goal
        v = Point(*goal) - Point(self.x, self.y)
        distanceThreshold = 0.5 # in inches

        if abs(v) < distanceThreshold:
            if self.state == len(self.path): self.motor[motorB], self.motor[motorC] = 0,0
            else: self.state += 1

        else:
            bearing = v.getTheta();
            difference = (bearing-self.theta)
            if difference > pi: difference -= 2*pi
            elif difference < -pi: difference += 2*pi
            
            # These angles can be tweaked
            hardTurnThreshold = 0.1
            slightTurnThreshold = 0.035

            maxPower = 30
            if difference > hardTurnThreshold: # Correct bearing is to the left, turn left
                self.motor[motorB] = -maxPower/2
                self.motor[motorC] = maxPower/2
            elif difference < -hardTurnThreshold: # Correct bearing is to the right, turn right
                self.motor[motorB] = maxPower/2
                self.motor[motorC] = -maxPower/2
            elif difference > slightTurnThreshold:
                self.motor[motorB] = maxPower-5
                self.motor[motorC] = maxPower
            elif difference < -slightTurnThreshold:
                self.motor[motorB] = maxPower
                self.motor[motorC] = maxPower-5
            else:
                self.motor[motorB] = maxPower
                self.motor[motorC] = maxPower





EventBasedAnimationDemo().run()

