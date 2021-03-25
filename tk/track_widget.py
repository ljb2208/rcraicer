import sys, math, time
if sys.version_info[0] < 3:
    from Tkinter import Tk, YES, BOTH
else:
    from tkinter import Tk, YES, BOTH
from OpenGL import GL, GLU, GLUT
from pyopengltk import OpenGLFrame
import glm
import math
from track import Track


class ModelView():
    def __init__(self):
        self.position = glm.vec3(0., 0., -7.)        
        self.rotation = glm.vec3(1., 0., 0.)
        self.rotationAngles = glm.vec3(0., 0., 0.)
        self.mouseSensitivity = 0.1
        self.mouseMoving = False
        self.mouseX = 0
        self.mouseY = 0

    def processMouseScroll(self, offset):
        self.position[2] += offset

    def processMouseMove(self):
        i = 0

    def processLeftMouseClick(self, event):
        print("Left mouse down: " + str(event))
        self.mouseMoving = True
        self.mouseX = event.x
        self.mouseY = event.y

    
    def processLeftMouseRelease(self, event):
        print("Left mouse release: " + str(event))
        self.mouseMoving = False

    def processMouseMove(self, event):
        if self.mouseMoving == False:
            return
        
        xDiff = (event.x - self.mouseX) * self.mouseSensitivity
        yDiff = (event.y - self.mouseY) * self.mouseSensitivity        

        self.rotationAngles[1] += xDiff
        self.rotationAngles[0] += yDiff
        

        self.mouseX = event.x
        self.mouseY = event.y
        
        
        

class Camera():
    def __init__(self, position=glm.vec3(0., 0., 3.)):
        self.position = position 
        self.worldUp=glm.vec3(0., 1., 0.)
        self.yaw=-90.
        self.pitch=0.
        self.speed=2.5
        self.mouseSensitivity = 0.1
        self.front = None
        self.right = None
        self.up = None
        self.zoom = 45.        

        self.updateCameraVectors()

    def updateCameraVectors(self):
        frontTemp = glm.vec3()
        frontTemp.x = math.cos(glm.radians(self.yaw)) * math.cos(glm.radians(self.pitch))
        frontTemp.y = math.sin(glm.radians(self.pitch))
        frontTemp.z = math.sin(glm.radians(self.yaw)) * math.cos(glm.radians(self.pitch))
        self.front = glm.normalize(frontTemp)
        self.right = glm.normalize(glm.cross(self.front, self.worldUp))
        self.up = glm.normalize(glm.cross(self.right, self.front))

    def getViewMatrix(self):
        return glm.lookAt(self.position, self.position + self.front, self.up)

    def processMouseScroll(self, yOffset):
        self.zoom -= float(yOffset)

        if (self.zoom < 1.):
            self.zoom = 1.

        if self.zoom > 45.0:
            self.zoom = 45.

        

    def processMouseMovement(self, xOffset, yOffset, constrainPitch=True):
        xOffset *= self.mouseSensitivity
        yOffset *= self.mouseSensitivity

        self.yaw += xOffset
        self.pitch += yOffset

        if constrainPitch == True:
            if self.pitch > 89.0:
                self.pitch = 89.0

            if self.pitch < -89.0:
                self.pitch = -89.0

        self.updateCameraVectors()



class TrackWidget(OpenGLFrame):

    def __init__(self, track, master, height, width):
        super().__init__(master, height=height, width=width)        
        self.start = 0
        self.nframes = 0
        self.master = master        
        self.track = track

        self.anglePyramid = 0.
        self.angleCube = 0.

        self.aspect = 1.0
        self.longZ = 100.0
        self.scaleFactor = 1.0        
        self.modelView = ModelView()

        self.bind('<MouseWheel>', self.onMouseWheel)
        self.bind('<Button-4>', self.onMouseWheel)
        self.bind('<Button-5>', self.onMouseWheel)

        self.bind('<Button-1>', self.modelView.processLeftMouseClick)
        self.bind('<B1-Motion>', self.modelView.processMouseMove)
        self.bind('<ButtonRelease-1>', self.modelView.processLeftMouseRelease)

        self.camera = Camera() #glm.vec3(0., 0., 3.)
        self.firstMouse = True

        
        

    def initgl(self):                
        # GL.glViewport(0, 0, self.master.winfo_width(), self.master.winfo_height())
        print("Width: " + str(self.master.winfo_width()))
        print("Height: " + str(self.master.winfo_height()))
        
        GL.glColor3f(255.0, 0.0, 0.0)
        GL.glPointSize(4.0)
        # GL.glMatrixMode(GL.GL_PROJECTION)        
        # GLU.gluOrtho3D(-5, 5, -5, 5, -5, 5)        
        # GLU.gluPerspective(45., 1.0, 0.0, 100.)
        # GL.glMatrixMode(GL.GL_MODELVIEW)
        # GLU.gluLookAt(-1., -1. -1, 10., 10., 10.)
        # GL.glLoadIdentity()        
        
        GL.glClearColor(0., 0., 0., 1.)
        GL.glClearDepth(1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LEQUAL)
        GL.glShadeModel(GL.GL_SMOOTH)
        GL.glHint(GL.GL_PERSPECTIVE_CORRECTION_HINT, GL.GL_NICEST)

        # GL.glMatrixMode(GL.GL_PROJECTION)      
        # GLU.gluPerspective(45., 1.0, 0.0, 100.)
        GL.glMatrixMode(GL.GL_MODELVIEW)        
        GL.glLoadIdentity()
        GL.glTranslatef(self.modelView.position[0], self.modelView.position[1], self.modelView.position[2])
        # GLU.gluLookAt(0., 0., 0., 0, 0., -10., 0, 0, 1)


    def redraw(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glMatrixMode(GL.GL_MODELVIEW)

        model = glm.mat4(1.0)
        view = self.camera.getViewMatrix()
        projection = glm.perspective(glm.radians(self.camera.zoom), float(self.width) / float(self.height), 0.1, 100.)


        if (self.scaleFactor == 1.):
            GL.glDisable(GL.GL_NORMALIZE)
        else:
            GL.glEnable(GL.GL_NORMALIZE)

        # GL.glPopMatrix()
        # GL.glPushMatrix()
        GL.glMatrixMode(GL.GL_MODELVIEW)        
        GL.glLoadIdentity()
        GL.glTranslatef(self.modelView.position[0], self.modelView.position[1], self.modelView.position[2])
        # GL.glLoadIdentity()
        # GL.glScalef(self.scaleFactor, self.scaleFactor, self.scaleFactor)
        
        GL.glRotatef(self.modelView.rotationAngles[0], 1., 0., 0.)
        GL.glRotatef(self.modelView.rotationAngles[1], 0., 1., 0.)        

        for segment in self.track.segments.segments:            
            # draw points
            GL.glBegin(GL.GL_POINTS)
            GL.glColor3f(0.0, 1.0, 0.0)     # Green

            pointCount = len(segment.points)

            for point in segment.points:
                GL.glVertex(point.point3d[0], point.point3d[1], point.point3d[2])

            GL.glEnd()

            # draw lines between points
            GL.glBegin(GL.GL_LINES)

            beginPoint = None
            endPoint = None

            for i in range(0, pointCount - 1):
                p1 = segment.points[i]
                p2 = segment.points[i+1]

                if p1.isBegin:
                    beginPoint = p1

                if p2.isEnd:
                    endPoint= p2
                
                GL.glVertex(p1.point3d[0], p1.point3d[1], p1.point3d[2])
                GL.glVertex(p2.point3d[0], p2.point3d[1], p2.point3d[2])

            if (beginPoint is not None and endPoint is not None):
                GL.glVertex(beginPoint.point3d[0], beginPoint.point3d[1], beginPoint.point3d[2])
                GL.glVertex(endPoint.point3d[0], endPoint.point3d[1], endPoint.point3d[2])

            GL.glEnd()
        
        GL.glFlush()        

    #     self.anglePyramid += 0.2
    #     self.angleCube -= 0.15
    

    # def redraw(self):
    #     GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)       
    #     GL.glBegin(GL.GL_POINTS)           

    #     segment = self.track.segments.getCurrentSegment()

    #     for pt in segment.points:                                 
    #         GL.glVertex3f(pt[0], pt[1], pt[2])
            
        
    #     GL.glEnd()
    #     GL.glFlush()               
    # 

    def drawCube(self):
        GL.glBegin(GL.GL_QUADS)
        

        GL.glColor3f(0.0, 1.0, 0.0)     # Green
        GL.glVertex3f( 1.0, 1.0, -1.0)
        GL.glVertex3f(-1.0, 1.0, -1.0)
        GL.glVertex3f(-1.0, 1.0,  1.0)
        GL.glVertex3f( 1.0, 1.0,  1.0)
 
        # Bottom face (y = -1.0f)
        GL.glColor3f(1.0, 0.5, 0.0)     # Orange
        GL.glVertex3f( 1.0, -1.0,  1.0)
        GL.glVertex3f(-1.0, -1.0,  1.0)
        GL.glVertex3f(-1.0, -1.0, -1.0)
        GL.glVertex3f( 1.0, -1.0, -1.0)
    
        # Front face  (z = 1.0f)
        GL.glColor3f(1.0, 0.0, 0.0)     # Red
        GL.glVertex3f( 1.0,  1.0, 1.0)
        GL.glVertex3f(-1.0,  1.0, 1.0)
        GL.glVertex3f(-1.0, -1.0, 1.0)
        GL.glVertex3f( 1.0, -1.0, 1.0)
    
        # Back face (z = -1.0f)
        GL.glColor3f(1.0, 1.0, 0.0)     # Yellow
        GL.glVertex3f( 1.0, -1.0, -1.0)
        GL.glVertex3f(-1.0, -1.0, -1.0)
        GL.glVertex3f(-1.0,  1.0, -1.0)
        GL.glVertex3f( 1.0,  1.0, -1.0)
    
        # Left face (x = -1.0f)
        GL.glColor3f(0.0, 0.0, 1.0)     # Blue
        GL.glVertex3f(-1.0,  1.0,  1.0)
        GL.glVertex3f(-1.0,  1.0, -1.0)
        GL.glVertex3f(-1.0, -1.0, -1.0)
        GL.glVertex3f(-1.0, -1.0,  1.0)
    
        # Right face (x = 1.0f)
        GL.glColor3f(1.0, 0.0, 1.0)     # Magenta
        GL.glVertex3f(1.0,  1.0, -1.0)
        GL.glVertex3f(1.0,  1.0,  1.0)
        GL.glVertex3f(1.0, -1.0,  1.0)
        GL.glVertex3f(1.0, -1.0, -1.0)

        GL.glEnd()
        GL.glFlush()        

    def resize(self, width, height):              
        self.configure(width=width, height=height)
    
        self.aspect = float(width) / float(height)
        GL.glViewport(0, 0, width, height)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(45., self.aspect, 0.01, 100.)

    def onMouseWheel(self, event):        
        zoom = 0
        if (event.num == 4 or event.delta == 120):
            zoom = 1        
        
        if (event.num == 5 or event.delta == -120):
            zoom = -1

        self.modelView.processMouseScroll(zoom)                    
        # self.camera.processMouseScroll(zoom)
    