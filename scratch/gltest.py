import sys, math, time
if sys.version_info[0] < 3:
    from Tkinter import Tk, YES, BOTH
else:
    from tkinter import Tk, YES, BOTH
import OpenGL.GL as gl
import OpenGL.GLU as glu
import OpenGL.GLUT as oglut
from pyopengltk import OpenGLFrame
import glm


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



class GlutWindow(object):

    def init_opengl(self):
        gl.glClearColor(0.0,0,0.4,0)
        gl.glDepthFunc(gl.GL_LESS)
        gl.glEnable(gl.GL_DEPTH_TEST)
        
    def ogl_draw(self):
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        oglut.GLUT_KEY_UP
        glu.gluLookAt(4.0,3.0,-3.0, 
                0.0,0.0,0.0,
                0.0,1.0,0.0)
        #built in model
        oglut.glutSolidTeapot(1)

        
    def display(self):    
        self.ogl_draw()
        oglut.glutSwapBuffers()
    def idle(self):
        pass
    def resize(self,Width,Height):
        print("please overrider resize")
        gl.glViewport(0, 0, Width, Height)
        glu.gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)        

    def on_keyboard(self,key,x,y):     
        if(self.controller!=None):
              self.controller.on_keyboard(key,x,y)
        else:
            print("please overrider on_keyboard")
    def on_special_key(self,key,x,y):     
        if(self.controller!=None):
              self.controller.on_special_key(key,x,y)
        else:
            print("please overrider on_keyboard")
        
    def on_mouse(self,*args,**kwargs):
        if(self.controller!=None):
              self.controller.on_mouse(*args,**kwargs)
        else:        
            print("please overrider on_mouse")

    def on_mousemove(self,*args,**kwargs):
        if(self.controller!=None):
              self.controller.on_mousemove(*args,**kwargs)
        else:                
            print("please overrider on_mousemove")
                
    def __init__(self,*args,**kwargs):

        oglut.glutInit(sys.argv)
        oglut.glutInitDisplayMode(oglut.GLUT_RGBA | oglut.GLUT_DOUBLE | oglut.GLUT_DEPTH)
        oglut.glutInitWindowSize(800, 480)
        self.window = oglut.glutCreateWindow(b"window")
        oglut.glutDisplayFunc(self.display)
        #oglut.glutIdleFunc(self.display) 
        oglut.glutReshapeFunc(self.resize)  
        oglut.glutKeyboardFunc(self.on_keyboard)   
        oglut.glutSpecialFunc(self.on_special_key)  
        oglut.glutMouseFunc(self.on_mouse)
        oglut.glutMotionFunc(self.on_mousemove)
        self.controller = None
        self.update_if = oglut.glutPostRedisplay

    def run(self):
        oglut.glutMainLoop()

class TestWindow(GlutWindow):    
    def GLContext(object):
        pass

    def init_opengl(self):
        gl.glClearColor(0.0,0,0.4,0)
        gl.glDepthFunc(gl.GL_LESS)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_CULL_FACE)

    def ogl_draw(self):
        print("draw++")

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        gl.glMatrixMode(gl.GL_MODELVIEW)        
        gl.glLoadIdentity()
        gl.glTranslatef(1.5, 0., -17)

        gl.glBegin(gl.GL_QUADS)        

        gl.glColor3f(0.0, 1.0, 0.0)     # Green
        gl.glVertex3f( 1.0, 1.0, -1.0)
        gl.glVertex3f(-1.0, 1.0, -1.0)
        gl.glVertex3f(-1.0, 1.0,  1.0)
        gl.glVertex3f( 1.0, 1.0,  1.0)
 
        # Bottom face (y = -1.0f)
        gl.glColor3f(1.0, 0.5, 0.0)     # Orange
        gl.glVertex3f( 1.0, -1.0,  1.0)
        gl.glVertex3f(-1.0, -1.0,  1.0)
        gl.glVertex3f(-1.0, -1.0, -1.0)
        gl.glVertex3f( 1.0, -1.0, -1.0)
    
        # Front face  (z = 1.0f)
        gl.glColor3f(1.0, 0.0, 0.0)     # Red
        gl.glVertex3f( 1.0,  1.0, 1.0)
        gl.glVertex3f(-1.0,  1.0, 1.0)
        gl.glVertex3f(-1.0, -1.0, 1.0)
        gl.glVertex3f( 1.0, -1.0, 1.0)
    
        # Back face (z = -1.0f)
        gl.glColor3f(1.0, 1.0, 0.0)     # Yellow
        gl.glVertex3f( 1.0, -1.0, -1.0)
        gl.glVertex3f(-1.0, -1.0, -1.0)
        gl.glVertex3f(-1.0,  1.0, -1.0)
        gl.glVertex3f( 1.0,  1.0, -1.0)
    
        # Left face (x = -1.0f)
        gl.glColor3f(0.0, 0.0, 1.0)     # Blue
        gl.glVertex3f(-1.0,  1.0,  1.0)
        gl.glVertex3f(-1.0,  1.0, -1.0)
        gl.glVertex3f(-1.0, -1.0, -1.0)
        gl.glVertex3f(-1.0, -1.0,  1.0)
    
        # Right face (x = 1.0f)
        gl.glColor3f(1.0, 0.0, 1.0)     # Magenta
        gl.glVertex3f(1.0,  1.0, -1.0)
        gl.glVertex3f(1.0,  1.0,  1.0)
        gl.glVertex3f(1.0, -1.0,  1.0)
        gl.glVertex3f(1.0, -1.0, -1.0)

        gl.glEnd()
        gl.glFlush()        

    def resize(self, Width, Height):
        gl.glViewport(0, 0, Width, Height)        
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        glu.gluPerspective(130., float(Width)/float(Height), 0.01, 100.)
        # self.calcMVP(Width,Height)

    def init_context(self):
        # self.context = gl.GLContext()
        self.context = self.GLContext()

    def calcMVP(self, width, height):
        self.context.Projection = glm.perspective(glm.radians(45.0),float(width)/float(height),0.1,1000.0)
        self.context.View =  glm.lookAt(glm.vec3(4,3,-3), # Camera is at (4,3,-3), in World Space
						        glm.vec3(0,0,0), #and looks at the (0.0.0))
						        glm.vec3(0,1,0) ) #Head is up (set to 0,-1,0 to look upside-down)

        self.context.Model =  glm.mat4(1.0)

        self.context.MVP =  self.context.Projection * self.context.View * self.context.Model	


if __name__ == "__main__":
    win = TestWindow()
    win.init_opengl()
    win.init_context()
    win.run()
    
