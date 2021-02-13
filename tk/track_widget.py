import sys, math, time
if sys.version_info[0] < 3:
    from Tkinter import Tk, YES, BOTH
else:
    from tkinter import Tk, YES, BOTH
from OpenGL import GL, GLU
from pyopengltk import OpenGLFrame
from track import Track

class TrackWidget(OpenGLFrame):

    def __init__(self, track, master, height, width):
        super().__init__(master, height=height, width=width)        
        self.start = 0
        self.nframes = 0
        self.master = master        
        self.track = track
        

    def initgl(self):                
        # GL.glViewport(0, 0, self.master.winfo_width(), self.master.winfo_height())
        print("Width: " + str(self.master.winfo_width()))
        print("Height: " + str(self.master.winfo_height()))
        GL.glClearColor(1.0, 1.0, 1.0, 0.0)
        # GL.glEnable(GL.GL_DEPTH_TEST)
        # GL.glEnable(GL.GL_POINT_SMOOTH)
        # GL.glColor3f(0.0, 0.0, 0.0)
        GL.glColor3f(255.0, 0.0, 0.0)
        # GL.glPointSize(4.0)
        # GL.glMatrixMode(GL.GL_PROJECTION)        
        # GLU.gluOrtho3D(-5, 5, -5, 5, -5, 5)        
        # GLU.gluPerspective(45., 1.0, 0.0, 100.)
        # GL.glMatrixMode(GL.GL_MODELVIEW)
        # GLU.gluLookAt(-1., -1. -1, 10., 10., 10.)
        # GL.glLoadIdentity()
 

    def redraw(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)                
        GL.glBegin(GL.GL_POINTS)           

        segment = self.track.segments.getCurrentSegment()

        for pt in segment.points:                                 
            GL.glVertex3f(pt[0], pt[1], pt[2])
            
        
        GL.glEnd()
        GL.glFlush()                

    def resize(self, width, height):
        # self.winfo_width = self.master.winfo_width()        
        self.configure(width=width, height=height)
        # self.tkResize(event)
    
        