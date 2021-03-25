from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk
import cv2
import threading
from queue import Queue
from cam_thread import CameraThread
from imagetools import ImageTools
from pose_estimator import PoseEstimator
from track_widget import TrackWidget
from track import Track
import pathlib
from os import path

from OpenGL import GL
from pyopengltk import OpenGLFrame

from bounding_box import BoundingBox


class ScrolledFrame(Frame):

    def __init__(self, parent, vertical=True, horizontal=False):
        super().__init__(parent)

        # canvas for inner frame
        self._canvas = Canvas(self)
        self._canvas.grid(row=0, column=0, sticky='news') # changed

         # create right scrollbar and connect to canvas Y
        self._vertical_bar = Scrollbar(self, orient='vertical', command=self._canvas.yview)
        if vertical:
            self._vertical_bar.grid(row=0, column=1, sticky='ns')
        self._canvas.configure(yscrollcommand=self._vertical_bar.set)

        # create bottom scrollbar and connect to canvas X
        self._horizontal_bar = Scrollbar(self, orient='horizontal', command=self._canvas.xview)
        if horizontal:
            self._horizontal_bar.grid(row=1, column=0, sticky='we')
        self._canvas.configure(xscrollcommand=self._horizontal_bar.set)

        # inner frame for widgets
        self.inner = Frame(self._canvas)        
        self._window = self._canvas.create_window((0, 0), window=self.inner, anchor='nw')
        self.inner.grid_columnconfigure(0, weight=1) # changed

        
        # autoresize inner frame
        self.columnconfigure(0, weight=1) # changed
        self.rowconfigure(0, weight=1) # changed

        # resize when configure changed
        self.inner.bind('<Configure>', self.resize)
        self._canvas.bind('<Configure>', self.frame_width)

    def frame_width(self, event):
        # resize inner frame to canvas size
        canvas_width = event.width
        self._canvas.itemconfig(self._window, width = canvas_width)

    def resize(self, event=None): 
        self._canvas.configure(scrollregion=self._canvas.bbox('all'))


class TrackBuilder():

    def __init__(self):
        self.main = Tk()
        self.mainFrame = None
        self.running = True

        self.mainFrame = None
        self.imageFrame = None
        self.ctrlFrame = None
        
        self.cameraThread = None        
        self.camera = None
        self.messageQueue = Queue()
        self.imageTools = ImageTools()

        self.leftImage = None
        self.rightImage = None

        self.leftFilterImage = None
        self.rightFilterImage = None

        self.leftImageCanvas = None
        self.rightImageCanvas = None
        self.filterImageCanvas = None

        self.leftBB = BoundingBox()
        self.rightBB = BoundingBox()

        self.saveImagesFlag = False
        self.saveImageIndex = 0

        self.xLeftSel = -1
        self.yLeftSel = -1

        self.xRightSel = -1
        self.yRightSel = -1

        self.image3d = None
        self.point3d = None
        self.eucDistance = None        

        self.track = Track()

        self.buildMainWindow()    
        self.buildTabs()



    def testFnc(self):
        # newImage = self.imageTools.detectOrb(self.leftFilterImage, 0, 0)
        # newImage = self.imageTools.detectEdges(self.leftFilterImage)
        # self.loadImage(self.leftFilterImageCanvas, newImage, False)
        #self.imageTools.keyPointTest(self.leftImagePrev, self.rightImagePrev, self.leftImage, self.rightImage)
        est = PoseEstimator()
        est.calculatePose(self.leftImagePrev, self.rightImagePrev, self.leftImage, self.rightImage)

    def getDepthToSelectedCone(self):

        if (self.xLeftSel < 0 or self.yLeftSel < 0):
            messagebox.showwarning("Warning", "No point selected in left image")
            return

        if (self.xRightSel < 0 or self.yRightSel < 0):
            messagebox.showwarning("Warning", "No point selected in right image")
            return
        
        newImageLeft, newImageRight = self.imageTools.isolateObject(self.leftImage, self.rightImage, self.xLeftSel, self.yLeftSel, self.xRightSel, self.yRightSel)
        self.loadImage(self.rightFilterImageCanvas, newImageRight, False)
        self.loadImage(self.leftFilterImageCanvas, newImageLeft, False)

        self.point3d, self.eucDistance = self.imageTools.getConeDepth(self.leftImage, self.rightImage, self.xLeftSel, self.yLeftSel, self.xRightSel, self.yRightSel)
        self.setStatusText()

    def addToSegment(self):
        if self.point3d is not None:
            self.track.segments.addToSegment(self.point3d)
            self.setStatusText()
            self.point3d = None
        else:
            messagebox.showwarning("Warning", "No 3D Point generated")


    def onImageMouseButton(self, event):
        self.xLeftSel = event.x * 2
        self.yLeftSel = event.y * 2 

        self.leftBB.addPoint([self.xLeftSel, self.yLeftSel])       

        if (self.leftBB.isComplete()):
            self.drawBoundingBox()

        self.point3d = None
        self.eucDistance = None

        if self.image3d is not None:
            self.point3d, self.eucDistance = self.imageTools.getDepthAtPoint(self.image3d, self.xLeftSel, self.yLeftSel)

        self.setStatusText()        

    def onRightImageMouseButton(self, event):
        self.xRightSel = event.x * 2
        self.yRightSel = event.y * 2        

        # self.point3d = None
        # self.eucDistance = None

        # if self.image3d is not None:
        #     self.point3d, self.eucDistance = self.imageTools.getDepthAtPoint(self.image3d, self.xSel, self.ySel)

        self.setStatusText()        

    def onImageMouseButtonHalf(self, event):
        self.xLeftSel = event.x * 4
        self.yLeftSel = event.y * 4        

        self.point3d = None
        self.eucDistance = None

        if self.image3d is not None:
            self.point3d, self.eucDistance = self.imageTools.getDepthAtPoint(self.image3d, self.xLeftSel, self.yLeftSel)

        self.setStatusText()

    def buildImageFrame(self):
        
        self.leftImageCanvas = Canvas(self.imageFrame, width=640, height=512)
        self.leftImageCanvas.grid(row=0, column=0, rowspan=2, sticky="N,S,W")        
        self.leftImageCanvas.bind("<Button-1>", self.onLeftImageMouseButton)

        self.rightImageCanvas = Canvas(self.imageFrame, width=320, height=256)
        self.rightImageCanvas.grid(row=0, column=1, sticky="N,S,W")

        self.filterImageCanvas = Canvas(self.imageFrame, width=320, height=256)
        self.filterImageCanvas.grid(row=1, column=1, sticky="N,S,W")

        self.ctrlFrame.grid_columnconfigure(0, weight=1)
        self.ctrlFrame.grid_rowconfigure(0, weight=1)       

    def buildTabs(self):
        self.tabs = ttk.Notebook(self.mainFrame)
        self.tabs.grid(column=0, row=0, sticky=NSEW)
        self.tabCams = Frame(self.mainFrame)
        self.tabDepth = Frame(self.mainFrame)
        self.tabTrack = Frame(self.mainFrame)

        self.tabs.add(self.tabCams, text="Cameras")
        self.tabs.add(self.tabDepth, text="Depth")
        self.tabs.add(self.tabTrack, text="Track")

        self.buildCamTab()
        self.buildDepthTab()
        self.buildTrackTab()

    def buildCamTab(self):
        camsFrame = Frame(self.tabCams)
        camsFrame.grid(row=0, column=0, sticky="N,S,W")
        camControlsFrame = Frame(self.tabCams)
        camControlsFrame.grid(row=0, column=1, sticky=NE)

        self.tabCams.grid_columnconfigure(0, weight=1)
        self.tabCams.grid_rowconfigure(0, weight=1)        

        # self.ctrlFrame.grid(row=0, column=1, sticky=NE)
        self.leftImageCanvas = Canvas(camsFrame, width=640, height=512)
        self.leftImageCanvas.grid(row=0, column=0, rowspan=2, sticky="N,S,W")        
        self.leftImageCanvas.bind("<Button-1>", self.onImageMouseButton)

        self.rightImageCanvas = Canvas(camsFrame, width=640, height=512)
        self.rightImageCanvas.grid(row=0, column=1, sticky="N,S,W")
        self.rightImageCanvas.bind("<Button-1>", self.onRightImageMouseButton)

        self.leftFilterImageCanvas = Canvas(camsFrame, width=320, height=256)
        self.leftFilterImageCanvas.grid(row=1, column=0, sticky="N,S,W")
        self.leftFilterImageCanvas.bind("<Button-1>", self.onImageMouseButtonHalf)

        self.rightFilterImageCanvas = Canvas(camsFrame, width=320, height=256)
        self.rightFilterImageCanvas.grid(row=1, column=1, sticky="N,S,W")

         #add connect button
        self.connectButton = Button(camControlsFrame, text="Connect", width=10, command=self.connect)
        self.connectButton.grid(row=0, column=0, sticky=NE)

        saveButton = Button(camControlsFrame, text="Save Images", width=10, command=self.saveImages)
        saveButton.grid(row=1, column=0, sticky=NE)

        loadButton = Button(camControlsFrame, text="Load Images", width=10, command=self.loadImages)
        loadButton.grid(row=2, column=0, sticky=NE)

        filterButton = Button(camControlsFrame, text="Filter Images", width=10, command=self.filterImages)
        filterButton.grid(row=3, column=0, sticky=NE)

        testButton = Button(camControlsFrame, text="Test", width=10, command=self.testFnc)
        testButton.grid(row=4, column=0, sticky=NE)

        getDepthButton = Button(camControlsFrame, text="Get Depth", width=10, command=self.getDepthToSelectedCone)
        getDepthButton.grid(row=5, column=0, sticky=SE)

        addToSegButton = Button(camControlsFrame, text="Add to Segment", width=10, command=self.addToSegment)
        addToSegButton.grid(row=6, column=0, sticky=SE)

    def buildDepthTab(self):
        depthFrame = Frame(self.tabDepth)
        depthFrame.grid(row=0, column=0, sticky="N,S,W")
        depthControlsFrame = Frame(self.tabDepth)
        depthControlsFrame.grid(row=0, column=1, sticky=NE)

        self.tabDepth.grid_columnconfigure(0, weight=1)
        self.tabDepth.grid_rowconfigure(0, weight=1)        

        # self.ctrlFrame.grid(row=0, column=1, sticky=NE)
        self.depthImageCanvas = Canvas(depthFrame, width=640, height=512)
        self.depthImageCanvas.grid(row=0, column=0, sticky="N,S,W")        
        self.depthImageCanvas.bind("<Button-1>", self.onImageMouseButton)

        self.depthImageCanvasFiltered = Canvas(depthFrame, width=640, height=512)
        self.depthImageCanvasFiltered.grid(row=0, column=1, sticky="N,S,W")        
        self.depthImageCanvasFiltered.bind("<Button-1>", self.onImageMouseButton)

        self.fixedLeftCanvas = Canvas(depthFrame, width=320, height=256)
        self.fixedLeftCanvas.grid(row=1, column=0, sticky="N,S,W")        
        self.fixedLeftCanvas.bind("<Button-1>", self.onImageMouseButtonHalf)
        
        depthButton = Button(depthControlsFrame, text="Depth", width=10, command=self.calculateDepth)
        depthButton.grid(row=0, column=0, sticky=NE)

    def buildTrackTab(self):
        self.trackFrame = Frame(self.tabTrack)
        self.trackFrame.grid(row=0, column=0, sticky="N,S,E,W")
        self.trackFrame.bind("<Configure>", self.onTrackFrameResize)

        self.trackCanvas = Canvas(self.trackFrame, height=600, width=600)
        self.trackCanvas.grid(row=0, column=0, sticky=NSEW)                

        self.trackControlsFrame = Canvas(self.trackFrame)
        self.trackControlsFrame.grid(row=0, column=1, sticky="N,S,E")

        loadTrackButton = Button(self.trackControlsFrame, text="Load Track", width=10, command=self.track.loadTrack)       
        loadTrackButton.grid(row=0, column=0, sticky=NE)

        saveTrackButton = Button(self.trackControlsFrame, text="Save Track", width=10, command=self.track.saveTrack)       
        saveTrackButton.grid(row=1, column=0, sticky=NE)


        self.trackWidget = TrackWidget(self.track, self.trackCanvas, 600, 600)
        self.trackWidget.grid(row=0, column=0, sticky=NSEW)                
        self.trackWidget.animate = 1

        

        self.tabTrack.grid_columnconfigure(0, weight=1)
        self.tabTrack.grid_rowconfigure(0, weight=1)  

        self.trackFrame.grid_columnconfigure(0, weight=1)
        self.trackFrame.grid_rowconfigure(0, weight=1)  

        self.trackWidget.grid_columnconfigure(0, weight=1)
        self.trackWidget.grid_rowconfigure(0, weight=1)  

    def onTrackFrameResize(self, event):
        print("resize: " + str(event.width) + " " + str(event.height))
        # self.trackCanvas.winfo_height = self.trackFrame.winfo_height
        # self.trackCanvas.winfo_width = self.trackFrame.winfo_width
        # self.trackWidget.winfo_height = self.trackCanvas.winfo_height
        # self.trackWidget.winfo_width = self.trackCanvas.winfo_width


        print("Canvas size: " + str(self.trackCanvas.winfo_width()) + " " + str(self.trackCanvas.winfo_height()))
        print("Widget size: " + str(self.trackWidget.winfo_width()) + " " + str(self.trackWidget.winfo_height()))

        width = self.trackFrame.winfo_width() - 5        
        height = self.trackFrame.winfo_height() - 5        
        self.trackCanvas.configure(width=width, height=height)        
        self.trackWidget.resize(width, height)

        print("Canvas size after: " + str(self.trackCanvas.winfo_width()) + " " + str(self.trackCanvas.winfo_height()))
        print("Widget size after: " + str(self.trackWidget.winfo_width()) + " " + str(self.trackWidget.winfo_height()))


    def buildMainWindow(self):
        self.main.title("Track Builder")
        self.mainWidth = self.main.winfo_width()
        self.mainHeight = self.main.winfo_height()
        self.main.bind("<Configure>", self.onResize)
        self.main.protocol("WM_DELETE_WINDOW", self.close)   

        self.mainFrame = Frame(self.main)
        self.mainFrame.grid(row=0, column=0, sticky=NSEW)        
        self.mainFrame.grid_columnconfigure(0, weight=1)
        self.mainFrame.grid_rowconfigure(0, weight=1)                        

        self.statusFrame = Frame(self.main, borderwidth=1)
        self.statusFrame.grid(row=1, column=0, sticky=S)
        self.statusText = Label(self.statusFrame)
        self.statusText.grid(row=0, column=0, sticky=NSEW)
        self.pointsText = Label(self.statusFrame)
        self.pointsText.grid(row=1, column=0, sticky=NSEW)

        self.main.grid_columnconfigure(0, weight=1)
        self.main.grid_rowconfigure(0, weight=1)        

    def setStatusText(self):
        text = ""

        if self.xLeftSel > -1 and self.yLeftSel > -1:            
            text = "LEFT X: " + str(self.xLeftSel) + " Y:" + str(self.yLeftSel)                

        if self.xRightSel > -1 and self.yRightSel > -1:            
            text += " RIGHT X: " + str(self.xRightSel) + " Y:" + str(self.yRightSel)                

        if self.point3d is not None:
            text += " 3D Point: {" + str(self.point3d) + "}"

        if self.eucDistance is not None:
            text += " Distance: " + str(self.eucDistance)        

        self.statusText['text'] = text

        pointsText = "Segments: " + str(self.track.segments.getSegmentCount()) + " Current: " + str(self.track.segments.currentSegmentIndex) + " Points: " + str(self.track.segments.getCurrentSegmentPointsCount())
        self.pointsText['text'] = pointsText
    

    def getImageFilePath(self):
        filePath = str(pathlib.Path(__file__).parent.absolute())
        filePath += "/images/"

        return filePath

    def saveImages(self):
        self.saveImagesFlag = True

    def loadImages(self):

        leftImageFile = self.getImageFilePath() + "left_" + str(self.saveImageIndex) + ".png"
        rightImageFile = self.getImageFilePath() + "right_" + str(self.saveImageIndex) + ".png"

        if not path.exists(leftImageFile):
            self.saveImageIndex = 0
            leftImageFile = self.getImageFilePath() + "left_" + str(self.saveImageIndex) + ".png"
            rightImageFile = self.getImageFilePath() + "right_" + str(self.saveImageIndex) + ".png"
        
        if not path.exists(leftImageFile):
            return

        li = cv2.imread(leftImageFile)
        ri = cv2.imread(rightImageFile)

        self.saveImageIndex += 1

        if (self.leftImage is not None and self.rightImage is not None):
            pose = self.imageTools.getNewCameraPose(self.leftImage, self.rightImage, li, ri)
            print("New camera pose: " + str(pose))
        

        self.leftImagePrev = self.leftImage
        self.rightImagePrev = self.rightImage
        self.leftImage = li.copy()
        self.rightImage = ri.copy()

        self.loadImage(self.leftImageCanvas, li)
        self.loadImage(self.rightImageCanvas, ri)

    def filterImages(self):
        self.leftFilterImage = self.imageTools.getFilterImage(self.leftImage, 2)
        self.rightFilterImage = self.imageTools.getFilterImage(self.rightImage, 2)

        self.loadImage(self.leftFilterImageCanvas, self.leftFilterImage, False)
        self.loadImage(self.rightFilterImageCanvas, self.rightFilterImage, False)

    def capture(self):
        if self.leftImage is not None:
            self.leftFilterImage = self.imageTools.getFilterImage(self.leftImage, 2)
            self.loadImage(self.filterImageCanvas, self.leftFilterImage, True)

        if self.rightImage is not None:
            self.rightFilterImage = self.imageTools.getFilterImage(self.rightImage, 2)            

    def connect(self):
        if self.cameraThread is None or not self.cameraThread.is_alive():
            self.cameras = CameraThread(self, self.messageQueue)
            self.cameraThread = threading.Thread(target=self.cameras.run)
            self.cameraThread.start()    
            self.connectButton['text'] = "Disconnect"
        else:
            if self.cameras is not None:
                self.cameras.terminate()
                self.cameras = None
            self.connectButton['text'] = "Connect"

    def showEdges(self):
        edges = self.imageTools.detectEdges(self.leftFilterImage)
        self.loadImage(self.filterImageCanvas, edges, True)

    def calculateDepth(self):
        raw, filtered, self.image3d = self.imageTools.getDepthImageMod(self.leftImage, self.rightImage)
        self.loadImage(self.depthImageCanvas, raw, True)
        self.loadImage(self.depthImageCanvasFiltered, filtered, True)        
        self.loadImage(self.fixedLeftCanvas, self.imageTools.fixedLeft, False)

        if self.xLeftSel > -1 and self.yLeftSel > -1:
            self.point3d, self.eucDistance = self.imageTools.getDepthAtPoint(self.image3d, self.xLeftSel, self.yLeftSel)
            self.setStatusText()

    def close(self):
        self.running = False

    def onResize(self, event):
        if event.widget == self.main:
            self.mainWidth = event.width
            self.mainHeight = event.height

    def onCameraImages(self, images):   
        self.loadImage(self.leftImageCanvas, images[0])   
        self.loadImage(self.rightImageCanvas, images[1])                   

        if self.saveImagesFlag:
            cv2.imwrite(self.getImageFilePath() + "left_" + str(self.saveImageIndex) + ".png", images[0])
            cv2.imwrite(self.getImageFilePath() + "right_" + str(self.saveImageIndex) + ".png", images[1])
            self.saveImageIndex += 1
            self.saveImagesFlag = False

        self.image3d = None
        self.leftImage = images[0]
        self.rightImage = images[1]

    
    def loadImage(self, imageCanvas, image, grayscale=False):
        

        self.main.update()
        

        if (grayscale == False):
            tkImage = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            tkImage = Image.fromarray(tkImage)
        else:
            tkImage = Image.fromarray(image)

        width, height = tkImage.size
        ctlWidth = imageCanvas.winfo_width()

        if width > ctlWidth:
            height = height * ctlWidth/width
            width = ctlWidth            
            tkImage.thumbnail((width, height))  

        tkImage = ImageTk.PhotoImage(image=tkImage)
        
        imageCanvas.create_image(0, 0, anchor=NW, image=tkImage)
        imageCanvas._image_cache = tkImage  # avoid garbage collection        

    def drawBoundingBox(self):
        self.imageTools.drawBoundingBox(self.leftImage, self.leftBB)
        self.loadImage(self.leftImageCanvas, self.leftImage)


    def runMainLoop(self):

        self.loadImages()
        while self.running:
            self.main.update_idletasks()
            self.main.update()

            while not self.messageQueue.empty():
                images = self.messageQueue.get_nowait()

                if images is not None:
                    self.onCameraImages(images)

        self.main.destroy()


def main():
    trackBuilder = TrackBuilder()
    trackBuilder.runMainLoop()

if __name__ == "__main__":
    main()
    pass
