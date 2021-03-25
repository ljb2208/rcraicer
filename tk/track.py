import pathlib
import pickle

class KeyFrame():
    def __init__(self, id, rotation, translation):
        self.id = id
        self.rotation = rotation
        self.translation = translation

class TrackPoint():
    def __init__(self, point3d, isBegin=False, isEnd=False):
        self.point3d = point3d
        self.isBegin = isBegin
        self.isEnd = isEnd

class TrackSegment():
    def __init__(self):
        self.points = []

    def addPoint(self, point3d, isBegin=False, isEnd=False):
        self.points.append(TrackPoint(point3d, isBegin, isEnd))
        return len(self.points)
    
    def getPointsCount(self):
        return len(self.points)

class TrackSegments():
    def __init__(self):
        self.segments = [TrackSegment()]
        self.currentSegmentIndex = 0
        self.originPoint = None

    def addToSegment(self, point3d, isBegin=False, isEnd=False):
        if (self.currentSegmentIndex == 0 and self.segments[self.currentSegmentIndex].getPointsCount() == 0):
            self.originPoint = point3d

        return self.segments[self.currentSegmentIndex].addPoint(point3d, isBegin, isEnd)

    def getCurrentSegmentIndex(self):
        return self.currentSegmentIndex

    def getCurrentSegmentPointsCount(self):
        return self.segments[self.currentSegmentIndex].getPointsCount()

    def getSegmentCount(self):
        return len(self.segments)

    def newSegment(self):
        self.segments.append(TrackSegment())
        self.currentSegmentIndex += 1

    def getCurrentSegment(self):
        return self.segments[self.currentSegmentIndex]
    
    def getSegmentAt(self, index):
        return self.segments[index]

    def convertPointRelativeToOrigin(self, point3d):
        return point3d
        # relPoint = [point3d[0] - self.originPoint[0],  point3d[1] - self.originPoint[1] , point3d[2] - self.originPoint[2]]
        # return relPoint
        


class Track():
    def __init__(self):
        self.segments = TrackSegments()            
        self.keyFrames = []
        self.testLoad()

    def testLoad(self):
        p1 = [1.5, 0., -50.]
        p2 = [-1.5, 0., -6.]
        p3 = [-3., 0., -4.]
        self.segments.addToSegment(p1, isBegin=True)
        self.segments.addToSegment(p2)
        self.segments.addToSegment(p3, isEnd=True)

    def addKeyFrame(self, rotation, translation):
        self.keyFrames.append(KeyFrame(len(self.keyFrames), rotation, translation))


    def loadTrack(self):
        filename = self.getTrackFilePath() + "track.pkl"
        trackFile = open(filename, 'rb')
        self.segments = pickle.load(trackFile)

        for segment in self.segments.segments:
            for pt in segment.points:
                print("Point: " + str(pt))

    def saveTrack(self):
        filename = self.getTrackFilePath() + "track.pkl"
        trackFile = open(filename, 'wb')
        pickle.dump(self.segments, trackFile)        

    def getTrackFilePath(self):            
        filePath = str(pathlib.Path(__file__).parent.absolute())
        filePath += "/tracks/"

        return filePath
    

