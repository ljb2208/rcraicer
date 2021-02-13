import pathlib
import pickle


class TrackSegment():
    def __init__(self):
        self.points = []

    def addPoint(self, point3d):
        self.points.append(point3d)
        return len(self.points)
    
    def getPointsCount(self):
        return len(self.points)

class TrackSegments():
    def __init__(self):
        self.segments = [TrackSegment()]
        self.currentSegmentIndex = 0
        self.originPoint = None

    def addToSegment(self, point3d):
        if (self.currentSegmentIndex == 0 and self.segments[self.currentSegmentIndex].getPointsCount() == 0):
            self.originPoint = point3d

        return self.segments[self.currentSegmentIndex].addPoint(self.convertPointRelativeToOrigin(point3d))

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
        self.testLoad()

    def testLoad(self):
        p1 = [10., 0., 10.]
        p2 = [20., 0., 20.]
        p3 = [30., 0., 30.]
        self.segments.addToSegment(p1)
        self.segments.addToSegment(p2)
        self.segments.addToSegment(p3)


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
    

