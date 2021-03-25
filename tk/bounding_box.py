import numpy as np

class BoundingBox():
    def __init__(self):
        self.points = np.array([], dtype='int32')

    def clear(self):
        self.points = np.array([], dtype='int32')

    def addPoint(self, point):
        if self.points.size > 8:
            self.clear()
        
        self.points = np.append(self.points, point)        

    def isComplete(self):
        if self.points.size == 8:
            return True
        
        return False
            
    