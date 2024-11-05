from __future__ import absolute_import
import json as js

class Net(object):
    def __init__(self):
        self.__name:unicode       = u""
        self.__pins:list      = [] 
        self.__template:list  = [] #center line set
        self.__width:list     = []
        self.__space:list     = []

    def getName(self):
        return self.__name
    def getPins(self):
        return self.__pins
    def getTemplate(self):
        return self.__template
    def getWidth(self):
        return self.__width
    def getSpace(self):
        return self.__space
    
    def setName(self, nm):
        self.name = nm
    def addPin(self, pin):
        if len(pin) != 3:
            print u"Error: pin must [x, y, z]!"
            return
        self.pins.append(pin)
    def setTemplate(self, tp):
        if len(tp) == 0:
            print u"Warning: temple expect not empty!"
            return
        self.template = tp
    def setWidth(self, w):
        if w< 0 :
            print u"Waring: width expect positive number"
        self.__width = w
    def setSpace(self, s):
        if s< 0 :
            print u"Waring: space expect positive number"
        self.__space = s


class Polygon(object):
    def __init__(self):
        self.__points:list = []
class Design(object):
    def __init__(self):
        self.__name:unicode           = u""
        self.__position:list      = [0, 0]
        self.__width: int         = 0
        self.__height: int        = 0
        self.__nets:list[Net]     = []
        self.__obs: list[Polygon] = []
    def readDesign(self, file):  

        pass
    def getNets(self):
        pass
    def getObs(self):
        pass
    def getPosition(self):
        pass
    def getName(self):
        pass
    def showDesign(self):
        pass