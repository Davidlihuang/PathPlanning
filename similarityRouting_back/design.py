import json as js

class Net:
    def __init__(self) -> None:
        self.__name:str       = ""
        self.__pins:list      = [] 
        self.__template:list  = [] #center line set
        self.__width:list     = []
        self.__space:list     = []

    def getName(self)->str:
        return self.__name
    def getPins(self)->list:
        return self.__pins
    def getTemplate(self)->list:
        return self.__template
    def getWidth(self)->int:
        return self.__width
    def getSpace(self)->int:
        return self.__space
    
    def setName(self, nm:str):
        self.name = nm
    def addPin(self, pin:list):
        if len(pin) != 3:
            print("Error: pin must [x, y, z]!")
            return
        self.pins.append(pin)
    def setTemplate(self, tp:list):
        if len(tp) == 0:
            print("Warning: temple expect not empty!")
            return
        self.template = tp
    def setWidth(self, w:int):
        if w< 0 :
            print("Waring: width expect positive number")
        self.__width = w
    def setSpace(self, s:int):
        if s< 0 :
            print("Waring: space expect positive number")
        self.__space = s


class Polygon:
    def __init__(self) -> None:
        self.__points:list = []
class Design:
    def __init__(self) -> None:
        self.__name:str           = ""
        self.__position:list      = [0, 0]
        self.__width: int         = 0
        self.__height: int        = 0
        self.__nets:list[Net]     = []
        self.__obs: list[Polygon] = []
    def readDesign(self, file:str):  

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