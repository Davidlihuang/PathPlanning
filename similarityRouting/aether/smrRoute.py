'''
Author       : huangli
Date         : 2024-11-07 14:46:49
LastEditors  : huangli
LastEditTime : 2024-11-08 10:30:01
FilePath     : smrRoute.py
E-mail       : li.huang@istar-tech.com
Copyright    : Copyright (c) 2024 by iStar, All Rights Reserved.
Description  : 
'''
from __future__ import absolute_import
# import pyAether as PA
# from   LAD.Rule.TechRule.ViaDef.CustomViaInfo import CustomViaInfo
# from   LAD.Rule.TechRule.ViaDef.StdViaInfo import StdViaInfo
# from   LAD.Template.Via.ViaUtilTool import ViaUtilTool
# import LAD.AutoRoute.tmprouter.bin.libarary.dynamic.ISTARRouterWrapper as ISTARRouterWrapper
import os
import json
import time
import sys
import threading
import matplotlib.pyplot as plt
import numpy as np
# from   LAD.Common.Logger import Logger

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
currPath = os.path.dirname(os.path.abspath(__file__))
import smrRouteGraph as routegraph
import smrMikami as mikami
import smrAstar  

class Net:
    def __init__(self, name, w=0, s=0):
        self.name     = name
        self.width    = w
        self.space    = s
        self.pins     = []
        self.template = []
    def getName(self):
        return self.name
    def getWidth(self):
        return self.width
    def getSpace(self):
        return self.space
    def getPinNum(self):
        return len(self.pins)
    def getPins(self):
        return self.pins
    def getTemplate(self):
        return self.template
    def addPin(self, pin):
        self.pins.append(pin)

    def setTemplate(self, tmp):
        self.template = tmp

    def printNet(self):
        print("name:{0}".format(self.name))
        print("width:{0}".format(self.width))
        print("space:{0}".format(self.space))
        print("pins:")
        for pin in self.pins:
            print("pin:", pin)
            print("\t({0},{1},{2}), ".format(pin[0],pin[1],pin[4]))
        print("template:")
        for tmp in self.template:
            print("\t({0},{1},{2}), ".format(tmp[0],tmp[1],tmp[2]))

        

class Layout:
    def __init__(self, data, template, isDebug=False):
        self.name     = ""
        self.pos      = []
        self.width    = 0
        self.height   = 0
        self.obstacle = []
        self.layers   = []
        self.nets     = dict()
        self.nmToLyrDict = dict()
        self.isDebug  = isDebug
        self.parseData(data, template)
    def setDebug(self, isDebug = False):
        self.isDebug = isDebug
    def getWidth(self):
        return self.width
    def getHeight(self):
        return self.height
    def getObstacle(self):
        return self.obstacle
    def getLayerNum(self):
        return len(self.layers)
    def getNetNum(self):
        return len(self.nets)
    def getObstacleNum(self):
        return len(self.obstacle)
    def getPosition(self):
        return self.pos
    def getName(self):
        return self.name
    def getNetByName(self, nm):
        if nm in self.nets:
            return self.nets[nm]
        else:
            return None
    def getLayerIdByName(self, nm):
        if nm in self.nmToLyrDict:
            return self.nmToLyrDict[nm]
        else:
            return None
    def getNets(self):
        nets = []
        for _,net in self.nets.items():
            nets.append(net)
        return nets

    def parseData(self, data, template):
        self.pos        = data["location"]
        self.width      = data["width"]
        self.height     = data["height"]
        self.designName = str(data["designName"])
        self.__getLayers(data["interposer"])
        self.__getObstacle(data["Obstacles"])
        self.__parseNets(data["netlist"], template)

    def printLayout(self):
        print("name:{0}".format(self.designName))
        print("pos:({0}, {1})".format(self.pos[0], self.pos[1]))
        print("width:{0}".format(self.width))
        print("height:{0}".format(self.height))
        print("obsNum:{0}".format(len(self.obstacle)))
        print("netsNum:{0}".format(len(self.nets)))
        print("netLayers:{0}".format(len(self.layers)))

    def __getObstacle(self, obsObj):
        print("------------")
        # print("-->Obstacle:", obsObj
        for lyr, item in obsObj.items():            
            for _, poly in item.items():
                newPolys = []
                for p in poly:
                    pt = (int(p[0]),int(p[1]),int(lyr))
                    newPolys.append(pt)
                self.obstacle.append(newPolys)
    def __getLayers(self, lyrOBj):
        for lyr, value in lyrOBj.items(): 
            nm = str(value["name"])
            self.layers.append(int(lyr))
            self.nmToLyrDict[nm] = int(lyr)
        

    def __parseNets(self, netObj, template):
        print("Parser Net: ----------")
        for name, value in netObj.items():
            # name ="net333" #toDelete
            net = Net(name, value["width"], value["space"])
            for pin in value["pins"]:
                net.addPin(pin[:5])
            self.nets[name] = net
        
        templateDict = {}
        for item in template:
            tmpDict   = {}
            netName   = "TmpNetName__1"#str(item["net"])
            width     = int(item["width"]*1000)
            lyrName   = str(item["layer"])
            purposeNm = str(item["purpose"])
            points    = item["points"]
            print("points:", points)
            
            toJsonList=[]
            
            tmp = []   
            if self.isDebug == False:
                for pt in points:
                    x   = int(pt.x()*1000)
                    y   = int(pt.y()*1000)
                    lyr = int(self.nmToLyrDict[lyrName])
                    tmp.append([x,y, lyr])
                for pt in points:
                    lyr = int(self.nmToLyrDict[lyrName])
                    toJsonList.append([pt.x(),pt.y(), lyr])
            else:
                for pt in points:
                    x = int(pt[0]*1000)
                    y = int(pt[1]*1000)
                    lyr = self.nmToLyrDict[lyrName]   
                    tmp.append([x,y, lyr])
                for pt in points:
                    lyr = int(self.nmToLyrDict[lyrName])
                    toJsonList.append([pt[0],pt[1], lyr])

            print("--netname:", netName)
            print("--nets:", self.nets)
            print("--template:", tmp)
            if netName in self.nets:
                curNet = self.nets[netName]
                curNet.setTemplate(tmp)
                print("setTmp")
            else:
                print("Unfound nets")
            
            tmpDict["width"]   = item["width"]
            tmpDict["layer"]   = str(lyrName)
            tmpDict["purpose"] = str(purposeNm)
            tmpDict["points"]  = toJsonList#tmp
 
            templateDict[str(netName)]  = tmpDict
            
        print("templateDict:", templateDict)
        #dumpTemplate
        tmpFile = currPath + "/" +"template.json"
        with open(tmpFile, 'w') as inFile:
            inFile.write(json.dumps(templateDict, indent=4))
        
        for _,net in self.nets.items():
            net.printNet()    

class SmrRoute:
    def __init__(self, layoutData, templateObj, isDebug = False):
        print("I'm similarity router!>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print("data:",layoutData)
        print("template:",templateObj)
        self.__layout   = Layout(layoutData, templateObj, isDebug)
        self.__path      = {}
        self.__rtGraph   = None
        self.__layout.printLayout()
        self.__isMultiLayer = True

    def enMultiLyr(self, ml):
        self.__isMultiLayer = ml

    def routing(self,  isAstar=True): 
        #construct map
        w = self.__layout.getWidth()
        h = self.__layout.getHeight()
        l = self.__layout.getLayerNum()
        offsetPos  = self.__layout.getPosition()
        region     = [offsetPos[0],offsetPos[1], w, h]
        print("floorplane:", region)


        print(self.__layout.getNets())
        resPaths = []
        for net in self.__layout.getNets():
            print("route net: ", net.getName())
            resolution   = int(net.getWidth()/10)
            print("w:{0}, h:{1}, r:{2}".format(w, h, resolution))
            print("offset:{0}".format(offsetPos))
            self.rtGraph = routegraph.Grid3D(w,h, l, resolution, offsetPos)
            
            #set obstacles
            polygons     = self.__layout.getObstacle()
            
            start_time = time.time()
            print("obstacles nums: ", len(polygons))
            print("obstacles", polygons)
            e = int(net.getWidth()/2 + net.getSpace())
            print("net width:{0}, netSpace:{1}".format(net.getWidth(), net.getSpace()))
            print("enlarge: {0}".format(e))
            self.rtGraph.set_obstacle(polygons, e)
            end_time = time.time()
            print("set obstacle finish: {0}s".format((end_time - start_time)*1000) )
            
            template = net.getTemplate()

            #show design
            templateShow   = self.rtGraph.setNetTemplate(template, l-1, 2)  #tempalte   Z
            fig, ax = plt.subplots() 
            # ax.set_xlim(0, w + 0.1)
            # ax.set_ylim(0, h + 0.1)
            self.rtGraph.drawShapelyPolygon(fig, ax, polygons,  u"black", 1)
            self.rtGraph.drawShapelyPolygon(fig, ax, templateShow,  u"yellow", 0.4)
            
            
            if net.getPinNum() >2:
                continue

            #run
            pin1 = net.getPins()[0]
            pin2 = net.getPins()[1]
            start = (pin1[0], pin1[1], pin1[4])
            goal  = (pin2[0], pin2[1], pin2[4])
            
            path = None
            if isAstar == True:
                self.rtGraph.calSmrWithBFS(template, True)
                self.rtGraph.drawSmrMap()
                plt.show()
                path = self.__runAstar(start, goal, template)
                print("Use astar algorithm")
            else:
                path = self.__runMikami(start, goal, template)
                # path.draw()
                plt.show()
                print("Use line search algorithm")
            
            if path == None:
                continue

            path.setName(net.getName())
            path.setWidth(net.getWidth())
            path.setSpace(net.getSpace())

            resPaths.append(path)


        #result process
        return self.__path  
    
    def __runAstar(self, start, goal, template):
        print("start: {0}, goal:{1}".format(start, goal))
        start =  tuple(self.rtGraph.point2Gpoint(start))
        goal  =  tuple(self.rtGraph.point2Gpoint(goal))
        print("start: {0}, goal:{1}".format(start, goal))
        print("pStart:{0}, pGoal:{1}".format(self.rtGraph.gPoint2point(start), self.rtGraph.gPoint2point(goal)))
        astar    = smrAstar.AStar(self.rtGraph, start, goal, u"manhattan")
        path, _  = astar.searchWithTp(template)
        return path

    def __runMikami(self, start, goal, template):
        start =  tuple(self.rtGraph.point2Gpoint(start))
        goal  =  tuple(self.rtGraph.point2Gpoint(goal))
        temp = []
        for t in template:
            pt = tuple(self.rtGraph.point2Gpoint(t))
            temp.append(pt)
        mkt = mikami.MikamiTauchi(self.rtGraph, self.__isMultiLayer)
        path = mkt.findPathWithTemplate(start, goal, temp)
        return path

    def __dumpToResult(self, pathList):
        self.__path["designName"] = self.__layout.getName()
        self.__path["result"] = None
        contDict = {}
        for path in pathList:
            if path == None:
                continue
            name = path.getName()
            contDict[str(name)] = path.formatToDict()
        self.__path["result"] = contDict
        return self.__path
        
        

    
        

