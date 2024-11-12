'''
Author       : huangli
Date         : 2024-07-05 18:09:38
LastEditors  : huangli
LastEditTime : 2024-11-07 15:08:10
FilePath     : show.py
E-mail       : li.huang@istar-tech.com
Copyright    : Copyright (c) 2024 by iStar, All Rights Reserved.
Description  : 
'''
# coding=utf-8
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.patches import Rectangle
import matplotlib.patches as patches
import numpy as np
import sys
import json



def drawFloorPlaneByLayer(layout, resultlayout, fig, ax):
    cr = ["red", "blue", "green", "magenta", "deepskyblue","gold", "lime"]
    DBU = 1000
    # read chip size
    designName = layout["designName"]
    width = layout["width"]
    height = layout["height"]
    location = [layout["location"][0], layout["location"][1]]
    boundary = [
        layout["boundary"][0],
        layout["boundary"][1],
        layout["boundary"][2],
        layout["boundary"][3],
    ]
    chips = layout["chips"]

    #drawObstcle
    if type(floorplane["Obstacles"]) == type(None):
        return
    allObs = floorplane["Obstacles"]
    for key, value in allObs.items():
        layer = int(key)
        for idx, pointArr in value.items():
            poly = []
            for point in pointArr:
                x = point[0]
                y = point[1]
                poly.append([x, y])
            polygon = Polygon(
                xy=poly,
                facecolor=cr[layer - 1],
                edgecolor= "black", #cr[layer - 1],
                linewidth=0.05,
            )
            ax.add_patch(polygon)


    # draw floorplane
    print("draw top floorplane: ", width, height)
    fpRect = Rectangle(
        (location[0], location[1]),
        width,
        height,
        edgecolor="black",
        linewidth=0.5,
        fill=False,
    )
    ax.add_patch(fpRect)
    for key, value in chips.items():
        chipName = key

        chipPosition = [value["position"][0], value["position"][1]]
        chipWidth = value["width"]
        chipHeight = value["height"]
        chipboundary = [value["boundary"][0], value["boundary"][1]]
        if "pins" not in value:
            continue
        chipPins = value["pins"]
        # draw chip
        print("draw top chip: ", chipName)
        # chipRect = plt.Rectangle(
        #     (chipPosition[0], chipPosition[1]),
        #     chipWidth,
        #     chipHeight,
        #     edgecolor="yellow",
        #     linewidth=0.1,
        #     fill=False,
        # )
        # if(chipName!="chip_c4"):
        #     plt.text(x=chipPosition[0], y=chipPosition[1], s = chipName, fontsize = 1, fontstyle = 'oblique')
        # if chipName != "chip_c4":
        #     ax.add_patch(chipRect)

        for idx, content in chipPins.items():
            pinIdx = int(idx)
            pinType = content["type"]
            pintBumpSize = content["bumpDiameter"]
            pintBumpW = content["boundary_w"]
            pintBumpH = content["boundary_h"]
            pinPostion = [content["position"][0], content["position"][1]]
            pinNetname = content["netName"]
            pinLayer = content["layer"]

            centerX = pinPostion[0]
            centerY = pinPostion[1]
            radius = pintBumpSize / 2
           
            r = 90
            if width < height:
                r = 0
            ax.text(x=centerX, y=centerY, rotation=r, s=pinNetname, fontsize=0.02, fontstyle='oblique') 
            # if pinNetname!="VDD" or pinNetname != "VSS":
            #     ax.text(centerX, centerY, pinNetname, rotation=r, color='green', fontsize= 0.0001, ha='left', va='center')
            pinCr = cr[pinLayer - 1]
            if(chipName == "DIE4"):
                pinCr = "black"

            lx = centerX- pintBumpW/2
            ly =  centerY - pintBumpH/2
            pinRect = plt.Rectangle( (lx, ly),
                    pintBumpW,
                    pintBumpH,
                    facecolor = pinCr,
                    edgecolor=pinCr,
                    linewidth=0.1,
                    fill=True,
                 )
            ax.add_patch(pinRect)
        
    # getNetlist
    allnets = []
    print("Total valid net size: ", len(floorplane["netlist"]))
    for net, value in floorplane["netlist"].items():
        localNetname =  net
        pinlist = value["pins"]
        if len(pinlist) < 2:
            continue
        allnets.append(localNetname)
       
    # drawResult
    successNets=[]
    name      = resultlayout["designName"]
    # wireWidth = resultlayout["wireWidth"]
    # wirespace = resultlayout["wireSpace"]
    # viawidth  = resultlayout["viaWidth"]
    # viaspace  = resultlayout["viaSpace"]

    hatch_par = ["/", "+", "x", "-", "|", "o", "O", ".", "*", "x"]

    if type(resultlayout["result"]) == type(None):
        return

    result = resultlayout["result"]

    print("All net nums: ", len(result))
    for net, value in result.items():
        netName = net
        print("    ->net: ", netName)
        successNets.append(netName)
        if type(value["path"]) == type(None):
            continue
        path = value["path"]
        idx = 1
        for shapes in path:
            polylist = []
            layer = shapes["layer"]
            poly = shapes["poly"]
            lineWidth = int(shapes["width"])
            if len(poly) == 2:  # draw line
                p1 = [poly[0][0], poly[0][1]]
                p2 = [poly[1][0], poly[1][1]]
                
                pathRect = [0, 0, 0, 0]
                if (p1[0] == p2[0]) and (p1[1] != p2[1]):  # vertical
                    if p1[1] < p2[1]:
                        pathRect[0] = p1[0] - lineWidth / 2
                        pathRect[1] = p1[1] - lineWidth / 2
                    else:
                        pathRect[0] = p2[0] - lineWidth / 2
                        pathRect[1] = p2[1] - lineWidth / 2

                    pathRect[2] = lineWidth
                    pathRect[3] = abs(p2[1] - p1[1]) + lineWidth
                    
                elif (p1[1] == p2[1]) and (p1[0] != p2[0]):  # hori
                    if p1[0] < p2[0]:
                        pathRect[0] = p1[0] - lineWidth / 2
                        pathRect[1] = p1[1] - lineWidth / 2
                    else:
                        pathRect[0] = p2[0] - lineWidth / 2
                        pathRect[1] = p2[1] - lineWidth / 2
                    pathRect[2] = abs(p2[0] - p1[0]) + lineWidth
                    pathRect[3] = lineWidth
                  
                pRect = plt.Rectangle( (pathRect[0], pathRect[1]),
                pathRect[2],
                pathRect[3],
                facecolor = cr[layer - 1],
                edgecolor= cr[layer - 1],
                linewidth=0.1,
                fill=True,
                )
                ax.add_patch(pRect)
                  
            else:
                for points in poly:
                    x = points[0]
                    y = points[1]
                    polylist.append([x, y])
                polygon = Polygon(
                        xy=polylist,
                        hatch=hatch_par[layer - 1],
                        facecolor="none",
                        edgecolor=cr[layer - 1],
                        linewidth=0.1,
                    )
                # ax.add_patch(polygon)
        
        viaObj = value["via"]
        if type(viaObj) != type(None):
            for via in viaObj:
                pos  = via["pos"]
                lyrs = via["layers"]
                size = via["size"]
                viaRect = plt.Rectangle( (pos[0]-(size/2.0), pos[1]-(size/2.0)),
                    size,
                    size,
                    facecolor = "green",
                    edgecolor = "green",
                    linewidth=0.1,
                    fill=True,
                 )
                ax.add_patch(viaRect)

           
    
    #reflact  faile net to origin map
    setALL = set(allnets)
    setSuc = set(successNets)
    setfail = setALL.symmetric_difference(setSuc)
    failist = list(setfail)
    for net, value in floorplane["netlist"].items():
        localNetname =  net
        pinlist = value["pins"]
        for pin in pinlist:
            centerX = pin[0]
            centerY = pin[1]
            pw    = pin[2]
            ph     = pin[3]
            pinLayer    = pin[4]
            lx = centerX - pw/2
            ly =  centerY - ph/2

            pinCr = cr[pinLayer - 1]
            if localNetname in failist:
                pinCr = "black"
                print("failedNet:", localNetname)
            pinRect = plt.Rectangle( (lx, ly),
                pw,
                ph,
                facecolor = pinCr,
                edgecolor= cr[pinLayer - 1],
                linewidth=0.1,
                fill=True,
                )
            ax.add_patch(pinRect)
            r = 90
            if width < height:
                r = 0
            ax.text(x=centerX, y=centerY, rotation=r, s=localNetname, fontsize=0.02, fontstyle='oblique') 
    

    #net name:
    ax.set(xlabel="width (um)", ylabel="height (um)", title="Floorplan")
    # plt.yticks(np.arange(0, width + 100, 1000))
    # plt.xticks(np.arange(0, height + 100, 1000))
   

def onlyDrawResult(resultlayout, fig, ax):
    # drawResult
    cr = ["red", "blue", "green", "magenta", "gold", "deepskyblue", "lime"]
    name = resultlayout["designName"]
    wireWidth = resultlayout["wireWidth"]
    wirespace = resultlayout["wireSpace"]
    viawidth = resultlayout["viaWidth"]
    viaspace = resultlayout["viaSpace"]

    hatch_par = ["/", "+", "x", "-", "|", "o", "O", ".", "*", "x"]

    if type(resultlayout["result"]) == type(None):
        return

    result = resultlayout["result"]

    print("All net nums: ", len(result))
    for net, value in result.items():
        netName = net
        print("    ->net: ", netName)
        if type(value["path"]) == type(None):
            continue
        path = value["path"]
        # print(netName)
        idx = 1
        for shapes in path:
            polylist = []
            layer = shapes["layer"]
            poly = shapes["poly"]

            if len(poly) == 2:  # draw line
                p1 = [poly[0][0], poly[0][1]]
                p2 = [poly[1][0], poly[1][1]]

                ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    linewidth=0.1,
                    color=cr[layer - 1],
                    alpha=0.8,
                )

                pathRect = [0, 0, 0, 0]
                if p1[0] == p2[0]:  # vertical
                    if p1[1] < p2[1]:
                        pathRect[0] = p1[0] - wireWidth / 2
                        pathRect[1] = p1[1]
                    else:
                        pathRect[0] = p2[0] - wireWidth / 2
                        pathRect[1] = p2[1]

                    pathRect[2] = wireWidth
                    pathRect[3] = abs(p2[1] - p1[1])
                elif p1[1] == p2[1]:  # horiton
                    if p1[0] < p2[0]:
                        pathRect[0] = p1[0]
                        pathRect[1] = p1[1] - wireWidth / 2
                    else:
                        pathRect[0] = p2[0]
                        pathRect[1] = p2[1] - wireWidth / 2

                    pathRect[2] = abs(p2[0] - p1[0])
                    pathRect[3] = wireWidth

                    polylist.append([pathRect[0], pathRect[1]])
                    polylist.append([pathRect[0], pathRect[1] + pathRect[3]])
                    polylist.append(
                        [pathRect[0] + pathRect[2], pathRect[1] + pathRect[3]]
                    )
                    polylist.append([pathRect[0] + pathRect[2], pathRect[1]])

                # polygon = Polygon(xy = polylist, facecolor='none', edgecolor=cr[layer-1], linewidth = 0.1)
                # ax.add_patch(polygon)
            else:
                for points in poly:
                    x = points[0]
                    y = points[1]
                    polylist.append([x, y])
                    polygon = Polygon(
                        xy=polylist,
                        hatch=hatch_par[layer - 1],
                        facecolor="none",
                        edgecolor=cr[layer - 1],
                        linewidth=0.1,
                    )
                    ax.add_patch(polygon)

    ax.set(xlabel="width (um)", ylabel="height (um)", title="Floorplan")
    # plt.yticks(np.arange(0, width + 100, 1000))
    # plt.xticks(np.arange(0, height + 100, 1000))


def save_subfig(fig, ax, save_path, fig_name):
    bbox = ax.get_tightbbox(fig.canvas.get_renderer()).expanded(1.02, 1.02)
    extent = bbox.transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(save_path + fig_name, dpi=2000, bbox_inches=extent)
    print("save files success!")



if __name__ == "__main__":
    floorplanefiles = "./dumpInput.json"
    resuleFile = "./result.json"
    # floorplanefiles = "./dumpInput_1.json"
    # resuleFile = "./result_1.json"

    with open(floorplanefiles, "r") as f:
        floorplane = json.load(f)

    with open(resuleFile, "r") as f:
        resultLay = json.load(f)

    # draw floorplane
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set(xlabel="width (um)", ylabel="height (um)", title="Floorplan")
    drawFloorPlaneByLayer(floorplane, resultLay, fig, ax)
    plt.plot()
    plt.show()
    save_subfig(fig, ax, "./", f"{resuleFile}.png")
