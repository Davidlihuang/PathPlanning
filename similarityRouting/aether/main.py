'''
Author       : huangli
Date         : 2024-11-08 09:53:08
LastEditors  : huangli
LastEditTime : 2024-11-08 10:33:35
FilePath     : main.py
E-mail       : li.huang@istar-tech.com
Copyright    : Copyright (c) 2024 by iStar, All Rights Reserved.
Description  : 
'''
import json
import smrRoute
import smrRouteGraph
#layout
with open('/home/huangli/Desktop/python/similarityRouter/PathPlanning/similarityRouting/aether/dumpInput.json', 'r') as file:
    data = file.read()
g_istarroutedata = json.loads(data)
print(g_istarroutedata)

#template
with open('/home/huangli/Desktop/python/similarityRouter/PathPlanning/similarityRouting/aether/template.json', 'r') as file1:
    data1 = file1.read()

tmp          = json.loads(data1)
testPathList = []
tempDict     = {}
for netNm, content in tmp.items():
    tempDict['net']     = str(netNm)
    tempDict['width']   = content['width']
    tempDict['layer']   = str(content['layer'])
    pLst                = []
    tempDict['points']  = content['points']
    tempDict['purpose'] = content['purpose']
testPathList.append(tempDict)


router = smrRoute.SmrRoute(g_istarroutedata, testPathList, True)
g_drawData =  router.routing(isAstar=False)

outputJson = './result.json'
try:
    with open(outputJson, 'w') as outFile:
        outFile.write(json.dumps(g_drawData, indent=4))
except Exception as e:
    print('error: fail to save ' + outputJson)
print(g_drawData)