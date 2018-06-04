from parseElem import Rectangle, Point, Bus, Bit, Pin, Track, Layer, Obstacle
import parser
import sys
import svgwrite
import subprocess
import numpy as np
import math
from datetime import datetime

def scaling(target, factor=0.01):
    return np.multiply(target, factor)

benchmarks = ['example_1', 'example_2']

if len(sys.argv) > 1:
    bench = sys.argv[1]
else:
    print("Invalid argument")     

dirpos = "../bench/"
colors = ['red', 'orange', 'yellow', 'green', 'blue', 'black', 'purple']
curTime = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
inputFile = dirpos + benchmarks[(int)(bench)] + ".input"
outputFile = "./svg/" + benchmarks[(int)(bench)] + curTime + ".output.svg"

layerDic = parser.getLayerInfo(inputFile)
trackList = parser.getTrackInfo(inputFile)
busDic = parser.getBusInfo(inputFile)
obsList = parser.getObstacleInfo(inputFile)
designBoundary = parser.getDesignBoundary(inputFile)
designWidth = designBoundary.ur.x - designBoundary.ll.x
designHeight = designBoundary.ur.y - designBoundary.ll.y

#Size Param
emptySpace = 1000*(designWidth/150000)
strokeWidthLayer = 0.5*math.sqrt(designWidth/150000)
strokeWidthObject = 0.07*math.sqrt(designWidth/150000)
fontSize = 200*(designWidth/150000)
svgFileSize = designWidth * len(layerDic) + emptySpace* len(layerDic)


svgFile = svgwrite.Drawing(filename = outputFile, size=scaling((svgFileSize , designHeight)), profile='full')
pattern = svgFile.defs.add(svgFile.pattern(size = (20,20), patternUnits="userSpaceOnUse"))
pattern.add(svgFile.rect((5,5),(10,10), fill="red"))

print("Design boundary ({} {})".format(designWidth, designHeight))
print("# of layers : {}".format(len(layerDic)))
print("# of tracks : {}".format(len(trackList)))
print("# of buses : {}".format(len(busDic)))
print("# of obstacles : {}".format(len(obsList)))


layerOrigin = {}
originX = [ scaling(x*(designWidth + emptySpace)) for x in range(0, len(layerDic)) ]
i = 0
for (layerName, layer) in layerDic.items():
    layerOrigin[layerName] = originX[i]
    print(originX[i])
    i+=1
    rectOrigin = (layerOrigin[layerName], 0)
    rectSize = scaling((designWidth, designHeight))
    svgFile.add(svgFile.rect(insert=rectOrigin, size=rectSize, fill='white', rx=None, ry=None, stroke='black', stroke_width=strokeWidthLayer, opacity='0.3'))

    textOriginX = rectOrigin[0] + rectSize[0]/2
    textOriginY = rectOrigin[1] + rectSize[1] + 400*(designWidth/150000) 
    textContent = layerName
    svgFile.add(svgFile.text(textContent, insert=(textOriginX, textOriginY), font_size = fontSize))




for track in trackList:
    lineStart = scaling((track.ll.x, track.ll.y))
    lineEnd = scaling((track.ur.x, track.ur.y))
    lineStart[0] += layerOrigin[track.layer] 
    lineEnd[0] += layerOrigin[track.layer] 
    svgFile.add(svgFile.line(start=lineStart, end=lineEnd, stroke='blue', stroke_width=strokeWidthObject, opacity='0.3'))

for (busName, bus) in busDic.items():
    for bit in bus.bits:
        for pin in bit.pins:
            rectOrigin = scaling((pin.rect.ll.x, pin.rect.ll.y))
            rectSize = scaling((pin.rect.ur.x - pin.rect.ll.x, pin.rect.ur.y - pin.rect.ll.y))
            rectOrigin[0] += layerOrigin[pin.layer]
            svgFile.add(svgFile.rect(insert=rectOrigin, size=rectSize, rx=None, ry=None, fill=colors[1], stroke='black', stroke_width=strokeWidthObject, fill_opacity='1'))


for obs in obsList:
    rectOrigin = scaling((obs.rect.ll.x, obs.rect.ll.y))
    rectSize = scaling((obs.rect.ur.x - obs.rect.ll.x, obs.rect.ur.y - obs.rect.ll.y))
    rectOrigin[0] += layerOrigin[obs.layer]
    svgFile.add(svgFile.rect(insert=rectOrigin, size=rectSize, rx=None, ry=None, fill=colors[0], stroke='black', stroke_width=strokeWidthObject, fill_opacity='0.1'))
    


svgFile.save()
print("SVG file saved")
script = "inkscape -f {} &".format(outputFile)
subprocess.call(script, shell=True)
