import sys
import re
from parseElem import Rectangle, Point, Bus, Bit, Pin, Track, Layer, Obstacle

def getLayerInfo(filePath):
    inFile = open(filePath, 'r')
    layerDic = {}
    lines = inFile.readlines()
    flag = False
    
    for i in range(0, len(lines)):
        vals = lines[i].replace('(',' ').replace(')',' ').split()
        print(vals)
        if len(vals) == 0:
            continue

        if vals[0] == "LAYERS":
            flag = True
            continue
        elif vals[0] == "ENDLAYERS":
            break    

        if flag == True:
            layerName = vals[0]
            direction = vals[1]
            spacing = vals[2]
            layer = Layer(layerName, direction, spacing)
            layerDic[layerName] = layer
            continue

    return layerDic


def getTrackInfo(filePath):
    inFile = open(filePath, 'r')
    trackList = []
    lines = inFile.readlines()
    flag = False

    for i in range(0, len(lines)):
        vals = lines[i].replace('(',' ').replace(')',' ').split()
        print(vals)

        if len(vals) == 0:
            continue

        if vals[0] == "TRACKS":
            flag = True
            continue
        elif vals[0] == "ENDTRACKS":
            break

        if flag == True:
            delim = ['(', ')']
            vals = [ elem for elem in vals if elem not in delim ]
            
            layerName = vals[0]
            llx = (int)(vals[1])
            lly = (int)(vals[2])
            urx = (int)(vals[3])
            ury = (int)(vals[4])
            width = (int)(vals[5])
            ll = Point(llx, lly)
            ur = Point(urx, ury)
            trackList.append(Track(layerName,ll,ur,width))
        
    return trackList

    
def getBusInfo(filePath):
    inFile = open(filePath, 'r')
    lines = inFile.readlines()
    busDic = {}
    bus = Bus('dummy')
    bit = Bit('dummy', bus.name)

    flag = False
    busFlag = False
    bitFlag = False
    for i in range(0, len(lines)):
        vals = lines[i].replace('(',' ').replace(')',' ').split()
        print(vals)

        if len(vals) == 0:
            continue

        if vals[0] == "BUSES":
            flag = True
            continue
        elif vals[0] == "ENDBUSES":
            break



        if flag == True:
            if vals[0] == "BUS":
                bus = Bus(vals[1])
                busFlag = True
                continue
            elif vals[0] == "ENDBUS":
                busDic[bus.name] = bus
                busFlag = False
                continue

        if busFlag == True:
            if vals[0] == "BIT":
                bit = Bit(vals[1], bus.name)
                bitFlag = True
                continue
            elif vals[0] == "ENDBIT":
                bus.bits.append(bit)
                bitFlag = False
                continue

        if bitFlag == True:
            delim = ['(', ')']
            vals = [ elem for elem in vals if elem not in delim ]
            layer = vals[0]
            llx = (int)(vals[1])
            lly = (int)(vals[2])
            urx = (int)(vals[3])
            ury = (int)(vals[4])
            rect = Rectangle(Point(llx,lly), Point(urx,ury))
            pin = Pin(layer,bit.name,rect)
            bit.pins.append(pin)

    return busDic

def getObstacleInfo(filePath):
    inFile = open(filePath, 'r')
    lines = inFile.readlines()
    obsList = []

    flag = False

    for i in range(0, len(lines)):
        vals = lines[i].replace('(',' ').replace(')',' ').split()
        print(vals)

        if len(vals) == 0:
            continue
        
        if vals[0] == "OBSTACLES":
            flag = True
            continue
        elif vals[0] == "ENDOBSTACLES":
            break

        if flag == True:
            delim = ['(', ')']
            vals = [ elem for elem in vals if elem not in delim ]
            layerName = vals[0]
            llx = (int)(vals[1])
            lly = (int)(vals[2])
            urx = (int)(vals[3])
            ury = (int)(vals[4])
            rect = Rectangle(Point(llx,lly), Point(urx,ury))
            obs = Obstacle(layerName, rect)
            obsList.append(obs)


    return obsList


def getDesignBoundary(filePath):
    inFile = open(filePath, 'r')
    lines = inFile.readlines()
    boundary = Rectangle(Point(0,0), Point(0,0))
    
    for i in range(0, len(lines)):
        vals = lines[i].replace('(',' ').replace(')',' ').split()
        
        if len(vals) == 0:
            continue
        
        if vals[0] == "DESIGN_BOUNDARY":
            delim = ['(', ')']
            vals = [ elem for elem in vals if elem not in delim ]
            llx = (int)(vals[1])
            lly = (int)(vals[2])
            urx = (int)(vals[3])
            ury = (int)(vals[4])
            boundary = Rectangle(Point(llx,lly), Point(urx,ury))
            break
    
    return boundary



