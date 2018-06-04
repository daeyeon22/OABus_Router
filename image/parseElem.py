class Point(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y

class Rectangle(object):
    def __init__(self,ll,ur):
        self.ll = ll
        self.ur = ur

class Bus(object):
    def __init__(self, name):
        self.name = name
        self.widthDic = {}
        self.bits = []

class Bit(object):
    def __init__(self, name, bus):
        self.name = name
        self.bus = bus
        self.pins = []

class Pin(object):
    def __init__(self, layer, bit, rect):
        self.layer = layer
        self.bit = bit
        self.rect = rect

class Track(object):
    def __init__(self, layer, ll, ur, width):
        self.layer = layer;
        self.ll = ll;
        self.ur = ur;
        self.width = width;

class Layer(object):
    def __init__(self, name, direction, spacing):
        self.name = name;
        self.direction = direction;
        self.spacing = spacing
        self.tracks = []

class Obstacle(object):
    def __init__(self, layer, rect):
        self.layer = layer
        self.rect = rect


