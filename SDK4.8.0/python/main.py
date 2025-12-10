import ctypes
import threading
import time

import numpy as np

global connectflag
global armdSteroTracking
global reference
global imageL, imageR, imageC

# network adaptor information struct
# name: adaptor name
# connectionType: connection type of the adaptor
# speed: network speed(Mb/s)
# linked: False-disconnect, True-connect
class NetworkAdaptorInfo(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char_p), ("connectionType", ctypes.c_char_p), ("speed", ctypes.c_int), ("linked", ctypes.c_bool)]

# RT device information struct
# hostname: host name
# IP: IP address
# MAC: MAC address
# deviceType: RT device type
# firmwareVersionL: firmware version number(0)
# firmwareVersionM: firmware version number(1)
# firmwareVersionH: firmware version number(2)
class RTDeviceInfo(ctypes.Structure):
    _fields_ = [("hostname", ctypes.c_char_p), ("IP", ctypes.c_char_p), ("MAC", ctypes.c_char_p),("deviceType", ctypes.c_int)\
        , ("firmwareVersionL", ctypes.c_int), ("firmwareVersionM", ctypes.c_int), ("firmwareVersionH", ctypes.c_int)]

# tool calibration structure
# name: tool name
# markerType: passive(false) or active(true) tool
# minNumMarker: minimum number of markers to be matched
# planeNum: plane number
# pointNum: point number
# dirx: virtual direction-x
# diry: virtual direction-y
# dirz: virtual direction-z
# pinx: tip position-x
# piny: tip position-y
# pinz: tip position-z
# calbError: calibration error
# maxFRE: maximum matching FRE
# maxAngle: maximum angle for matching
# structureType: tool structure, 0(single plane), 1(multiple planes), 2(non-plane)
# algorithmType: algorithm setting, 0(balance), 1(accuracy), 2(speed)
class ToolCalibration(ctypes.Structure):
    _fields_ = [ ("name",ctypes.c_char_p), ("markerType",ctypes.c_bool), ("minNumMarker", ctypes.c_int), ("planeNum", ctypes.c_int), ("pointNum", ctypes.c_int),\
                ("dirx", ctypes.c_double), ("diry", ctypes.c_double), ("dirz", ctypes.c_double),\
                ("pinx", ctypes.c_double), ("piny", ctypes.c_double), ("pinz", ctypes.c_double),\
                ("calbError", ctypes.c_double), ("maxFRE", ctypes.c_double), ("maxAngle", ctypes.c_double),\
                ("structureType", ctypes.c_int), ("algorithmType", ctypes.c_int)]

# point cloud data structure
# x, y, z: point position
# r, g, b: point color
class PointInf(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float), \
                ("r", ctypes.c_float), ("g", ctypes.c_float), ("b", ctypes.c_float)]

# tracking data structure
# matchStatus: tool matching status
# matchedPlane: tool matching plane index
# markerNum: tool marker number
# pointNum: tool point number
# tx, ty, tz: tool tip position
# ox, oy, oz: offset
# qw, qx, qy, qz: rotation quaternion
# rxx, ryx, rzx, rxy, ryy, rzy, rxz, ryz, rzz: rotation matrix (0,0), (0,1), (0,2), (1,0), (1,1), (1,2), (2,0), (2,1), (2,2)
# dx, dy, dz: virtual direction
# error: matching error
class TrackingData(ctypes.Structure):
    _fields_ = [("matchStatus", ctypes.c_int), ("matchedPlane", ctypes.c_int), ("markerNum", ctypes.c_int), ("pointNum", ctypes.c_int), \
                ("tx", ctypes.c_double), ("ty", ctypes.c_double), ("tz", ctypes.c_double), \
                ("ox", ctypes.c_double), ("oy", ctypes.c_double), ("oz", ctypes.c_double), \
                ("qw", ctypes.c_double), ("qx", ctypes.c_double), ("qy", ctypes.c_double), ("qz", ctypes.c_double), \
                ("rxx", ctypes.c_double), ("ryx", ctypes.c_double), ("rzx", ctypes.c_double), \
                ("rxy", ctypes.c_double), ("ryy", ctypes.c_double), ("rzy", ctypes.c_double), \
                ("rxz", ctypes.c_double), ("ryz", ctypes.c_double), ("rzz", ctypes.c_double), \
                ("dx", ctypes.c_double), ("dy", ctypes.c_double), ("dz", ctypes.c_double), \
                ("error", ctypes.c_double)]

# marker position structure
# x, y, z: marker position
# type: 0-unknow, 1-passive, 2-active
# RF: true-real or false-phantom
class MarkerPosition(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double), ("type", ctypes.c_int), ("RF", ctypes.c_bool)]

# driection structure
# dx, dy, dz: direction in x, y and z
class DirVector(ctypes.Structure):
    _fields_ = [("vx", ctypes.c_double), ("vy", ctypes.c_double), ("vz", ctypes.c_double)]

# tracking and imaging update thread
class TrackingUpdate(threading.Thread):
    def __init__(self, thread_name):
        super(TrackingUpdate, self).__init__(name = thread_name)

    def run(self):
        global connectflag
        global imageL, imageR

        # select a tool as reference
        referneceName = b'ToolHead'
        armdSteroTracking.setReferenceTool(referneceName)
        reference = TrackingData()

        # get infrared image size
        IFImageSize = (ctypes.c_int * 2)(0, 0)
        armdSteroTracking.getIFImageSize(IFImageSize)
        IFImageW = IFImageSize[0]
        IFImageH = IFImageSize[1]

        while(connectflag):
            # update tracking information
            armdSteroTracking.trackingUpdate()

            # print connection status
            print(armdSteroTracking.getConnectionStatus())

            # get gravity vector of the device
            gravityVector = armdSteroTracking.getGravityVector()
            print("Gravity vector is ", gravityVector.vx, gravityVector.vy, gravityVector.vz)

            # get all markers in the scene and print markers' tracking information
            n = ctypes.c_int()
            allMarkers = armdSteroTracking.getAllMarkers(ctypes.byref(n))
            print("All markers number is ", n.value)
            for i in range(n.value):
                print(allMarkers[i].x, allMarkers[i].y, allMarkers[i].z, allMarkers[i].type)

            # get tracking information of each tool
            trackingdata = armdSteroTracking.getTrackingData()

            # print tracking information of each tool
            for i in range(armdSteroTracking.getTrackingToolCurrentNum()):
                name = ctypes.c_char_p(b'name')
                armdSteroTracking.getTrackToolsName(i, name, 200)
                print("Toolname: ", name.value.decode())

                currentTool = trackingdata[i]
                if currentTool.matchStatus == 1:
                    markerdata = armdSteroTracking.getToolMarkerData(i)
                    pointdata = armdSteroTracking.getToolVirtualPointData(i)

                    matrix = np.array([[currentTool.rxx, currentTool.ryx, currentTool.rzx, currentTool.tx],\
                                       [currentTool.rxy, currentTool.ryy, currentTool.rzy, currentTool.ty],\
                                       [currentTool.rxz, currentTool.ryz, currentTool.rzz, currentTool.tz],\
                                       [0,0,0,1]])
                    print("    trackingdata: ", currentTool.matchedPlane, \
                          currentTool.tx, currentTool.ty, currentTool.tz, \
                          currentTool.ox, currentTool.oy, currentTool.oz, \
                          currentTool.qw, currentTool.qx, currentTool.qy, currentTool.qz, \
                          currentTool.rxx, currentTool.ryx, currentTool.rzx, \
                          currentTool.rxy, currentTool.ryy, currentTool.rzy, \
                          currentTool.rxz, currentTool.ryz, currentTool.rzz)

                    for j in range(currentTool.markerNum):
                        print("    markerdata-" + str(j), markerdata[j].x, markerdata[j].y, markerdata[j].z)

                    if (i == 0):
                        reference = currentTool
                    else:
                        outTrackingdata = armdSteroTracking.trackingDataProjection(reference, currentTool)
                        outMarker = armdSteroTracking.trackingDataProjection2(reference, currentTool, markerdata, True)
                        outPoint = armdSteroTracking.trackingDataProjection2(reference, currentTool, pointdata, False)

                        projectedMatrix = np.array(
                            [[outTrackingdata.rxx, outTrackingdata.ryx, outTrackingdata.rzx, outTrackingdata.tx], \
                             [outTrackingdata.rxy, outTrackingdata.ryy, outTrackingdata.rzy, outTrackingdata.ty], \
                             [outTrackingdata.rxz, outTrackingdata.ryz, outTrackingdata.rzz, outTrackingdata.tz], \
                             [0, 0, 0, 1]])

                        print("    projected trackingdata: ", outTrackingdata.matchedPlane, \
                              outTrackingdata.tx, outTrackingdata.ty, outTrackingdata.tz, \
                              outTrackingdata.ox, outTrackingdata.oy, outTrackingdata.oz, \
                              outTrackingdata.qw, outTrackingdata.qx, outTrackingdata.qy, outTrackingdata.qz, \
                              outTrackingdata.rxx, outTrackingdata.ryx, outTrackingdata.rzx, \
                              outTrackingdata.rxy, outTrackingdata.ryy, outTrackingdata.rzy, \
                              outTrackingdata.rxz, outTrackingdata.ryz, outTrackingdata.rzz)

                        for j in range(currentTool.markerNum):
                            print("    projected markerdata-" + str(j), outMarker[j].x, outMarker[j].y, outMarker[j].z)
                else:
                    print("Tool is missing")

            # get stray markers in the scene
            m = ctypes.c_int()
            unMatchedMarkers = armdSteroTracking.getUnMatchedMarkers(ctypes.byref(m))
            print("Stray markers number is ", m.value)
            for i in range(m.value):
                print(unMatchedMarkers[i].x, unMatchedMarkers[i].y, unMatchedMarkers[i].z)

            # get infrared imaging data
            leftImg = armdSteroTracking.getLeftImagingData()
            rightImg = armdSteroTracking.getRightImagingData()

            leftImgArray = np.ctypeslib.as_array(leftImg, shape=(IFImageH, IFImageW))
            imageL = np.ascontiguousarray(np.copy(leftImgArray), dtype=np.uint8)
            rightImgArray = np.ctypeslib.as_array(rightImg, shape=(IFImageH, IFImageW))
            imageR = np.ascontiguousarray(np.copy(rightImgArray), dtype=np.uint8)

def functionsetting():
    # autoDeviceScan(), return the number of currently scanned device
    armdSteroTracking.autoDeviceScan.argtypes = [ctypes.POINTER(ctypes.c_int)]

    # getNetAdaptor(), return the linked adaptor information
    armdSteroTracking.getNetAdaptor.restype = NetworkAdaptorInfo

    # getDeviceInfo(), return the linked RT device information
    armdSteroTracking.getDeviceInfo.restype = RTDeviceInfo

    # getTrackToolsMarker(), return the marker position in the coordinate of the tool
    armdSteroTracking.getTrackToolsMarker.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    armdSteroTracking.getTrackToolsMarker.restype = ctypes.POINTER(MarkerPosition)

    # getTrackToolsCalibration(), return the tool calibration information
    armdSteroTracking.getTrackToolsCalibration.argtype = [ctypes.c_int]
    armdSteroTracking.getTrackToolsCalibration.restype = ToolCalibration

    # getGravityVector(), return the device gravity vector
    armdSteroTracking.getGravityVector.restype = DirVector

    # getAllMarkers(n), return all markers' positions (MarkerPosition)
    # parameter n is the marker number
    armdSteroTracking.getAllMarkers.argtypes = [ctypes.POINTER(ctypes.c_int)]
    armdSteroTracking.getAllMarkers.restype = ctypes.POINTER(MarkerPosition)

    # getTrackingData(), return all tools' tracking data (TrackingData)
    armdSteroTracking.getTrackingData.restype = ctypes.POINTER(TrackingData)

    # getToolMarkerData(i), return the markers' tracking data of the selected tool
    # parameter i is the tool index
    armdSteroTracking.getToolMarkerData.argtypes = [ctypes.c_int]
    armdSteroTracking.getToolMarkerData.restype = ctypes.POINTER(MarkerPosition)

    # getToolVirtualPointData(i), return the virtual points' tracking data of the selected tool
    # parameter i is the tool index
    armdSteroTracking.getToolVirtualPointData.argtypes = [ctypes.c_int]
    armdSteroTracking.getToolVirtualPointData.restype = ctypes.POINTER(MarkerPosition)

    # getToolVirtualPointData(i), return the virtual points' tracking data of the selected tool
    # parameter i is the tool index
    armdSteroTracking.getToolDir.argtypes = [ctypes.c_int]
    armdSteroTracking.getToolDir.restype = DirVector

    # trackingDataProjection(ref, src), return the projected tracking data of the source tool (TrackingData) in the coordinate of the reference tool (TrackingData)
    # parameter ref and src are the tracking data of reference and source tools, respectively
    armdSteroTracking.trackingDataProjection.argtypes = [TrackingData, TrackingData]
    armdSteroTracking.trackingDataProjection.restype = TrackingData

    # trackingDataProjection2(ref, src, marker, flag), return the projected tracking data of markers/virtual points (MarkerPosition) of the source tool (TrackingData) in the coordinate of the reference tool (TrackingData)
    # parameter ref and src are the tracking data of reference and source tools, respectively.
    # parameter marker is the tracking date of markers/virtual points of the source tool
    # paramerter flag indicate the input marker is tracking date of markers (True) or virtual points (False)
    armdSteroTracking.trackingDataProjection2.argtypes = [TrackingData, TrackingData, ctypes.POINTER(MarkerPosition), ctypes.c_bool]
    armdSteroTracking.trackingDataProjection2.restype = ctypes.POINTER(MarkerPosition)

    # getUnMatchedMarkers(n), return all stray markers' positions (MarkerPosition)
    # parameter n is the stray marker number
    armdSteroTracking.getUnMatchedMarkers.argtypes = [ctypes.POINTER(ctypes.c_int)]
    armdSteroTracking.getUnMatchedMarkers.restype = ctypes.POINTER(MarkerPosition)

    # imaging
    armdSteroTracking.getIFImageSize.argtypes = [ctypes.POINTER(ctypes.c_int)]
    armdSteroTracking.getLeftImagingData.restype = ctypes.POINTER(ctypes.c_byte)
    armdSteroTracking.getRightImagingData.restype = ctypes.POINTER(ctypes.c_byte)

def generateArom():
    newtool = ToolCalibration()
    newtool.name = b'newTool'
    newtool.markerType = False
    newtool.minNumMarker = 3
    newtool.planeNum = 1
    newtool.pointNum = 0
    newtool.dirx = 1
    newtool.diry = 0
    newtool.dirz = 0
    newtool.pinx = 0
    newtool.piny = 0
    newtool.pinz = 0
    newtool.calbError = -1
    newtool.maxFRE = 0.5
    newtool.maxAngle = 60
    newtool.structureType = 0
    newtool.algorithmType = 1
    dir = DirVector()
    dir.vx = 0
    dir.vy = 0
    dir.vz = 0
    m1 = MarkerPosition()
    m1.x = 0
    m1.y = 0
    m1.z = 0
    m1.type = 0
    m1.RF = True
    m2 = MarkerPosition()
    m2.x = 0
    m2.y = 42
    m2.z = 29
    m2.type = 0
    m2.RF = True
    m3 = MarkerPosition()
    m3.x = 0
    m3.y = 0
    m3.z = 76
    m3.type = 0
    m3.RF = True
    markers = (MarkerPosition * 3)(m1, m2, m3)
    points = ctypes.POINTER(MarkerPosition)()
    planes = (ctypes.c_int * 1)(3)
    armdSteroTracking.generateAROM(b"./", newtool, ctypes.byref(planes), dir, ctypes.byref(markers), points, 5)

if __name__ == '__main__':
    # load the C++ dll
    armdSteroTracking = ctypes.cdll.LoadLibrary("./ARMDCSDLLWarpper.dll")

    global connectflag
    global imageL, imageR

    # set input and output type for ARMD function
    functionsetting()

    # generate a new Tool
    generateArom()

    # connect the device
    connectflag = 0
    hostname = b''
    auto = False
    if auto:
        # auto scan the reachable device
        num = ctypes.c_int(0)
        while (num.value == 0):
            armdSteroTracking.autoDeviceScan(ctypes.byref(num))
            if num.value != 0:
                name = ctypes.c_char_p(b'hostname')
                ip = ctypes.c_char_p(b'ip')
                armdSteroTracking.getScannedDevice(0, name, ip)
                hostname = name.value
                print(hostname)
    else:
        # or manually set hostname and connect RT device
        hostname = b'RT-PRO300113.local'

    connectStatue = armdSteroTracking.connectDevice(hostname)
    if connectStatue == 0:
        # device is connected
        connectflag = 1
        print("Connection successed!")

        # device type
        device = armdSteroTracking.getDeviceInfo()
        print(device.deviceType)

        # load all .arom files from the input path
        arompath = b'./Tool'
        armdSteroTracking.loadPassiveToolAROM(arompath)
        toolnum = armdSteroTracking.getTrackToolsNum()
        print("Tracking tool number is: ", toolnum)

        # set tracking data transmission type
        armdSteroTracking.setTrackingDataTransmissionType(1)

        # start tracking and imaging process and thread
        armdSteroTracking.setAreaDisplay(360, 363, 776, 418, 776, 418)
        armdSteroTracking.startTracking()
        armdSteroTracking.startImaging()
        TrackingUpdate("trackingUpdate").start()

        # show infrared imaging data
        showIFImage = False
        if(showIFImage):
            import matplotlib.pyplot as plt
            IFImageSize = (ctypes.c_int * 2)(0, 0)
            armdSteroTracking.getIFImageSize(IFImageSize)
            IFImageW = IFImageSize[0]
            IFImageH = IFImageSize[1]
            imageL = np.zeros((IFImageH, IFImageW))
            imageR = np.zeros((IFImageH, IFImageW))

            figIF = plt.figure()
            fig1 = figIF.add_subplot(1,2,1)
            fig2 = figIF.add_subplot(1,2,2)
            while(True):
                plt.pause(0.05)
                fig1.imshow(imageL, cmap='gray')
                fig2.imshow(imageR, cmap='gray')
    else:
        print(armdSteroTracking.getConnectionStatus())
        print("Connection failed!")