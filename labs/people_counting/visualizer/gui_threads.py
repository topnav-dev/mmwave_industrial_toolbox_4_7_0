from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog)
from PyQt5.QtGui import QPainter, QColor, QFont

import pyqtgraph as pg
import pyqtgraph.opengl as gl
import random
import numpy as np
import time

from oob_parser import uartParserSDK
from graphUtilities import *

class parseUartThread(QThread):
        fin = pyqtSignal('PyQt_PyObject')

        def __init__(self, uParser):
                QThread.__init__(self)
                self.parser = uParser

        def run(self):
                pointCloud = self.parser.readAndParseUart()
                self.fin.emit(pointCloud)

class sendCommandThread(QThread):
        done = pyqtSignal()
        def __init__(self, uParser, command):
                QThread.__init__(self)
                self.parser = uParser
                self.command = command

        def run(self):
            self.parser.sendLine(self.command)
            self.done.emit()



class update2DQTGraphThread(QThread):
        done = pyqtSignal()

        def __init__(self, pointCloud, targets, numTargets, indexes, numPoints, trailData, activeTrails, trailPlots, plot2D, gatingPlot):
                QThread.__init__(self)
                self.plot2D = plot2D
                self.gatingPlot = gatingPlot
                self.pointCloud=pointCloud
                self.numTargets=numTargets
                self.indexes=indexes
                self.numPoints = numPoints
                self.targets = targets
                self.trailData = trailData
                self.activeTrails = activeTrails
                self.trailPlots = trailPlots
                self.colorArray = ('r','g','b','w')

        def run(self):
                #plot point Cloud
                print('updating 2d points')
                toPlot = [{'pos':self.pointCloud[:,i]} for i in range(np.shape(self.pointCloud)[1])]
                self.plot2D.setData(toPlot)
                #plot trails
                for i in range(20):
                    lifespan = int(self.activeTrails[i,0])
                    if (lifespan > 0):
                        #plot trail
                        if (lifespan > 100):
                        	lifespan = 100
                        trDat = self.trailData[i,:lifespan,0:2]
                        #print(np.shape(trDat))
                        self.trailPlots[i].setData(trDat[:,0], trDat[:,1], pen=pg.mkPen(width=3,color=self.colorArray[i%3]))
                        self.trailPlots[i].setVisible(True)
                    else:
                        self.trailPlots[i].hide()

                #plot tracks
                if (self.numTargets > 0):
                    trackPlot = [{'pos':self.targets[1:3,i],'pen':pg.mkPen(width=25,color=self.colorArray[i%3])} for i in range(self.numTargets)]
                    self.gatingPlot.clear()
                    self.gatingPlot.setData(trackPlot)
                else:
                    self.gatingPlot.clear()
                self.done.emit()

class updateQTTargetThread3D(QThread):
    done = pyqtSignal()

    def __init__(self, pointCloud, targets, indexes, scatter, pcplot, numTargets, ellipsoids, coords, classifierOut=[], zRange=[-3, 3], gw=[], colorByIndex=False, drawTracks=True, bbox=[0,0,0,0,0,0], bbox_en=0):
        QThread.__init__(self)
        self.pointCloud = pointCloud
        self.targets = targets
        self.indexes = indexes
        self.scatter = scatter
        self.pcplot = pcplot
        self.colorArray = ('r','g','b','w')
        self.numTargets = numTargets
        self.ellipsoids = ellipsoids
        self.coordStr = coords
        self.classifierOut = classifierOut
        self.zRange = zRange
        self.gw = gw
        self.colorByIndex = colorByIndex
        self.drawTracks = drawTracks
        self.bbox = bbox
        self.bbox_en = bbox_en

    def drawTrack(self, index):
        #get necessary target data
        tid = int(self.targets[0,index])
        x = self.targets[1,index]
        y = self.targets[2,index]
        z = self.targets[3,index]
        xr = self.targets[11, index]
        yr = self.targets[10, index]
        zr = self.targets[12, index]
        edge_color = pg.glColor(self.colorArray[tid%3])
        track = self.ellipsoids[tid]
        #if classifier is on, set non human targets to white
        if (len(self.classifierOut) != 0):
            try:
                dTID = self.classifierOut[0].tolist()
                posit = dTID.index(tid)
                decision = self.classifierOut[1,posit]
            except Exception as ex:
                print ('Cannot find tid ', tid, ' in list:')
                print (dTID)
                print(ex)
            if(decision != 1):
                edge_color = pg.glColor('w')
        mesh = getBoxLinesCoords(x,y,z)
        track.setData(pos=mesh,color=edge_color,width=2,antialias=True,mode='lines')
        track.setVisible(True)
        #add text coordinates
        ctext = self.coordStr[tid]
        #print('coordstr = ',str(self.coordStr[tid]))
        ctext.setPosition(x,y,z)#+self.profile['sensorHeight'])
        ctext.setVisible(True)

    def run(self):
        #sanity check indexes = points
        if (len(self.indexes) != np.shape(self.pointCloud)[1]) and (len(self.indexes)):
            print ('I: ',len(self.indexes), ' P: ',  np.shape(self.pointCloud)[1])
        #clear all previous targets
        for e in self.ellipsoids:
            if (e.visible()):
                e.hide()
        for c in self.coordStr:
            if (c.visible()):
                c.hide()
        #remove points outside boundary box
        #only used in fall detection
        #if (self.bbox_en):
        #    to_delete = []
        #    for i in range(np.shape(self.pointCloud)[1]):
        #        x = self.pointCloud[0,i]
        #        y = self.pointCloud[1,i]
        #        z = self.pointCloud[2,i]
        #        if (x < self.bbox[0] or x > self.bbox[1]):
        #            to_delete.append(i)
        #        elif (y < self.bbox[2] or y > self.bbox[3]):
        #            to_delete.append(i)
        #        elif(z < self.bbox[4] or z > self.bbox[5]):
        #            to_delete.append(i)
        #    self.pointCloud=np.delete(self.pointCloud, to_delete, 1)
        #graph the points with colors
        toPlot = self.pointCloud[0:3,:].transpose()
        size = np.log2(self.pointCloud[4,:].transpose())
        colors = np.zeros((np.shape(self.pointCloud)[1], 4))
        if (self.colorByIndex):
            if (len(self.indexes) > 0):
                try:
                    for i in range(len(self.indexes)):
                        ind = int(self.indexes[i])
                        if (ind < 100):
                            color = pg.glColor(self.colorArray[ind%3])
                        else:
                            color = pg.glColor(self.colorArray[3])
                        colors[i,:] = color[:]
                    self.scatter.setData(pos=toPlot, color=colors, size=size)
                except:
                    print ('Index color fail')
                    self.scatter.setData(pos=toPlot, size=size)
            else:
                self.scatter.setData(pos=toPlot, size=size)
        else:
            for i in range(np.shape(self.pointCloud)[1]):
                #zs = self.zRange + (self.pointCloud[2,i] - self.zRange/2)
                zs = self.pointCloud[2,i]
                if (zs < self.zRange[0]) or (zs > self.zRange[1]):
                    colors[i]=pg.glColor('k')
                else:
                    colorRange = self.zRange[1]+abs(self.zRange[0])
                    #zs = colorRange/2 + zs
                    #zs = self.zRange[0]-zs
                    zs = self.zRange[1] - zs
                    #print(zs)
                    #print(self.zRange[1]+abs(self.zRange[0]))
                    #print(zs/colorRange)
                    colors[i]=pg.glColor(self.gw.getColor(abs(zs/colorRange)))
            self.scatter.setData(pos=toPlot, color=colors, size=size)
        #graph the targets
        if (self.drawTracks):
            for i in range(self.numTargets):
                try:
                    self.drawTrack(i)
                except Exception as e:
                    print(e)
                    print('No Plot Update')
        self.done.emit()


class updateHeightGraphs(QThread):
    done = pyqtSignal('PyQt_PyObject')

    def __init__(self, targetSize, plots, frameNum, tids):
        QThread.__init__(self)
        self.targetSize = targetSize
        self.plots = plots
        self.frameNum = frameNum
        self.tids = tids

    def run(self):
        out ={'success':0, 'height':[],'mH':[],'dH':[],'x':[]}
        #start by plotting height data, mean height, and delta height of first TID only
        if (len(self.tids) > 0):
            tid = int(self.tids[0])
            age = int(self.targetSize[4,tid,0])
            height = self.targetSize[0,tid,:]
            mH = self.targetSize[5,tid,:]
            dH = self.targetSize[6,tid,:]
            fNum = self.frameNum%100
            shift=99-fNum
            height=np.roll(height,shift)
            mH=np.roll(mH,shift)
            dH=np.roll(dH,shift)
            if age<100:
                height[:int(100-age)]=0
                mH[:int(100-age)]=0
                dH[:int(100-age)]=0
            x=np.arange(self.frameNum-100,self.frameNum)
            out['success']=1
            out['height']=height
            out['mH']=mH
            out['dH']=dH
            out['x']=x
            self.done.emit(out)
        else:
            self.done.emit(out)

