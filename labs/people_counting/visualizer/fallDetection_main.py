import sys
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog, QButtonGroup)
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.pgcollections import OrderedDict

import random
import numpy as np
import time
import math
import struct
import os

from oob_parser import uartParserSDK
from gui_threads import *
from graphUtilities import *
from gl_classes import GLTextItem
compileGui = 0
#only when compiling
if (compileGui):
    from fbs_runtime.application_context.PyQt5 import ApplicationContext

class Window(QDialog):
    def __init__(self, parent=None, size=[]):
        super(Window, self).__init__(parent)
        #when running Gree Demo
        self.Gree = 1
        # set window toolbar options, and title. #deb_gp
        self.setWindowFlags(
            Qt.Window |
            Qt.CustomizeWindowHint |
            Qt.WindowTitleHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )
        self.setWindowTitle("mmWave People Counting")

        if (1): #set to 1 to save terminal output to logFile, set 0 to show terminal output
            ts = time.localtime()
            terminalFileName = str('logData/logfile_'+ str(ts[2]) + str(ts[1]) + str(ts[0]) + '_' + str(ts[3]) + str(ts[4]) +'.txt')
            sys.stdout = open(terminalFileName, 'w')

        print('Python is ', struct.calcsize("P")*8, ' bit')
        print('Python version: ', sys.version_info)

        self.frameTime = 50
        self.graphFin = 1
        self.hGraphFin = 1
        self.threeD = 1
        self.lastFramePoints = np.zeros((5,1))
        self.plotTargets = 1
        self.frameNum = 0
        self.lastTID = []
        self.profile = {'startFreq': 60.25, 'numLoops': 64, 'numTx': 3, 'sensorHeight':3, 'maxRange':10, 'az_tilt':0, 'elev_tilt':0}
        self.lastFrameHadTargets = False
        self.sensorHeight = 1.5
        self.numFrameAvg = 20
        self.configSent = 0
        self.previousFirstZ = -1
        self.yzFlip = 0
        #timer to reset fall detected message
        self.fallTimer = QTimer()
        self.fallTimer.setSingleShot(True)
        self.fallTimer.timeout.connect(self.resetFallText)
        self.fallResetTimerOn = 0
        self.fallThresh = -0.22
        #color gradients
        self.Gradients = OrderedDict([
    ('bw', {'ticks': [(0.0, (0, 0, 0, 255)), (1, (255, 255, 255, 255))], 'mode': 'rgb'}),
    ('hot', {'ticks': [(0.3333, (185, 0, 0, 255)), (0.6666, (255, 220, 0, 255)), (1, (255, 255, 255, 255)), (0, (0, 0, 0, 255))], 'mode': 'rgb'}),
    ('jet', {'ticks': [(1, (166, 0, 0, 255)), (0.32247191011235954, (0, 255, 255, 255)), (0.11348314606741573, (0, 68, 255, 255)), (0.6797752808988764, (255, 255, 0, 255)), (0.902247191011236, (255, 0, 0, 255)), (0.0, (0, 0, 166, 255)), (0.5022471910112359, (0, 255, 0, 255))], 'mode': 'rgb'}),
    ('summer', {'ticks': [(1, (255, 255, 0, 255)), (0.0, (0, 170, 127, 255))], 'mode': 'rgb'} ),
    ('space', {'ticks': [(0.562, (75, 215, 227, 255)), (0.087, (255, 170, 0, 254)), (0.332, (0, 255, 0, 255)), (0.77, (85, 0, 255, 255)), (0.0, (255, 0, 0, 255)), (1.0, (255, 0, 127, 255))], 'mode': 'rgb'}),
    ('winter', {'ticks': [(1, (0, 255, 127, 255)), (0.0, (0, 0, 255, 255))], 'mode': 'rgb'}),
    ('spectrum2', {'ticks': [(1.0, (255, 0, 0, 255)), (0.0, (255, 0, 255, 255))], 'mode': 'hsv'}),
])
        cmap = 'spectrum2'
        if (cmap in self.Gradients):
            self.gradientMode = self.Gradients[cmap]
        self.zRange = [-3, 3]
        self.plotHeights = 1;
        #gui size
        if (size):
            left = 50
            top = 50
            width = math.ceil(size.width()*0.9)
            height = math.ceil(size.height()*0.9)
            self.setGeometry(left, top, width, height)
        #persistent point cloud
        self.previousCloud = np.zeros((6,1150,10))
        self.previousPointCount = np.zeros((10,1))
        #self.previousIndex = np.zeros((1,1150,10))
        #images
        self.standingPicture = QPixmap('images/stickFigureStanding.png')
        self.fallingPicture = QPixmap('images/stickFigureFalling.png')
        #remove points outside boundary box
        self.bbox = [-1000, 1000, -1000, 1000, -1000, 1000]

        #setup graph pyqtgraph
        self.plot3DQTGraph()
        self.colorGradient()
        self.heightPlots()
        self.fallDetData()
        #self.plot2DQTGraph()

        #add connect options
        self.setConnectionLayout()
        self.setStatsLayout()
        self.setPlotControlLayout()
        self.setConfigLayout()
        #self.setControlLayout()
        self.setUpBoundaryBoxControls()
        self.setSensorPositionControls()

        # set the layout
        #create tab for different graphing options
        self.graphTabs = QTabWidget();
        self.graphTabs.addTab(self.pcplot, '3D Plot')
        #self.graphTabs.addTab(self.legacyPlot, '2D Plot')
        self.graphTabs.currentChanged.connect(self.whoVisible)

        gridlay = QGridLayout()
        gridlay.addWidget(self.comBox, 0,0,1,1)
        gridlay.addWidget(self.statBox, 1,0,1,1)
        gridlay.addWidget(self.configBox,2,0,1,1)
        gridlay.addWidget(self.plotControlBox,3,0,1,1)
        gridlay.addWidget(self.boxTab,4,0,1,1)
        gridlay.addWidget(self.spBox,5,0,1,1)
        gridlay.addWidget(self.graphTabs,0,1,6,1)
        gridlay.addWidget(self.gw, 0, 2, 6, 1)
        gridlay.addWidget(self.demoData, 0,3,1,2)
        gridlay.addWidget(self.hPlot,1,3,4,2)
        gridlay.setColumnStretch(0,1)
        gridlay.setColumnStretch(1,3)
        self.setLayout(gridlay)
#
# left side pane layout
#
    def setConnectionLayout(self):
        self.comBox = QGroupBox('Connect to Com Ports')
        self.uartCom = QLineEdit('')    #deb_gp
        self.dataCom = QLineEdit('')    #deb_gp
        self.uartLabel = QLabel('UART COM:')
        self.dataLabel = QLabel('DATA COM:')
        self.connectStatus = QLabel('Not Connected')
        self.connectButton = QPushButton('Connect')
        self.connectButton.clicked.connect(self.connectCom)
        self.configLabel = QLabel('Config Type:')
        self.configType = QComboBox()
        self.configType.addItems(["3D People Counting", "SDK Out of Box Demo", "Long Range People Detection", "Sense and Detect HVAC Control", "(Legacy) 2D People Counting", "(Legacy): Overhead People Counting", "Capon3DAOP", "Replay"])
        self.comLayout = QGridLayout()
        self.comLayout.addWidget(self.uartLabel,0,0)
        self.comLayout.addWidget(self.uartCom,0,1)
        self.comLayout.addWidget(self.dataLabel,1,0)
        self.comLayout.addWidget(self.dataCom,1,1)
        self.comLayout.addWidget(self.configLabel,2,0)
        self.comLayout.addWidget(self.configType,2,1)
        self.comLayout.addWidget(self.connectButton,3,0)
        self.comLayout.addWidget(self.connectStatus,3,1)
        self.comBox.setLayout(self.comLayout)

    def setStatsLayout(self):
        self.statBox = QGroupBox('Statistics')
        self.frameNumDisplay = QLabel('Frame: 0')
        self.plotTimeDisplay = QLabel('Average Plot Time: 0 ms')
        self.numPointsDisplay = QLabel('Points: 0')
        self.numTargetsDisplay = QLabel('Targets: 0')
        self.statsLayout = QVBoxLayout()
        self.statsLayout.addWidget(self.frameNumDisplay)
        self.statsLayout.addWidget(self.plotTimeDisplay)
        self.statsLayout.addWidget(self.numPointsDisplay)
        self.statsLayout.addWidget(self.numTargetsDisplay)
        self.statBox.setLayout(self.statsLayout)

    def setPlotControlLayout(self):
        self.plotControlBox = QGroupBox('Plot Controls')
        self.staticclutter = QCheckBox('Display Static Points')
        self.plotByIndex = QCheckBox('Plot Point Color by Index')
        self.plotByHeight = QCheckBox('Plot Point Color By Height')
        self.plotTracks = QCheckBox('Plot Tracks')
        self.pointColorGroup = QButtonGroup()
        self.pointColorGroup.addButton(self.plotByIndex)
        self.pointColorGroup.addButton(self.plotByHeight)
        self.pointColorGroup.setExclusive(True)
        self.persistentFramesInput = QComboBox()
        self.persistentFramesInput.addItems(['1','2','3','4','5','6','7','8','9','10'])
        self.persistentFramesInput.setCurrentIndex(2)
        self.pFILabel = QLabel('# of Persistent Frames')
        self.orientationSelection = QComboBox()
        self.orientationSelection.addItems(['Side Mount', 'Overhead Mount'])
        self.orientationSelection.currentIndexChanged.connect(self.swapOrientations)
        self.oriLabel = QLabel('Orientation')
        self.fallThreshInput = QLineEdit(str(self.fallThresh))
        self.fallThreshInput.textEdited.connect(self.updateFallThresh)
        self.fallTLabel = QLabel('Fall Detection Threshold')
        self.plotControlLayout = QGridLayout()
        self.plotControlLayout.addWidget(self.plotByIndex, 0, 0,1,1)
        self.plotControlLayout.addWidget(self.plotByHeight, 1, 0,1,1)
        self.plotControlLayout.addWidget(self.plotTracks, 2, 0,1,1)
        self.plotControlLayout.addWidget(self.persistentFramesInput,4,0,1,1)
        self.plotControlLayout.addWidget(self.pFILabel,4,1,1,1)
        self.plotControlLayout.addWidget(self.staticclutter,3,0,1,1)
        self.plotControlLayout.addWidget(self.orientationSelection, 5,0,1,1)
        self.plotControlLayout.addWidget(self.oriLabel, 5,1,1,1)
        self.plotControlLayout.addWidget(self.fallThreshInput,6,0,1,1)
        self.plotControlLayout.addWidget(self.fallTLabel,6,1,1,1)
        self.plotControlBox.setLayout(self.plotControlLayout)
        #initialize button values
        self.plotByHeight.setChecked(True)
        self.plotByIndex.setChecked(False)
        self.plotTracks.setChecked(True)


    def setConfigLayout(self):
        self.configBox = QGroupBox('Configuration')
        self.selectConfig = QPushButton('Select Configuration')
        self.sendConfig = QPushButton('Send Configuration')
        self.selectConfig.clicked.connect(self.selectCfg)
        self.sendConfig.clicked.connect(self.sendCfg)       
        self.configTable = QTableWidget(5,2)
        #set parameter names
        self.configTable.setItem(0,0,QTableWidgetItem('Radar Parameter'))
        self.configTable.setItem(0,1,QTableWidgetItem('Value'))
        self.configTable.setItem(1,0,QTableWidgetItem('Max Range'))
        self.configTable.setItem(2,0,QTableWidgetItem('Range Resolution'))
        self.configTable.setItem(3,0,QTableWidgetItem('Max Velocity'))
        self.configTable.setItem(4,0,QTableWidgetItem('Velcoity Resolution'))
        self.configLayout = QVBoxLayout()
        self.configLayout.addWidget(self.selectConfig)
        self.configLayout.addWidget(self.sendConfig)
        self.configLayout.addWidget(self.configTable)       
        #self.configLayout.addStretch(1)
        self.configBox.setLayout(self.configLayout)

    def setControlLayout(self):
        self.controlBox = QGroupBox('Control')
        self.rangecfar = QSlider(Qt.Horizontal)
        self.azcfar = QSlider(Qt.Horizontal)
        self.snrthresh = QSlider(Qt.Horizontal)
        self.pointsthresh = QSlider(Qt.Horizontal)
        self.gatinggain = QSlider(Qt.Horizontal)
        self.staticclutter = QCheckBox('Static Clutter Removal')
        self.controlLayout = QVBoxLayout()
        self.rangelabel = QLabel('Range CFAR Threshold: ')
        self.azlabel = QLabel('Azimuth CFAR Threshold: ')
        self.snrlabel = QLabel('SNR Threshold: ')
        self.pointslabel = QLabel('Points Threshold: ')
        self.gatinglabel = QLabel('Gating Gain: ')
        self.controlLayout.addWidget(self.rangelabel)
        self.controlLayout.addWidget(self.rangecfar)
        self.controlLayout.addWidget(self.azlabel)
        self.controlLayout.addWidget(self.azcfar)
        self.controlLayout.addWidget(self.snrlabel)
        self.controlLayout.addWidget(self.snrthresh)
        self.controlLayout.addWidget(self.pointslabel)
        self.controlLayout.addWidget(self.pointsthresh)
        self.controlLayout.addWidget(self.gatinglabel)
        self.controlLayout.addWidget(self.gatinggain)
        self.controlLayout.addWidget(self.staticclutter)
        self.controlBox.setLayout(self.controlLayout)

    #boundary box control section
    def setBoxControlLayout(self, name):
        #set up one boundary box control
        boxControl = QGroupBox(name)
        #input boxes
        lx = QLineEdit('-6')
        rx = QLineEdit('6')
        ny = QLineEdit('0')
        fy = QLineEdit('6')
        bz = QLineEdit('-6')
        tz = QLineEdit('6')
        enable = QCheckBox()
        #labels
        lxL = QLabel('Left X')
        rxL = QLabel('Right X')
        nyL = QLabel('Near Y')
        fyL = QLabel('Far Y')
        bzL = QLabel('Bottom Z')
        tzL = QLabel('Top Z')
        enableL = QLabel('Enable Box')
        boxConLayout = QGridLayout()
        boxConLayout.addWidget(lxL, 0, 0,1,1)
        boxConLayout.addWidget(lx,0,1,1,1)
        boxConLayout.addWidget(rxL,0,2,1,1)
        boxConLayout.addWidget(rx,0,3,1,1)
        boxConLayout.addWidget(nyL, 1, 0,1,1)
        boxConLayout.addWidget(ny,1,1,1,1)
        boxConLayout.addWidget(fyL,1,2,1,1)
        boxConLayout.addWidget(fy,1,3,1,1)
        boxConLayout.addWidget(bzL, 2, 0,1,1)
        boxConLayout.addWidget(bz,2,1,1,1)
        boxConLayout.addWidget(tzL,2,2,1,1)
        boxConLayout.addWidget(tz,2,3,1,1)
        boxConLayout.addWidget(enableL,3,0,1,1)
        boxConLayout.addWidget(enable,3,1,1,1)
        boxControl.setLayout(boxConLayout)
        boundList = [lx,rx,ny,fy,bz,tz]
        for text in boundList:
            text.textEdited.connect(self.changeBoundaryBox)
        enable.stateChanged.connect(self.changeBoundaryBox)
        return {'boxCon':boxControl, 'boundList':boundList, 'checkEnable':enable, 'boxNum':-1}

    def setSensorPositionControls(self):
        self.az_tilt = QLineEdit('0')
        self.az_tiltL = QLabel('Azimuth Tilt')
        self.elev_tilt = QLineEdit('0')
        self.elev_tiltL = QLabel('Elevation Tilt')
        self.s_height = QLineEdit(str(self.profile['sensorHeight']))
        self.s_heightL = QLabel('Sensor Height')
        self.spLayout = QGridLayout()
        self.spLayout.addWidget(self.az_tilt,0,1,1,1)
        self.spLayout.addWidget(self.az_tiltL,0,0,1,1)
        self.spLayout.addWidget(self.elev_tilt,1,1,1,1)
        self.spLayout.addWidget(self.elev_tiltL,1,0,1,1)
        self.spLayout.addWidget(self.s_height,2,1,1,1)
        self.spLayout.addWidget(self.s_heightL,2,0,1,1)
        self.spBox = QGroupBox('Sensor Position')
        self.spBox.setLayout(self.spLayout)
        self.s_height.textEdited.connect(self.updateSensorPosition)
        self.az_tilt.textEdited.connect(self.updateSensorPosition)
        self.elev_tilt.textEdited.connect(self.updateSensorPosition)

    def updateSensorPosition(self):
        try:
            float(self.s_height.text())
            float(self.az_tilt.text())
            float(self.elev_tilt.text())
        except:
            print("fail to update")
            return
        command = "sensorPosition " + self.s_height.text() + " " + self.az_tilt.text() + " " + self.elev_tilt.text() + " \n"
        self.cThread = sendCommandThread(self.parser,command)
        self.cThread.start(priority=QThread.HighestPriority-2)
        self.gz.translate(dx=0,dy=0,dz=self.profile['sensorHeight'])
        self.profile['sensorHeight'] = float(self.s_height.text())
        self.gz.translate(dx=0,dy=0,dz=-self.profile['sensorHeight'])

    def setUpBoundaryBoxControls(self):
        #set up all boundary box controls
        self.boundaryBoxes = []
        self.boxTab = QTabWidget()
        for i in range(2):
            name = 'Box'+str(i)
            self.boundaryBoxes.append(self.setBoxControlLayout(name))
            toAdd = self.boundaryBoxes[i]
            toAdd['boxNum'] = i
            #if (i == 0):
            #    toAdd['checkEnable'].setChecked(True)
            self.boxTab.addTab(toAdd['boxCon'], name)
    #for live tuning when available
    def changeBoundaryBox(self):
        #send box values
        numBoxes = 0
        for box in self.boundaryBoxes:
            if(box['checkEnable'].isChecked()):
                numBoxes += 1
        boundaryString = "LiveScenery " + str(numBoxes) + " " 
        for box in self.boundaryBoxes:
            if(box['checkEnable'].isChecked()):
                for text in box['boundList']:
                    val = text.text()
                    val = val.replace(" ","")
                    try:
                        float(val)
                    except:
                        print('nothing here')
                        return
                    boundaryString += text.text() + " "
                self.drawBoundaryBox3d(box['boxNum'])
            else:
                print("Setting box ", box['boxNum'], " invisisble")
                self.boundaryBoxViz[box['boxNum']].setVisible(False)
                self.bottomSquare[box['boxNum']].setVisible(False)
        boundaryString += "\n"
        if (self.configSent):
            print(boundaryString)
            self.cThread = sendCommandThread(self.parser,boundaryString)
            self.cThread.start(priority=QThread.HighestPriority-2)
    #for tuning before demo start
    #def changeBoundaryBox(self):
    #    #send box values
    #    numBoxes = 0
    #    for box in self.boundaryBoxes:
    #        if(box['checkEnable'].isChecked()):
    #            numBoxes += 1
    #    boundaryString = "boundaryBox "
    #    staticString = "staticBoundaryBox " 
    #    flip = 1;
    #    for box in self.boundaryBoxes:
    #        if(box['checkEnable'].isChecked()):
    #            for text in box['boundList']:
    #                val = text.text()
    #                val = val.replace(" ","")
    #                try:
    #                    num=float(val)
    #                except:
    #                    print('nothing here')
    #                    return
    #                boundaryString += text.text() + " "
    #                num += flip*0.5
    #                flip = flip*-1
    #                staticString += str(num) + " "
    #            self.drawBoundaryBox3d(box['boxNum'])
    #        else:
    #            print("Setting box ", box['boxNum'], " invisisble")
    #            self.boundaryBoxViz[box['boxNum']].setVisible(False)
    #            self.bottomSquare[box['boxNum']].setVisible(False)
    #    boundaryString += "\n"
    #    staticString += "\n"
    #    self.cfg[self.boundaryLine] = boundaryString
    #    self.cfg[self.staticLine] = staticString

    def setBoundaryTextVals(self, profile):
        #update box text values based on config
        for box in self.boundaryBoxes:
            bList = box['boundList']
            bList[0].setText(str(profile['leftX']))
            bList[1].setText(str(profile['rightX']))
            bList[2].setText(str(profile['nearY']))
            bList[3].setText(str(profile['farY']))
            bList[4].setText(str(profile['bottomZ']))
            bList[5].setText(str(profile['topZ']))

    def drawBoundaryGrid(self, mRange):
        #re-draw the grid based on boundary box 2-dimensional
        bList = self.boundaryBoxes[self.boxTab.currentIndex()]['boundList']
        xL = mRange*2
        xC = 0
        yL = mRange
        yC = yL/2
        self.gz.resetTransform()
        if (self.orientationSelection.currentText() == 'Overhead Mount'):
            #self.gz.setSize(x=mRange*2, y=mRange*2)
            self.gz.setSize(x=8, y=8)
            self.gz.translate(dx=xC, dy=0, dz=-2)
        else:
            self.gz.setSize(x=mRange*2, y=mRange)
            #self.gz.translate(0,0,0)
            self.gz.translate(dx=xC, dy=yC, dz=-2)

    def drawBoundaryBox3d(self, index):
        #print(index)
        bList = self.boundaryBoxes[index]['boundList']
        xl = float(bList[0].text())
        xr = float(bList[1].text())
        yl = float(bList[2].text())
        yr = float(bList[3].text())
        if (self.orientationSelection.currentText() == 'Side Mount'):
            zl = float(bList[4].text())-self.profile['sensorHeight'] #set z low of bbox to world coords
            zr = float(bList[5].text())-self.profile['sensorHeight'] #set z hi of bbox to world coords
        elif (self.orientationSelection.currentText() == 'Overhead Mount'):
            zl = float(bList[4].text())-self.profile['sensorHeight']
            zr = float(bList[5].text())-self.profile['sensorHeight']
        #print(xl,yl,zl,xr,yr,zr)
        self.bbox = [xl, xr, yl, yr, zl, zr]
        boxLines = getBoxLines(xl,yl,zl,xr,yr,zr)
        squareLine = getSquareLines(xl,yl,xr,yr,zl)
        if (self.boundaryBoxViz[index].visible() == False):
            print ("Setting Box ", str(index), " to visible")
            self.boundaryBoxViz[index].setVisible(True)
            self.bottomSquare[index].setVisible(True)
        self.boundaryBoxViz[index].setData(pos=boxLines,color=pg.glColor('r'),width=2,antialias=True,mode='lines')
        self.bottomSquare[index].setData(pos=squareLine,color=pg.glColor('b'),width=2,antialias=True,mode='line_strip')
        print('Drew both boxes')

    def colorGradient(self):
        self.gw = pg.GradientWidget(orientation='right')
        self.gw.restoreState(self.gradientMode)

    def swapOrientations(self):
        print('orientation changed')
        print(self.orientationSelection.currentText())
        dsh = self.profile['sensorHeight']
        if (self.orientationSelection.currentText() == 'Overhead Mount'):
            self.yzFlip = 1
            self.evmBox.rotate(90,1,0,0)
            self.gz.resetTransform()
            #self.gz.setSize(x=self.profile['maxRange']*2, y=self.profile['maxRange']*2)
            self.gz.setSize(x=8, y=8)
            #self.gz.translate(0,0,0)
            self.gz.translate(dx=0, dy=0, dz=-self.profile['sensorHeight'])
            self.changeBoundaryBox()
            self.zRange = [-1*self.profile['sensorHeight'], 0]
        elif (self.orientationSelection.currentText() == 'Side Mount'):
            self.yzFlip = 0
            self.evmBox.rotate(-90,1,0,0)
            self.gz.resetTransform()
            self.gz.setSize(x=self.profile['maxRange']*2, y=self.profile['maxRange'])
            #self.gz.translate(0,0,0)
            self.gz.translate(dx=0, dy=self.profile['maxRange']/2, dz=-self.profile['sensorHeight'])
            self.changeBoundaryBox()
            self.zRange = [-3, 3]

    def heightPlots(self):
        self.heightPlots = {'instant':[],'mean':[],'deriv':[]}
        yMin = -3
        yMax = 3
        self.targetSize = np.zeros((7,20,100)) #100 frames of data, 20 tracks, height, relHeight, length, width, age, average height, height delta
        self.hPlot = QGroupBox('Height Plots')
        #setup instant height plot
        self.iHPlot = pg.PlotWidget()
        self.iHPlot.setYRange(yMin,yMax,padding=0.1)
        self.iHPlot.setTitle('Instantaneous Height')
        self.iHPlot.showGrid(x=False, y=True)
        self.iHPlot.setBackground('w')
        self.iHPlot.setMouseEnabled(False,False)
        self.iHPlot.hideAxis('bottom')
        #add curve
        self.heightPlots['instant']=pg.PlotCurveItem(pen=pg.mkPen(width=1, color='b'))
        self.iHPlot.addItem(self.heightPlots['instant'])
        #setup mean height plot
        self.mHPlot = pg.PlotWidget()
        self.mHPlot.setTitle('Average Height')
        self.mHPlot.setYRange(yMin,yMax,padding=0.1)
        self.mHPlot.showGrid(x=False,y=True)
        self.mHPlot.setBackground('w')
        self.mHPlot.setMouseEnabled(False,False)
        self.mHPlot.hideAxis('bottom')
        #add curve
        self.heightPlots['mean']=pg.PlotCurveItem(pen=pg.mkPen(width=1, color='b'))
        self.mHPlot.addItem(self.heightPlots['mean'])
        #setup Delta Height Plot
        self.dHPlot = pg.PlotWidget()
        self.dHPlot.setTitle('Delta Height')
        self.dHPlot.setYRange(-1,1,padding=0.1)
        self.dHPlot.showGrid(x=False,y=True)
        self.dHPlot.setBackground('w')
        self.dHPlot.setMouseEnabled(False,False)
        self.dHPlot.hideAxis('bottom')
        #add curve
        self.heightPlots['deriv']=pg.PlotCurveItem(pen=pg.mkPen(width=1, color='b'))
        self.dHPlot.addItem(self.heightPlots['deriv'])
        #add infinite line
        self.fallThreshMarker = pg.InfiniteLine(pos=self.fallThresh,angle=0,pen=pg.mkPen(width=1,color='r'))
        self.dHPlot.addItem(self.fallThreshMarker)
        #add plots to vertical layout
        self.heightLayout = QVBoxLayout()
        self.heightLayout.addWidget(self.iHPlot)
        self.heightLayout.addWidget(self.mHPlot)
        self.heightLayout.addWidget(self.dHPlot)
        self.hPlot.setLayout(self.heightLayout)

    def fallDetData(self):
        self.demoData = QGroupBox('Data')
        self.demoName = QLabel('Fall Detection Demo')
        self.numDetPeople = QLabel('Number of Detected People')
        self.fallDetEnabled = QLabel('Fall Detection Disabled - No People Detected')
        self.fallAlert = QLabel('Standing')
        self.fallPic = QLabel()
        self.fallPic.setPixmap(self.standingPicture)
        layoutDD = QGridLayout()
        layoutDD.addWidget(self.demoName,1,1)
        layoutDD.addWidget(self.numDetPeople,2,1)
        layoutDD.addWidget(self.fallDetEnabled,3,1)
        layoutDD.addWidget(self.fallAlert,1,2)
        layoutDD.addWidget(self.fallPic,2,2)
        self.demoData.setLayout(layoutDD)        

    def plot3DQTGraph(self):
        sphereDebug = 0
        self.pcplot = gl.GLViewWidget()
        dummy = np.zeros((1,3))
        #use if need to debug the ellipsoid drawing
        if (sphereDebug == 1):
            colorArray = ('r','g','b','w','y')
            colors = np.zeros((42,4))
            for c in range(0,7):
                colors[c*6:c*6+6,:] = pg.glColor(colorArray[c%5])
            sphereTrigs = getSphereMesh()
            self.sphere =gl.GLMeshItem(vertexes=sphereTrigs,smooth=False,drawEdges=True,edgeColor=pg.glColor('w'),drawFaces=False)
            self.pcplot.addItem(self.sphere)
        # create the background grids
        self.gz = gl.GLGridItem()
        self.gz.translate(0, 0, -1*self.profile['sensorHeight'])
        self.boundaryBoxViz = [gl.GLLinePlotItem(), gl.GLLinePlotItem()]
        self.bottomSquare = [gl.GLLinePlotItem(), gl.GLLinePlotItem()]
        for box in self.boundaryBoxViz:
            box.setVisible(False)
        self.scatter = gl.GLScatterPlotItem(size=5)
        self.scatter.setData(pos=dummy)
        self.pcplot.addItem(self.gz)
        self.pcplot.addItem(self.boundaryBoxViz[0])
        self.pcplot.addItem(self.boundaryBoxViz[1])
        self.pcplot.addItem(self.bottomSquare[0])
        self.pcplot.addItem(self.bottomSquare[1])
        self.pcplot.addItem(self.scatter)
        #create box to represent device
        verX = 0.0625
        verY = 0.05
        verZ = 0.125
        verts = np.empty((2,3,3))
        verts[0,0,:] = [-verX, 0, verZ]
        verts[0,1,:] = [-verX,0,-verZ]
        verts[0,2,:] = [verX,0,-verZ]
        verts[1,0,:] = [-verX, 0, verZ]
        verts[1,1,:] = [verX, 0, verZ]
        verts[1,2,:] = [verX, 0, -verZ]
        self.evmBox = gl.GLMeshItem(vertexes=verts,smooth=False,drawEdges=True,edgeColor=pg.glColor('r'),drawFaces=False)
        self.pcplot.addItem(self.evmBox)
        #add text items for tracks
        self.coordStr = []
        #coordinateString = GLTextItem()
        #coordinateString.setGLViewWidget(self.pcplot)
        #self.pcplot.addItem(coordinateString)
        #coordinateString.setPosition(1,1,1)
        #add mesh objects for ellipsoids
        self.ellipsoids = []
        #self.dataImages = []
        edgeColor = pg.glColor('k')
        for m in range(0,20):
            #add track object
            mesh = gl.GLLinePlotItem()
            mesh.setVisible(False)
            self.pcplot.addItem(mesh)
            self.ellipsoids.append(mesh)
            #add track coordinate string
            text = GLTextItem()
            text.setGLViewWidget(self.pcplot)
            text.setVisible(False)
            self.pcplot.addItem(text)
            self.coordStr.append(text)
            
    def updateGraph(self, parsedData):
        updateStart = int(round(time.time()*1000))
        self.useFilter = 0
        classifierOutput = []
        pointCloud = parsedData[0]
        targets = parsedData[1]
        indexes = parsedData[2]
        numPoints = parsedData[3]
        numTargets = parsedData[4]
        self.frameNum = parsedData[5]
        fail = parsedData[6]
        classifierOutput = parsedData[7]
        #print("indexes: ", indexes)
        fallDetEn = 0
        indicesIn = []
        #pass target XYZ vals and rotate due to elevation tilt angle (rotX uses Euler rotation around X axis)
        #print('elev_tilt = ',self.profile['elev_tilt'])
        #print('targets = ',targets)
        rotTargetDataX,rotTargetDataY,rotTargetDataZ = rotX (targets[1],targets[2],targets[3],-1*self.profile['elev_tilt'])
        #print('Rotated Data TID,X,Y = ' +str(rotTargetDataX)+', '+str(rotTargetDataY)+', '+str(rotTargetDataZ))
        targets[1] = rotTargetDataX
        targets[2] = rotTargetDataY
        targets[3] = rotTargetDataZ
        
        #pass pointCloud XYZ vals and rotate due to elevation tilt angle (rotX uses Euler rotation around X axis)
        for i in range(numPoints):
            #print('graph point cloud pt = ',pointCloud[:,i])
            #print('graph point cloud Y = ',pointCloud[1][i])
            #print('graph point cloud Z = ',pointCloud[2][i])
            rotPointDataX,rotPointDataY,rotPointDataZ = rotX ([pointCloud[0,i]],[pointCloud[1,i]],[pointCloud[2,i]],-1*self.profile['elev_tilt'])
            #print('graph point cloud rotated pt = ',rotPointDataX,rotPointDataY,rotPointDataZ)
            #print('graph point cloud Y = ',pointCloud[1][i])
            #print('graph point cloud Z = ',pointCloud[2][i])
            pointCloud[0,i] = rotPointDataX
            pointCloud[1,i] = rotPointDataY
            pointCloud[2,i] = rotPointDataZ                                                                                                             
        if (fail != 1):
            #left side
            pointstr = 'Points: '+str(numPoints)
            targetstr = 'Targets: '+str(numTargets)
            self.numPointsDisplay.setText(pointstr)
            self.numTargetsDisplay.setText(targetstr)
            #right side fall detection
            peopleStr = 'Number of Detected People: '+str(numTargets)
            if (numTargets == 0):
                fdestr = 'Fall Detection Disabled - No People Detected'
            elif (numTargets == 1):
                fdestr = 'Fall Detection Enabled'
                fallDetEn = 1
            elif (numTargets > 1):
                fdestr = 'Fall Detected Disabled - Too Many People'
            self.numDetPeople.setText(peopleStr)
            self.fallDetEnabled.setText(fdestr)
        if (len(targets) < 13):
            targets = []
            classifierOutput = []
        if (fail):
            return
        #check for mounting position
        if (self.yzFlip == 1):
            pointCloud[[1, 2]] = pointCloud[[2, 1]]
            pointCloud[2,:] = -1*pointCloud[2,:]
            targets[[2,3]] = targets[[3,2]]
            targets[3,:] = -1*targets[3,:]

        #remove static points
        if (self.configType.currentText() == '3D People Counting' or self.configType.currentText() == 'Capon3DAOP' or self.configType.currentText() == 'Sense and Detect HVAC Control'):
            if (not self.staticclutter.isChecked()):
                statics = np.where(pointCloud[3,:] == 0)
                try:
                    firstZ = statics[0][0]
                    numPoints = firstZ
                    pointCloud = pointCloud[:,:firstZ]
                    indexes = indexes[:,:self.previousFirstZ]
                    self.previousFirstZ = firstZ
                except:
                    firstZ = -1
        #point cloud persistence
        fNum = self.frameNum%10
        if (numPoints):
            self.previousCloud[:5,:numPoints,fNum] = pointCloud[:5,:numPoints]
            self.previousCloud[5,:len(indexes),fNum] = indexes
        self.previousPointCount[fNum]=numPoints
        #plotting 3D - get correct point cloud (persistent points and synchronize the frame)
        if (self.configType.currentText() == 'SDK3xPeopleCount'):
            pointIn = pointCloud
        else:
            totalPoints = 0
            persistentFrames = int(self.persistentFramesInput.currentText())
            #allocate new array for all the points
            for i in range(1,persistentFrames+1):
                totalPoints += self.previousPointCount[fNum-i]
            pointIn = np.zeros((5,int(totalPoints)))
            indicesIn = np.ones((1, int(totalPoints)))*255
            totalPoints = 0
            #fill array for indices and points
            for i in range(1,persistentFrames+1):
                prevCount = int(self.previousPointCount[fNum-i])
                pointIn[:,totalPoints:totalPoints+prevCount] = self.previousCloud[:5,:prevCount,fNum-i]
                if (numTargets > 0):
                    indicesIn[0,totalPoints:totalPoints+prevCount] = self.previousCloud[5,:prevCount,fNum-i]
                totalPoints+=prevCount
        if (self.graphFin):
            self.plotstart = int(round(time.time()*1000))
            self.graphFin = 0
            if (self.threeD):
                try:
                    indicesIn = indicesIn[0,:]
                except:
                    indicesIn = []
                self.get_thread = updateQTTargetThread3D(pointIn, targets, indicesIn, self.scatter, self.pcplot, numTargets, self.ellipsoids, self.coordStr, classifierOutput, self.zRange, self.gw, self.plotByIndex.isChecked(), self.plotTracks.isChecked(), self.bbox,self.boundaryBoxes[0]['checkEnable'].isChecked())
                self.get_thread.done.connect(self.graphDone)
                self.get_thread.start(priority=QThread.HighestPriority-1)
            else:
                npc = pointIn[0:2,:]
                print (np.shape(npc))
                self.legacyThread = update2DQTGraphThread(npc, targets, numTargets, indexes, numPoints, self.trailData, self.activeTrail, self.trails, self.scatter2D, self.gatingScatter)
                self.legacyThread.done.connect(self.graphDone)
                self.legacyThread.start(priority=QThread.HighestPriority-1)
        else:
            return
        #height plotting - only if 3D plot is good to go
        #first loop is instantaneous absolute height, relative height, length, and width
        if (self.configType.currentText() == '3D People Counting'):
            pointIn = self.previousCloud[:,:int(self.previousPointCount[fNum-1]),fNum-1]
        elif (self.configType.currentText() == 'Long Range People Detection'):
            pointIn = self.previousCloud[:,:int(self.previousPointCount[fNum]),fNum]
        fNum = self.frameNum%100
        for t in range(numTargets):
            tid = int(targets[t,0])
            #print("TID: ", tid)
            tIndices = np.where(np.array(indexes) == tid)
            #print("Indexex: ", np.size(indexes)," , pointIn: ",  np.size(pointIn,1))
            if (np.size(tIndices) and np.size(pointIn,1) == np.size(indexes)):
                #print("indices statement")
                tPoints = np.take(pointIn, tIndices, 1)
                self.targetSize[0,tid,fNum] = np.amax(tPoints[2,0,:]) + self.sensorHeight #absolute height
                self.targetSize[1,tid,fNum] = np.amax(tPoints[2,0,:]) - np.amin(tPoints[2,:]) #relative height
                self.targetSize[2,tid,fNum] = np.amax(tPoints[1,0,:]) - np.amin(tPoints[1,:]) #length
                self.targetSize[3,tid,fNum] = np.amax(tPoints[0,0,:]) - np.amin(tPoints[0,:]) #width
                if tid in self.lastTID:
                    #print("lastTID")
                    self.targetSize[4,tid,0] = self.targetSize[4,tid,0]+1
                    age = self.targetSize[4,tid,0] #age
                    #a = 1/self.numFrameAvg*self.targetSize[0,tid,fNum]
                    #b = ((self.numFrameAvg-1)/self.numFrameAvg)*self.targetSize[5,tid,(fNum-1)%100]
                    #c = a + b
                    #print('a: ',a,' b: ', b,' c: ',c)
                    self.targetSize[5,tid,fNum]= (1/self.numFrameAvg*self.targetSize[0,tid,fNum])+((self.numFrameAvg-1)/self.numFrameAvg)*self.targetSize[5,tid,(fNum-1)%100] #avg height over 10 frames
                    #need 2 seconds to get accurate height
                    if(age>40):
                        self.targetSize[6,tid,fNum]= self.targetSize[5,tid,fNum]-self.targetSize[5,tid,(fNum-10)%100] #delta height after 10 frames
                        if (self.targetSize[6,tid,fNum] < self.fallThresh and fallDetEn):
                            self.fallAlert.setText('Fallen!')
                            self.fallPic.setPixmap(self.fallingPicture)
                            if (self.fallResetTimerOn == 0):
                                self.fallResetTimerOn = 1
                                self.fallTimer.start(5000) #5 second timer
                    else:
                        self.targetSize[6,tid,fNum] = 0
                else:
                    self.targetSize[4,tid,0] = 1
                    self.targetSize[5,tid,fNum]= self.targetSize[0,tid,fNum]
                    self.targetSize[6,tid,fNum]=0
            #nothing detected use values from last frame
            else:
                self.targetSize[0,tid,fNum]=self.targetSize[0,tid,fNum-1]
                self.targetSize[1,tid,fNum]=self.targetSize[1,tid,fNum-1]
                self.targetSize[2,tid,fNum]=self.targetSize[2,tid,fNum-1]
                self.targetSize[3,tid,fNum]=self.targetSize[3,tid,fNum-1]
                self.targetSize[4,tid,fNum]=self.targetSize[4,tid,fNum-1]
                self.targetSize[5,tid,fNum]=self.targetSize[5,tid,fNum-1]
                self.targetSize[6,tid,fNum]=self.targetSize[6,tid,fNum-1]

        #state tracking
        if (numTargets > 0):
            self.lastFrameHadTargets = True
        else:
            self.lastFrameHadTargets = False
        if (numTargets):
            self.lastTID = targets[0,:]
        else:
            self.lastTID = []
        #update height plot
        if (self.hGraphFin):
            self.hGraphFin = 0
            self.heightThread = updateHeightGraphs(self.targetSize, self.heightPlots, self.frameNum, self.lastTID)
            self.heightThread.done.connect(self.heightGraphDone)
            self.heightThread.start(priority=QThread.HighestPriority-2)

    def graphDone(self):
        plotend = int(round(time.time()*1000))
        plotime = plotend - self.plotstart
        try:
            if (self.frameNum > 1):
                self.averagePlot = (plotime*1/self.frameNum) + (self.averagePlot*(self.frameNum-1)/(self.frameNum))
            else:
                self.averagePlot = plotime
        except:
            self.averagePlot = plotime
        self.graphFin = 1
        pltstr = 'Average Plot time: '+str(plotime)[:5] + ' ms'
        fnstr = 'Frame: '+str(self.frameNum)
        self.frameNumDisplay.setText(fnstr)
        self.plotTimeDisplay.setText(pltstr)

    def heightGraphDone(self, toPlot):
        self.hGraphFin = 1
        #print("hGraphFin")
        success = toPlot['success']
        if (success):
            height = toPlot['height']
            mH=toPlot['mH']
            dH=toPlot['dH']
            x=toPlot['x']
            self.heightPlots['instant'].setData(x,height)
            self.heightPlots['mean'].setData(x,mH)
            self.heightPlots['deriv'].setData(x,dH)

    def resetFallText(self):
        self.fallAlert.setText('Standing')
        self.fallPic.setPixmap(self.standingPicture)
        self.fallResetTimerOn = 0

    def updateFallThresh(self):
        try:
            newThresh = float(self.fallThreshInput.text())
            self.fallThresh = newThresh
            self.fallThreshMarker.setPos(self.fallThresh)
        except:
            print('No numberical threshold')

    def connectCom(self):
        #get parser
        self.parser = uartParserSDK(type=self.configType.currentText())
        self.parser.frameTime = self.frameTime
        print('Parser type: ',self.configType.currentText())
        #init threads and timers
        self.uart_thread = parseUartThread(self.parser)
        if (self.configType.currentText() != 'Replay'):
            self.uart_thread.fin.connect(self.parseData)
        self.uart_thread.fin.connect(self.updateGraph)
        self.parseTimer = QTimer()
        self.parseTimer.setSingleShot(False)
        self.parseTimer.timeout.connect(self.parseData)        
        try:
            uart = "COM"+ self.uartCom.text()       #deb_gp
            data = "COM"+ self.dataCom.text()       #deb_gp
#TODO: find the serial ports automatically.
            self.parser.connectComPorts(uart, data)
            self.connectStatus.setText('Connected')     #deb_gp
            self.connectButton.setText('Disconnect')    #deb_gp
#TODO: create the disconnect button action
        except Exception as e:
            print (e)
            self.connectStatus.setText('Unable to Connect')
        if (self.configType.currentText() == "Replay"):
            self.connectStatus.setText('Replay')
        if (self.configType.currentText() == "Long Range People Detection"):
            self.frameTime = 400
#
# Select and parse the configuration file
# TODO select the cfgfile automatically based on the profile.
    def selectCfg(self):
        try:
            self.parseCfg(self.selectFile())
        except Exception as e:
            print(e)
            print('No cfg file selected!')
    def selectFile(self):
        #search for latest mmwave toolbox
        largest = -1
        root = "C:/ti/"
        iwrTools = "mmwave_industrial_toolbox"
        dirs = os.listdir(root)
        for name in dirs:
            if len(name) > len(iwrTools):
                if (name[:25] == iwrTools):
                    numstr = name[26:31]
                    #print(numstr)
                    version = int(numstr)
                    if (version > largest):
                        largest = version
                        toolbox = name
        #user may not have correct directory, or OS scan may not find it
        try:
            configDirectory = root+toolbox+"/labs/people_counting/visualizer/chirp_configs/"
        except:
            configDirectory = root
        
        fd = QFileDialog()
        filt = "cfg(*.cfg)"
        filename = fd.getOpenFileName(directory=configDirectory,filter=filt)    #deb_gp - added folder name
        return filename[0]
    def parseCfg(self, fname):
        cfg_file = open(fname, 'r')
        self.cfg = cfg_file.readlines()
        counter = 0
        chirpCount = 0
        for line in self.cfg:
            args = line.split()
            if (len(args) > 0):
                if (args[0] == 'cfarCfg'):
                    zy = 4
                    #self.cfarConfig = {args[10], args[11], '1'}
                elif (args[0] == 'AllocationParam'):
                    zy=3
                    #self.allocConfig = tuple(args[1:6])
                elif (args[0] == 'GatingParam'):
                    zy=2
                    #self.gatingConfig = tuple(args[1:4])
                elif (args[0] == 'SceneryParam' or args[0] == 'boundaryBox'):
                    self.boundaryLine = counter
                    self.profile['leftX'] = float(args[1])
                    self.profile['rightX'] = float(args[2])
                    self.profile['nearY'] = float(args[3])
                    self.profile['farY'] = float(args[4])
                    if (self.configType.currentText() == '3D People Counting'):
                        self.profile['bottomZ'] = float(args[5])
                        self.profile['topZ'] = float(args[6])
                    else:
                        self.profile['bottomZ'] = float(-3)
                        self.profile['topZ'] = float(3)
                    self.setBoundaryTextVals(self.profile)
                    self.boundaryBoxes[0]['checkEnable'].setChecked(True)
                elif (args[0] == 'staticBoundaryBox'):
                    self.staticLine = counter
                elif (args[0] == 'profileCfg'):
                    self.profile['startFreq'] = float(args[2])
                    self.profile['idle'] = float(args[3])
                    self.profile['adcStart'] = float(args[4])
                    self.profile['rampEnd'] = float(args[5])
                    self.profile['slope'] = float(args[8])
                    self.profile['samples'] = float(args[10])
                    self.profile['sampleRate'] = float(args[11])
                    print(self.profile)
                elif (args[0] == 'frameCfg'):
                    self.profile['numLoops'] = float(args[3])
                    self.profile['numTx'] = float(args[2])+1
                elif (args[0] == 'chirpCfg'):
                    chirpCount += 1
                elif (args[0] == 'sensorPosition'):
                    self.profile['sensorHeight'] = float(args[1])
                    self.profile['az_tilt'] = float(args[2])
                    self.profile['elev_tilt'] = float(args[3])
            counter += 1
        self.profile['maxRange'] = self.profile['sampleRate']*1e3*0.9*3e8/(2*self.profile['slope']*1e12)
        #update boundary box
        #self.drawBoundaryGrid(self.profile['maxRange']) #2D legacy version
        self.gz.translate(0, 0, 3-self.profile['sensorHeight']) #reposition the ground level to be at sensor height
        self.changeBoundaryBox() #redraw bbox from cfg file values                                                                                                                   
        #update chirp table values
        bw = self.profile['samples']/(self.profile['sampleRate']*1e3)*self.profile['slope']*1e12
        rangeRes = 3e8/(2*bw)
        Tc = (self.profile['idle']*1e-6 + self.profile['rampEnd']*1e-6)*chirpCount
        lda = 3e8/(self.profile['startFreq']*1e9)
        maxVelocity = lda/(4*Tc)
        velocityRes = lda/(2*Tc*self.profile['numLoops']*self.profile['numTx'])
        self.configTable.setItem(1,1,QTableWidgetItem(str(self.profile['maxRange'])[:5]))
        self.configTable.setItem(2,1,QTableWidgetItem(str(rangeRes)[:5]))
        self.configTable.setItem(3,1,QTableWidgetItem(str(maxVelocity)[:5]))
        self.configTable.setItem(4,1,QTableWidgetItem(str(velocityRes)[:5]))
        #update sensor position
        self.az_tilt.setText(str(self.profile['az_tilt']))
        self.elev_tilt.setText(str(self.profile['elev_tilt']))
        self.s_height.setText(str(self.profile['sensorHeight']))

    def sendCfg(self):
        try:
            if (self.configType.currentText() != "Replay"):
                self.parser.sendCfg(self.cfg)
                self.configSent = 1
            self.parseTimer.start(self.frameTime)
        except Exception as e:
            print(e)
            print ('No cfg file selected!')

    # Needed ?? deb_gp
    # def setParser(self, uParser):
    #     self.parser = uParser

    def parseData(self):
        self.uart_thread.start(priority=QThread.HighestPriority)

    def whoVisible(self):
        if (self.threeD):
            self.threeD = 0
        else:
            self.threeD = 1
        print('3d: ', self.threeD)

if __name__ == '__main__':
    if (compileGui):
        appctxt = ApplicationContext()
        app = QApplication(sys.argv)
        screen = app.primaryScreen()
        size = screen.size()
        main = Window(size=size)
        main.show()
        exit_code = appctxt.app.exec_()
        sys.exit(exit_code)
    else:
        app = QApplication(sys.argv)
        screen = app.primaryScreen()
        size = screen.size()
        main = Window(size=size)
        main.show()
        sys.exit(app.exec_())
