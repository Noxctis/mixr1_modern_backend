from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QMainWindow, QPushButton, QApplication, QWidget, QFrame, QVBoxLayout, QGridLayout, QLCDNumber, QLabel, QMessageBox
from PyQt5.QtGui import QPalette, QFont, QColor
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
#Code added here
from pyqtgraph.Qt import QtGui, QtCore
from sklearn import linear_model
import pandas as pd
import commcmd
# from locale import atoi
import time
import os,sys
import shutil
import string
import ctypes
import mplcursors
from collections import deque

# from ctypes import windll
# import operator
# Hardcoded here due to stupid WinDLL stuff that does not give us access to these values.
DRIVE_REMOVABLE = 2 # [CodeStyle: Windows Enum value]
cwd = os.getcwd()
fw_l4 = cwd + '\\'+'FWL4CD'+'\\'+'VL53L4CDRanging.bin'
maxsteps=10
totalheight=0
OG_result=pd.DataFrame()
def mean(data):
    """Return the sample arithmetic mean of data."""
    n = len(data)
    if n < 1:
        raise ValueError('mean requires at least one data point')
    return sum(data)/n # in Python 2 use sum(data)/float(n)

def _ss(data):
    """Return sum of square deviations of sequence data."""
    c = mean(data)
    ss = sum((x-c)**2 for x in data)
    return ss

def stdev(data, ddof=0):
    """Calculates the population standard deviation
    by default; specify ddof=1 to compute the sample
    standard deviation."""
    n = len(data)
    if n < 2:
        raise ValueError('variance requires at least two data points')
    ss = _ss(data)
    pvar = ss/(n-ddof)
    return pvar**0.5
class CharacterizationPlot(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=5, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        # self.axes.grid(color='lightgrey')
        super(CharacterizationPlot, self).__init__(fig)
        self.init_figure()
        # fig.canvas.mpl_connect('pick_event', self.onpick)

    def show_datapoints(self,sel):
        xi, yi = sel[0], sel[0]
        xi, yi = xi._xorig.tolist(), yi._yorig.tolist()
        sel.annotation.set_text(
            'x: ' + str(xi[round(sel.target.index)]) + '\n' + 'y: ' + str(yi[round(sel.target.index)]))
    def clear(self):
        # for ax in self.axes:
        #     ax.clear()
        self.axes.cla()
        # self.Liquidlevel.clear()
        self.Rangingresult.clear()

        self.init_figure()
        self.draw()
    def init_figure(self):
        global totalheight
        steps = 10
        self.axes.grid(color='lightgrey')
        self.Expectedresult=[]
        self.Rangingresult = []
        self.totalheight=totalheight
        self.Liquidlevel = []
        self.iter_dev_list=[]
        self.offalgo_pred=[]
        self.Characteriz=1
        # self.Liquidlevel = [90,80,70]
        ratio = (self.height() / self.totalheight)
        intv = ((self.height() / steps))
        for lvl in range(maxsteps):
            if lvl==0:
                continue
            self.Liquidlevel.append(lvl*10)
            self.Expectedresult.append( (int((intv * lvl) / ratio)))
        self.Liquidlevel=self.Liquidlevel[::-1]
        self.axes.set(title="Characterization plot", xlabel="Water level (%)", ylabel="Distance (mm)")
        # print('====',self.Liquidlevel,self.Expectedresult)
        line=self.axes.plot(self.Liquidlevel, self.Expectedresult, color='blue', marker='o',label='ExpectedRes')
        mplcursors.cursor(line,hover=True)
        # mplcursors.cursor(self.axes).connect('add', self.show_datapoints)
        # self.fig.canvas.mpl_connect("motion_notify_event", self.on_plot_hover)
    def udpate_char(self,charcter):
        self.Characteriz = charcter
    def update_figure(self,Rangingresult):
        self.axes.set(title="Characterization plot", xlabel="Water level (%)", ylabel="Distance (mm)")
        self.axes.grid(color='lightgrey')
        line1 = self.axes.plot(self.Liquidlevel, self.Expectedresult, color='blue', marker='o')
        line2 = self.axes.plot(self.Liquidlevel, Rangingresult, color='red', marker='o')
        # mplcursors.cursor(line1, hover=True)
        mplcursors.cursor(line2, hover=True)
        self.draw()

    def regression_plot(self,characteAll):
        df = pd.DataFrame(characteAll)
        total_rows = df.shape[0]
        ind_lst = []
        iter_dev_list=[]
        r_sq=0
        value = int(round(((self.Characteriz * 0.7) - self.Characteriz), 0)+1) #Why extra one bcz expected result is appened which need to discard
        
        for cnt in ((df.iterrows())):
            ind_lst.append('C'+str(total_rows))
            total_rows-= 1
        df.index= ind_lst
        #Renameing columns
        for col in df.columns:
            col_ch = str(col) + '_iter'
            df = df.rename(columns={col: col_ch})
        df['ExpectedRes'] = self.Expectedresult
        print('DATAFRAM2', df)
        reg = linear_model.LinearRegression()
        #Trained data, Except 30% one which is using for test
        for col in df.columns[:-2]:
            print('col__iter',col)
            reg.fit(df[[col]], df['ExpectedRes'])
        # y=mx+C
        print('col--',df[df.columns[-2]])
        r_sq = reg.score(df[[df.columns[-2]]], df['ExpectedRes'])
        print('coefficient of determinationR^2:', r_sq)
        print('Coefficent=m',reg.coef_)
        print('Intercept=C',reg.intercept_)
        dist = reg.predict(df[['1_iter']])
        df['reg_predication'] = dist
        df['Liquidlevel'] = self.Liquidlevel
        # print(df)
        # Add OGalgo code
        self.OffsetGainAlgo(df)
        # To use Linear regression then uncomment below lines
        # line = self.axes.plot(df['Liquidlevel'], df['reg_predication'], color='green', marker='o',label='RegressionRes')
        line = self.axes.plot(df['Liquidlevel'], df['offalgo_pred'], color='black', marker='o', label='FinalRes')
        self.axes.legend()
        mplcursors.cursor(line, hover=True)
        self.draw()
        return reg.coef_,reg.intercept_
    def OffsetGainAlgo(self,df):
        global OG_result
        for col in df.columns[:-4]:
            print(col)
            col_ch = col + '_dev'
            self.iter_dev_list.append(col_ch)
            df[col_ch] = abs(df[col] - df['ExpectedRes'])
        df['mean_dev'] = df[self.iter_dev_list].mean(axis=1)
        df['OC_Val'] = df['mean_dev'].rolling(window=2).mean().fillna(method='bfill')
        print('OffsetGainAlgo--\n',df)
        #Prepare predication algo based on water level postion
        for index, val in df.iterrows():
            # Find the position of water level
            if (df.loc[index, '1_iter'] < self.totalheight):
                # if value is less than 50% of index then consider the lower one
                value = (df.loc[index, 'ExpectedRes'] - (df.loc[index, 'ExpectedRes'] * 0.5))
                # print('index value--========================', index, df.loc[index, '1_iter'], value)
                if (df.loc[index, '1_iter'] <= value):
                    pos = self.findprevindex(index)
                else:
                    pos = index
            else:
                if (df.loc[index, '1_iter'] >= self.totalheight):
                    pos = '99'
            self.offalgo_pred.append(self.waterlevel_range(df.loc[index, '1_iter'], df,pos))
        df['offalgo_pred']=self.offalgo_pred
        OG_result=df[['ExpectedRes','OC_Val']]
        print('offalgo_pred', self.offalgo_pred)
        OG_result.to_csv('OffGainVal.csv')
        df.to_csv('Finalresult.csv')
        return self.offalgo_pred
    def findprevindex(self,index):
        index_prv_pos = '99'
        print('findprevindex-->', index)
        if (index == 'C1'):
            index_prv_pos = 'C2'
        elif (index == 'C2'):
            index_prv_pos = 'C3'
        elif (index == 'C3'):
            index_prv_pos = 'C4'
        elif (index == 'C4'):
            index_prv_pos = 'C5'
        elif (index == 'C5'):
            index_prv_pos = 'C6'
        elif (index == 'C6'):
            index_prv_pos = 'C7'
        elif (index == 'C7'):
            index_prv_pos = 'C8'
        elif (index == 'C8'):
            index_prv_pos = 'C9'
        return index_prv_pos

    # 50% above water level is underranging so added the offset value and 50% lower value which deduct the value ,
    # if the chracterization behvation is like this else need to fine tune
    def waterlevel_range(self,range, df,pos):
        if pos == '99':
            return
        value = df.loc[pos, 'ExpectedRes']
        print('VALUE-->', value)
        if pos == 'C1':
            if (range >= df.loc[pos, 'ExpectedRes']):
                range = range - df.loc[ pos, 'OC_Val']  # Catergory Overranging, 15mm  average b/w C1&C2, Catergory Overranging
        elif pos == 'C2':
            if (range >= df.loc[pos, 'ExpectedRes']):
                range = range - df.loc[pos, 'OC_Val']  # Catergory Overranging, 30mm average b/w C3&C2
        elif pos == 'C3':
            print('C3--', range, df.loc[pos, 'ExpectedRes'])
            if (range >= df.loc[pos, 'ExpectedRes']):
                range = range - df.loc[pos, 'OC_Val']  # Catergory Overranging,25mm average b/w C4&C3
        elif pos == 'C4':
            print('C4--', range, df.loc[pos, 'ExpectedRes'])
            if (range >= df.loc[pos, 'ExpectedRes']):
                range = range - df.loc[pos, 'OC_Val']  # Catergory Overranging, 12mm average b/w C2&C2
        elif pos == 'C5':
            print('C5--', range, df.loc[pos, 'ExpectedRes'])
            if (range > df.loc[pos, 'ExpectedRes']):
                range = range - df.loc[pos, 'OC_Val']  # Catergory underranging,4mm average b/w C2&C2
        elif pos == 'C6':
            if (range < df.loc[pos, 'ExpectedRes']):
                range = range + df.loc[pos, 'OC_Val']  # 1mm average b/w C2&C2
        elif pos == 'C7':
            if (range < df.loc[pos, 'ExpectedRes']):
                range = range + df.loc[pos, 'OC_Val']  # 2mm average b/w C2&C2
        elif pos == 'C8':
            if (range < df.loc[pos, 'ExpectedRes']):
                range = range + df.loc[pos, 'OC_Val']  # 4mm average b/w C2&C2
        elif pos == 'C9':
            if (range < df.loc[pos, 'ExpectedRes']):
                range = range + df.loc[pos, 'OC_Val']  # 4mm average b/w C2&C2
        return range
class LiquidMeasPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._progress = 0.0
        self.range=0
        self.terbulence=0
    progressChanged = QtCore.pyqtSignal(float)
    @QtCore.pyqtProperty(float, notify=progressChanged)
    def progress(self):
        return self._progress

    @progress.setter
    def progress(self, p):
        if -3 <= p <= 3:
            self._progress = p
            self.progressChanged.emit(p)
            self.update()

    def drawLines(self, qp,height,steps):
        # steps =10
        x0 = self.geometry().width()
        y0 = self.geometry().height()
        # print('xy',x0,y0)
        qp.setPen(QtGui.QColor(("darkgrey")))
        qp.drawLine(x0 - 2, 0, x0 - 2, y0)
        qp.drawLine(2, 0, 2, y0)
        qp.setFont(QFont('Decorative', 8))
        ratio=(self.height()/self.totalheight)
        intv=((self.height()/steps))
        for lvl in range(steps+1):
            text=str(abs(int((intv*(lvl-steps)/ratio))))
            text1=str(abs(int(100/steps*(lvl-steps))))+'%' # This one need to be changed
            # print(intv, ratio, lvl, steps, text, text1)
            # print(x0 - 10, int((intv*lvl)), x0 + 50,int((intv*lvl)),intv,lvl)
            qp.drawLine(x0 - 15, int((intv*lvl)), x0 + 50,int((intv*lvl)))
            qp.drawText(x0 - 25, int((intv*lvl)),text)
            qp.drawLine(0, int((intv * lvl)), 12, int((intv * lvl)))
            qp.drawText(2, int((intv * lvl)), text1)
        steps += 1

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        height = self.progress * self.height()
        self.drawLines(painter, height,maxsteps)
        r = QtCore.QRect(25, int(self.height() - height), int(self.width()-50), int(height))
        painter.fillRect(r, QtGui.QBrush(QtCore.Qt.blue))
        pen = QtGui.QPen(QtGui.QColor("darkgray"), 6)
        painter.setPen(pen)
        # print("(self.terbulence ",self.terbulence )
        if (self.terbulence == 1):
            self.text2 = 'Detect Turbulence'
            painter.fillRect(r, QtGui.QBrush(QtCore.Qt.green))
            painter.setPen(QtGui.QColor(168, 34, 3))
            painter.setFont(QFont('Decorative', 20))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, self.text2)
            painter.drawRect(self.rect())
        # Liquid overlflow
        elif (self.progress >0 and (self.totalheight-self.range)>=self.totalheight ):
            self.text = 'Liquid Overflow'
            painter.setPen(QtGui.QColor(168, 34, 3))
            painter.setFont(QFont('Decorative', 20))
            painter.drawText(self.rect(),QtCore.Qt.AlignCenter, self.text)
        # Misplaced sensor position
        elif ( self.progress< 0 and (self.totalheight-self.range)<=0):
            self.text1 = 'Misplaced\nSensor Position'
            painter.setPen(QtGui.QColor(168, 34, 3))
            painter.setFont(QFont('Decorative', 20))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, self.text1)
        else:
            painter.drawRect(self.rect())
    def updatetankerheight(self,configheight):
        self.totalheight = configheight
    def sizeHint(self):
        return QtCore.QSize(600, 800)
    def clear(self):
        self.progress=0
        self.text2=""
        self.text1=""
        self.text=""

    def func_timer(self):
        if self.dataUpdate:
            self.dataUpdate = False
            self.update()
class Liquidmeasuredemo(QMainWindow):
    def __init__(self):
        super().__init__()
        self.isStart = False
        self.connected = False
        self.offset = False
        self.charcterizationfile = None
        self.btnclicked_count = 0
        self.btnclicked_count_t=0
        self.totalheight=210
        self.connected=False
        self.calibrationoffset=100
        self.Character_iter=1
        self.text="RANGING"
        self.send_msg=''
        self.recv_msg=''
        self.indicators=0
        self.NoSet=0
        self.coef =0.0
        self.intercept=0.0
        self.linearreg=0
        self.firmware=''
        self.OGalgo=0
        self.lev=['0','1','2','3','4','5','6','7','8','9']
        # self.deviation =[]
        # self.Rangcalc =[]
        # self.level =[10,20,30,40,50,60,70,80,90]
        self.Rangingresult=[]
        self.char_level = []
        self.character = []
        self.offalgo_pred=[]
        self.iter_dev_list=[]
        self.characterouter = {}
        self.characteAll={}
        # self.charc_iter={'C1': [], 'C2': [], 'C3': [], 'C4': [], 'C5': [], 'C6': [], 'C7': [],'C8': [], 'C9': []};
        # self.charc_iter={'C1': [], 'C2': [], 'C3': []}
        self.charc_iter={}
        self.distance=0
        self.range_dq=deque(maxlen=8) #Strore 5 values, measure upto 5sec check terbulence
        self.device = None
        self.turbulence_det = None
        self.max_r_l=0
        self.min_r_l=0
        self.turbu_det_std=10
        # self.logfile = open('characterization.txt', 'w')
        #Define Main window and layout
        self.read_config()
        self.configsteps()
        #Define Main window and layout
        self.initUI()
    def initUI(self):
        Liquid_W = QWidget(self)
        vbox = QVBoxLayout(self)
        Liquid_W.setLayout(vbox)
        self.setCentralWidget(Liquid_W)
        self.setWindowTitle("Liquid level monitoring Demo")
        self.setWindowIcon(QtGui.QIcon('st_logo_64x64.ico'))
        # self.resize(600, 600)
        self.setGeometry(40,40, 1000, 980)
        self.tank = LiquidMeasPlot()
        self.tank.updatetankerheight(self.totalheight)
        self.lcd = QLCDNumber()
        self.lcd.setFixedHeight(75)
        self.text='mm'
        self.rlabel = QLabel(self.text,self)

        # self.rlabel.setFixedHeight(75)
        # self.rlabel.setFixedHeight(75)
        # self.rlabel.setFixedWidth(50)

        self.qf_gridlay = QFrame(self)
        self.gridlay = QGridLayout(self)
        self.qf_gridlay.setLayout(self.gridlay)
        self.qf_gridlay.setFrameShape(QFrame.StyledPanel)

        self.gridlay.addWidget(self.lcd, 0, 1, 1, 2)
        self.gridlay.addWidget(self.rlabel, 0, 2, 1, 1,alignment=QtCore.Qt.AlignHCenter)
        my_font = QFont("Times New Roman", 22)
        self.rlabel.setFont(my_font)
        self.rlabel.setStyleSheet("font-weight: bold")
        self.rlabel.move(50, 30)
        self.lcd.move(100, 100)
        self.gridlay.addWidget(self.tank, 1, 1, 1, 2, alignment=QtCore.Qt.AlignTop)
        # self.tank.move(100, 100)
        self.label = QLabel("Liquid Container (Units display in mm)", self,alignment=QtCore.Qt.AlignTop)
        self.label.setFont(QFont("Times New Roman", 10))
        self.label.setStyleSheet("font-weight: bold")
        self.gridlay.addWidget(self.label, 2, 1, 1, 2, alignment=QtCore.Qt.AlignCenter)


        #All buttons
        self.fwbutton = QPushButton("FlashFW", self)
        # gridlay.addWidget(self.fwbutton, 3, 2, alignment=QtCore.Qt.AlignRight)
        self.calibration = QPushButton("Off+Xtalk", self)
        # gridlay.addWidget(self.calibration, 3, 3, alignment=QtCore.Qt.AlignLeft)
        self.startbutton = QPushButton("Start",self)
        # gridlay.verticalSpacing()
        self.characterization = QPushButton("Characterization", self)
        # self.Character=Characterization()
        # self.Character.graphvalinit()
        # =========================

        self.Control_box = QFrame(self)
        self.Control_box_layout = QGridLayout(self)
        self.Control_box.setLayout(self.Control_box_layout)
        self.Control_box.setFrameShape(QFrame.StyledPanel)

        self.Control_box_layout.addWidget(self.fwbutton, 0, 1)
        self.Control_box_layout.addWidget(self.calibration, 0, 2)
        self.Control_box_layout.addWidget(self.characterization, 0, 3)
        self.Control_box_layout.addWidget(self.startbutton, 0, 4)

        self.fwbutton.clicked.connect(self.fw_start_clicked)
        self.calibration.clicked.connect(self.btn_calibration_clicked)
        self.characterization.clicked.connect(self.btn_character_clicked)
        self.startbutton.clicked.connect(self.btn_start_clicked)
        self.char_chart=CharacterizationPlot(self,width=5, height=6, dpi=100)
        self.gridlay.addWidget(self.char_chart, 1, 3, 2, 2, alignment=QtCore.Qt.AlignTop)
        self.char_chart.udpate_char(self.Character_iter)
        vbox.addWidget(self.qf_gridlay, 300)
        vbox.addWidget(self.Control_box)
        self.show()
        self.device = commcmd.SerialCOM('STMicroelectronics STLink Virtual COM Port')
        if len(self.device.lComName):
            print(self.device.lComName)
        else:
            self.fwbutton.setDisabled(True)
            self.calibration.setDisabled(True)
            self.startbutton.setDisabled(True)

            self.characterization.setDisabled(True)
            QMessageBox.warning(self.window(), 'Warn',
                                'Please connect the board first!\nClose the window'
                                )
            # sys.exit()

    def configsteps(self):
        for cnt in range(self.indicators):
            if cnt ==0:
                continue
            val = 'C' + str(self.indicators-cnt)
            self.charc_iter[val] = []

    def read_config(self):
        global totalheight
        global maxsteps
        global OG_result
        filenames = os.listdir(os.getcwd())
        if "config.txt" in filenames:
            fp = open("config.txt", "r")
            while True:
                line = fp.readline()
                if not line: break
                if "firmware" in line:
                    self.firmware = str(line.split('=')[1])
                    self.firmware=self.firmware.strip()
                    # print("FW selection :", self.firmware)
                if "totalheight" in line:
                    self.totalheight = int(line.split('=')[1])
                    totalheight=self.totalheight
                    # print("totalheight of container :", self.totalheight)
                if "offsetcalibration" in line:
                    self.calibrationoffset = int(line.split('=')[1])
                    # print("calibration offset value  :", self.calibrationoffset)
                if "indicators" in line:
                    self.indicators = int(line.split('=')[1])
                    maxsteps = self.indicators
                    # print("configsteps  :", self.indicators)
                if "characterization" in line:
                    self.Character_iter = int(line.split('=')[1])
                    print("Characterization iteration :", self.Character_iter)
                if "coefficent" in line:
                    self.coef = float(line.split('=')[1])
                    # print("coefficent  :", self.coef)
                if "intercept" in line:
                    self.intercept = float(line.split('=')[1])
                    # print("intercept  :", self.intercept)
                if "linearregression" in line:
                    self.linearreg = int(line.split('=')[1])
                    # print("linearregression  :", self.linearreg)
                if "OCalgorithm" in line:
                    self.OGalgo = int(line.split('=')[1])
                    # print("OCalgorithm  :", self.OGalgo)
                if "turbu_det_std" in line:
                    self.turbu_det_std = int(line.split('=')[1])
                    # print("turbu_det_std  :", self.OGalgo)
            fp.close()
        else:
            print('config file doesnt exist')
            self.totalheight=210 #HW coded
        
    #if again characterization, then delete coefficent and intercept to rewrite the new value otherwise it will append. Need to check overwrite
    def write_config(self):
        filenames = os.listdir(os.getcwd())
        if "config.txt" in filenames:
            fp = open("config.txt", "a+")
            # fp_w = open("config.txt", "w")
            fp.seek(0)
            while True:
                line = fp.readline()
                if not line: break
                if "characterization" in line:
                    if self.coef and self.intercept:
                        # print("coefficent  :", str(self.coef))
                        fp.write( "coefficent=" + str(self.coef))
                        fp.write("\nintercept=" + str(self.intercept))
                        fp.write("\nlinearregression=" + str(0))
                        break
                else:
                    print('not exist')
            fp.close()
        else:
            print('config file doesnt exist')
    def closeEvent(self, *args, **kwargs):
        # if self.logfile is not None:
        #     self.logfile.close()
        self.device.__del__()
        event = args[0]
        event.accept()
    def fw_start_clicked(self):
        global fw_l4
        if len(self.device.lComName):
            ret = checkRemovableDrives(self)
            for key, values in ret.items():
                val = values.split(' ')[0]
                if (val == 'NODE_F401RE'):
                    break
            try:
                self.tank.clear()
                self.lcd.display(0)
                self.char_chart.clear()
                self.character.clear()
                # self.logfile.close()
                print('self.firmware',self.firmware)
                if self.firmware == 'VL53L4CD':
                    shutil.copy(fw_l4, key)
                    QMessageBox.information(self.window(), 'infomration',
                                            'Flash VL53L4CD FW Successfully\n''Click Offset or Characterization', QMessageBox.Yes)
                else:
                    QMessageBox.warning(self.window(), 'warning',
                                            'FW config is failed\n', QMessageBox.Yes)
                self.calibration.setEnabled(True)
                self.characterization.setEnabled(True)
                self.btnclicked_count = 0
                self.characterization.setText('Characterization')
                self.isStart = False
                self.connected = False
                time.sleep(2)
            except EnvironmentError:
                print("Error happened")
                QMessageBox.information(self.window(), 'error',
                                        'Fw load failed\n', QMessageBox.Yes)
        else:
            QMessageBox.information(self.window(), 'information',
                                    'Can not connect to the device!\n'
                                    'Maybe it is used by other device.', QMessageBox.Yes)

    def btn_calibration_clicked(self):
        if self.connected and self.offset:
            self.tank.clear()
            self.lcd.display(0)
            self.device.disconnect()
            self.offset = False
            self.connected = False
            self.calibration.setText('Off+Xtalk')
        else:
            self.tank.clear()
            self.connected = self.device.connect(self.device.lComName[0], 460800, msg_handler)
            if self.connected:
                self.connected = True
                self.offset = True
                offsetvalue = 'O-' + str(self.calibrationoffset)
                msgsend = self.device.send_command(offsetvalue)
                print('btn_calibration_clicked==>', msgsend, offsetvalue)
                self.calibration.setDisabled(True)
            else:
                QMessageBox.information(self.window(), 'information',
                                        'Can not connect to the device!\n'
                                        'Maybe it is used by others.', QMessageBox.Yes)
        # Charcterization with 10 levels

    def btn_character_clicked(self):
        global maxsteps
        if self.connected:
            self.connected = True
        else:
            self.connected = self.device.connect(self.device.lComName[0], 460800, msg_handler)
            if self.connected:
                self.connected = True
            else:
                QMessageBox.information(self.window(), 'information',
                                        'Can not connect to the device!\n'
                                        'Maybe it is used by others.', QMessageBox.Yes)
        if self.connected:
            if(self.send_msg == self.recv_msg):
                self.btnclicked_count+=1
                self.btnclicked_count_t+=1
            if self.btnclicked_count >= self.indicators:
                self.NoSet += 1
                # self.characterization.setDisabled(True)
                self.character.append(self.charc_iter)

                for set_iter in range(self.NoSet):
                    self.characterouter = dict(zip(self.lev[set_iter], self.character))
                    for key, value in (self.characterouter[self.lev[set_iter]].items()):
                        self.Rangingresult.append(value[0])
                    self.char_chart.update_figure(self.Rangingresult)
                    self.characteAll[self.Character_iter] = self.Rangingresult.copy()
                    self.Rangingresult.clear()
                for key in self.charc_iter.keys():
                    self.charc_iter[key] = []
                self.btnclicked_count=0
                self.Character_iter-=1

                if (self.Character_iter == 0):
                    self.coef,self.intercept=self.char_chart.regression_plot(self.characteAll)
                    self.coef=round(self.coef[0],1)
                    self.intercept=round(self.intercept,1)
                    self.logfile = open('characterization.txt', 'w')
                    if self.logfile is not None:
                        self.logfile.write(str(self.characteAll))
                        self.logfile.close()
                    self.characterization.setDisabled(True)

            else:
                clickdis = 'level-' + str(self.btnclicked_count)
                self.characterization.setText(clickdis)
                if self.btnclicked_count == 1:
                    value = self.totalheight - (self.totalheight * 0.1)
                elif self.btnclicked_count ==2:
                    value=self.totalheight -(self.totalheight*0.2)
                elif self.btnclicked_count ==3:
                    value=self.totalheight -(self.totalheight*0.3)

                elif self.btnclicked_count ==4:
                    value=self.totalheight -(self.totalheight*0.4)
          
                elif self.btnclicked_count ==5:
                    value=self.totalheight -(self.totalheight*0.5)

                elif self.btnclicked_count ==6:
                    value=self.totalheight -(self.totalheight*0.6)

                elif self.btnclicked_count ==7:
                    value=self.totalheight -(self.totalheight*0.7)

                elif self.btnclicked_count ==8:
                    value=self.totalheight -(self.totalheight*0.8)

                elif self.btnclicked_count ==9:
                    value=self.totalheight -(self.totalheight*0.9)

                if self.btnclicked_count >0:
                    charvalue = 'C' + str(self.btnclicked_count)+'-'+str(int(value))
                    self.send_msg ='C' + str(self.btnclicked_count)
                    msgsend = self.device.send_command(charvalue)
                    time.sleep(3)

    def btn_start_clicked(self):
        if self.connected and self.isStart:
            self.tank.clear()
            self.lcd.display(0)
            self.device.disconnect()
            self.isStart = False
            self.connected = False
            self.startbutton.setText('Start')
        elif self.connected and ~self.isStart:
             value='R-'+ str(self.totalheight)
             msgsend = self.device.send_command(value)
             self.isStart = True
             self.connected = True
             self.startbutton.setText('Stop')
        else:
            self.tank.clear()
            self.connected = self.device.connect( self.device.lComName[0],460800, msg_handler)
            if self.connected:
                self.isStart = True
                self.connected = True
                self.startbutton.setText('Stop')
                # self.charcterizationfile = open('slam_log.txt', 'w')
                value = 'R-' + str(self.totalheight)
                msgsend = self.device.send_command(value)
            else:
                QMessageBox.information(self.window(), 'information',
                                        'Can not connect to the device!\n'
                                        'Maybe it is used by others.', QMessageBox.Yes)

    def RangingValue_changed(self):
        progress = ((self.totalheight-self.distance) * 1.0) / self.totalheight

        self.tank.progress = progress
        self.tank.range =self.distance
        self.tank.terbulence=self.turbulence_det
        value=(self.totalheight-self.distance)
        self.lcd.display(value)

    def update_plot(self, obj):
        if '\n' in obj.msg:
            obj.msg=obj.msg.strip()
        if '=' in obj.msg:
            result=obj.msg.split('=')
            if result[0]!='':
                if ',' in result[1]:
                    #This lag is used when charcterization and deviation value
                    self.recv_msg=result[0]
                    deviation = result[1].split(',')
                    if (len(deviation[0]) < 5 and deviation[0].isalnum()):
                        self.distance = (float(deviation[0]))
                        self.deviation= (float(deviation[1]))
                        #Sometimes missing first character then its broken
                        if result[0][0] != 'C':
                            result[0]='C'+result[0]
                        if (len(result[0]) == 2):
                            self.charc_iter[result[0]].append(self.distance)
                            # self.character['level1'].append(self.deviation)
                            self.RangingValue_changed()
                else:
                    if (len(result[1])<5 and result[1].isalnum()):
                        if (  self.isStart and self.coef and self.intercept and self.linearreg ==1):
                            self.distance=self.coef*(float(result[1]))+self.intercept
                        elif(self.isStart and self.OGalgo ==1):
                            self.distance=self.OGalogrithm_range(float(result[1]))
                        else:
                            self.distance=(float(result[1]))

                        self.distance=round(self.distance,0)
                        print('distance-->',self.distance)
                        #turbulence detect
                        self.range_dq.append(self.distance)
                        # During terbulence , the ranging value is increased thats why compare wtih max value is make sense
                        if ((len(list(self.range_dq))) == 8): #Every 5sec it checks
                       
                            if (stdev(list(self.range_dq)) > self.turbu_det_std and (max(list(self.range_dq)) > self.max_r_l)):  # All datas come under 3STD
                                self.turbulence_det=1
                            elif (stdev(list(self.range_dq)) <= 4): #Still water condition
                                self.turbulence_det=0
                                self.max_r_l = max(list(self.range_dq)) + stdev(list(self.range_dq))
                                self.min_r_l = min(list(self.range_dq)) #- stdev(list(self.range_dq))

                        self.RangingValue_changed()

    def findprevindex(self,index):
        index_prv_pos = '99'
        if (index == 'C1'):
            index_prv_pos = 'C2'
        elif (index == 'C2'):
            index_prv_pos = 'C3'
        elif (index == 'C3'):
            index_prv_pos = 'C4'
        elif (index == 'C4'):
            index_prv_pos = 'C5'
        elif (index == 'C5'):
            index_prv_pos = 'C6'
        elif (index == 'C6'):
            index_prv_pos = 'C7'
        elif (index == 'C7'):
            index_prv_pos = 'C8'
        elif (index == 'C8'):
            index_prv_pos = 'C9'
        elif (index == 'C9'):
            index_prv_pos = 'C9'
        return index_prv_pos

    def waterlevel_range(self,range,OG_df,pos):
        if pos == 'C1':
            if (range >= OG_df.loc[pos, 'ExpectedRes']):
                range = range - OG_df.loc[pos, 'OC_Val']  # Category Overranging, 15mm  average b/w C1&C2, Catergory Overranging
        elif pos == 'C2':
            if (range >= OG_df.loc[pos, 'ExpectedRes']):
                range = range - OG_df.loc[pos, 'OC_Val']  # Category Overranging, 30mm average b/w C3&C2
        elif pos == 'C3':
            # print('C3--', range, OG_df.loc[pos, 'ExpectedRes'])
            if (range >= OG_df.loc[pos, 'ExpectedRes']):
                range = range - OG_df.loc[pos, 'OC_Val']  # Category Overranging,25mm average b/w C4&C3
        elif pos == 'C4':
            # print('C4--', range, OG_df.loc[pos, 'ExpectedRes'])
            if (range >= OG_df.loc[pos, 'ExpectedRes']):
                range = range - OG_df.loc[pos, 'OC_Val']  # Category Overranging, 12mm average b/w C2&C2
        elif pos == 'C5':
            # print('C5--', range, OG_df.loc[pos, 'ExpectedRes'])
            if (range > OG_df.loc[pos, 'ExpectedRes']):
                range = range - OG_df.loc[pos, 'OC_Val']  # Category underranging,4mm average b/w C2&C2
        elif pos == 'C6':
            if (range < OG_df.loc[pos, 'ExpectedRes']):
                range = range + OG_df.loc[pos, 'OC_Val']  # 1mm average b/w C2&C2
        elif pos == 'C7':
            if (range < OG_df.loc[pos, 'ExpectedRes']):
                range = range + OG_df.loc[pos, 'OC_Val']  # 2mm average b/w C2&C2
        elif pos == 'C8':
            if (range < OG_df.loc[pos, 'ExpectedRes']):
                range = range + OG_df.loc[pos, 'OC_Val']  # 4mm average b/w C2&C2
        elif pos == 'C9':
            if (range < OG_df.loc[pos, 'ExpectedRes']):
                range = range + OG_df.loc[pos, 'OC_Val']  # 4mm average b/w C2&C2
        return range
    def OGalogrithm_range(self,range):
        global OG_result
        OGrange=0
        value=0
        prev_idx=''
        pos=99
        #Find the position of water level
        for index, val in OG_result.iterrows():
            if (range < self.totalheight +5):
                if (range < OG_result.loc[index, 'ExpectedRes']):
                    prev_idx=self.findprevindex(index)
                    # if value is less than 50% of index then consider the lower one
                    #Check range value < 1st value or not
                    if index!=OG_result.index[0]:
                        value = (OG_result.loc[index, 'ExpectedRes'] +(OG_result.loc[prev_idx, 'ExpectedRes']))/2
                    else:
                        value = (OG_result.loc[index, 'ExpectedRes'])
                    if (range  <= value):
                        pos = prev_idx
                        break;
                    else:
                        pos = index
                        break;
                else:
                    #Keep buffer 5mm somtimes max value is showing
                    if (range >= self.totalheight+5):
                        pos = '99'
                    else:
                        pos=OG_result['OC_Val'].idxmax()
            else:
                pos='99'
        if pos != '99':
            OGrange=self.waterlevel_range(range,OG_result,pos)
        return OGrange
def checkRemovableDrives(self):
    drives = {}

    bitmask = ctypes.windll.kernel32.GetLogicalDrives()
    # Check possible drive letters, from A to Z
    # Note: using ascii_uppercase because we do not want this to change with locale!
    for letter in string.ascii_uppercase:
        drive = "{0}:\\".format(letter)
        if bitmask & 1 and ctypes.windll.kernel32.GetDriveTypeA(drive.encode("ascii")) == DRIVE_REMOVABLE:
            volume_name = ""
            name_buffer = ctypes.create_unicode_buffer(1024)
            filesystem_buffer = ctypes.create_unicode_buffer(1024)
            error = ctypes.windll.kernel32.GetVolumeInformationW(ctypes.c_wchar_p(drive), name_buffer,
                                                                 ctypes.sizeof(name_buffer), None, None, None,
                                                                 filesystem_buffer, ctypes.sizeof(filesystem_buffer))

            if error != 0:
                volume_name = name_buffer.value

            if not volume_name:
                volume_name = print("@item:intext", "Removable Drive")
            if filesystem_buffer.value == "":
                continue

            # Check for the free space. Some card readers show up as a drive with 0 space free when there is no card inserted.
            free_bytes = ctypes.c_longlong(0)
            if ctypes.windll.kernel32.GetDiskFreeSpaceExA(drive.encode("ascii"), ctypes.byref(free_bytes), None,
                                                          None) == 0:
                continue

            if free_bytes.value < 1:
                continue

            drives[drive] = "{0} ({1}:)".format(volume_name, letter)
        bitmask >>= 1

    return drives

def msg_handler(obj):
    lqm.update_plot(obj)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    qp = QPalette()
    qp.setColor(QPalette.Window, QtGui.QColor(255,255,255))
    app.setPalette(qp)
    lqm = Liquidmeasuredemo()
    sys.exit(app.exec_())

