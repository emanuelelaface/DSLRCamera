'''
Camera-M10 -- Astrophotography with Canon M10
Copyright (c) 2018, Emanuele Laface (Emanuele.Laface@gmail.com)

All rights reserved.

Redistribution and use, with or without modification, are permitted provided that the following conditions are met:
Redistributions must retain the above copyright notice, this list of conditions and the following disclaimer.
Neither the name of the AstroPhoto Author nor the names of any contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import sys
import numpy
import chdkptp
import cv2
import time
import threading
import rawpy
import gpio
import imageio
import os
import subprocess
import astropy.io.fits
import astropy.wcs
import socket
from PyQt4 import QtGui, QtCore

class EOSUI(QtGui.QWidget):
    def __init__(self):
        super(EOSUI, self).__init__()
        self.initUI()
        self.initConn()

    def initUI(self):
        self.iso = QtGui.QComboBox(self)
        self.iso.setEnabled(False)
        for i in ['Auto','100','125','160','200','250','320','400',
                '500','640','800','1000','1250','1600','2000','2500',
                '3200','4000','5000','6400','8000','10000','12800']:
            self.iso.addItem(i)
        self.shutter = QtGui.QLineEdit(self)
        self.shutter.setAlignment(QtCore.Qt.AlignCenter)
        self.shutter.setEnabled(False)
        self.pics = QtGui.QLineEdit(self)
        self.pics.setAlignment(QtCore.Qt.AlignCenter)
        self.pics.setEnabled(False)
        self.shoot = QtGui.QPushButton('Shoot', self)
        self.shoot.setEnabled(False)
        self.video = QtGui.QPushButton('Video', self)
        self.video.setEnabled(False)
        self.is_filming=False
        self.stop = QtGui.QPushButton('Stop', self)
        self.stop.setEnabled(False)
        self.live = QtGui.QCheckBox('Live View', self)
        self.live.setEnabled(False)
        self.zoom = QtGui.QComboBox(self)
        for i in ['1', '5', '10']:
            self.zoom.addItem(i)
        self.zoom.setEnabled(False)
        self.reticle = QtGui.QCheckBox('Reticle', self)
        self.reticle.setEnabled(False)
        self.image_label = QtGui.QLabel(self)
        self.rgbImage = (numpy.ones((480, 720,3))*155).astype(numpy.uint8)
        self.text_line = QtGui.QLabel("", self)
        self.connect = QtGui.QPushButton('Connect Camera', self)
        self.focus_in = QtGui.QPushButton('In', self)
        self.focus_in.setEnabled(False)
        self.focus_out = QtGui.QPushButton('Out', self)
        self.focus_out.setEnabled(False)
        self.focus_auto = QtGui.QPushButton('Auto', self)
        self.focus_auto.setEnabled(False)
        self.solve_scale = QtGui.QComboBox(self)
        for i in ['4.394','1.970', '0.255']:
            self.solve_scale.addItem(i)
        self.solve = QtGui.QPushButton('Solve', self)
        self.mount_ip = QtGui.QLineEdit(self)
        self.mount_ip.setAlignment(QtCore.Qt.AlignCenter)
        self.mount_ip.setText('192.168.212.66')
        self.mount_ip.setEnabled(False)
        self.mount = QtGui.QPushButton('Update Mount', self)
        self.mount.setEnabled(False)

        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        grid.addWidget(QtGui.QLabel('ISO'), 0,3)
        grid.addWidget(QtGui.QLabel('Shutter'), 0,4)
        grid.addWidget(QtGui.QLabel('Pics'), 0,5)
        grid.addWidget(QtGui.QLabel('Zoom'), 0,7)
        grid.addWidget(self.shoot, 1,0)
        grid.addWidget(self.video, 1,1)
        grid.addWidget(self.stop, 1,2)
        grid.addWidget(self.iso, 1,3)
        grid.addWidget(self.shutter, 1,4)
        grid.addWidget(self.pics, 1,5)
        grid.addWidget(self.live, 1,6)
        grid.addWidget(self.zoom, 1,7)
        grid.addWidget(self.reticle, 1,8)
        grid.addWidget(self.image_label, 2, 0, 2, 9)
        grid.addWidget(self.text_line, 4, 0, 1, 9)

        side_grid = QtGui.QGridLayout()
        side_grid.setSpacing(5)
        focuser_label=QtGui.QLabel('Focuser')
        side_grid.addWidget(focuser_label, 0,0,1,2)
        side_grid.setAlignment(focuser_label, QtCore.Qt.AlignCenter)
        hline1 = QtGui.QFrame()
        hline1.setFrameShape(QtGui.QFrame.HLine)
        hline1.setFrameShadow(QtGui.QFrame.Sunken)
        hline2 = QtGui.QFrame()
        hline2.setFrameShape(QtGui.QFrame.HLine)
        hline2.setFrameShadow(QtGui.QFrame.Sunken)
        side_grid.addWidget(hline1, 1,0,1,2)
        side_grid.addWidget(self.focus_in, 2,0)
        side_grid.addWidget(self.focus_out, 2,1)
        side_grid.addWidget(self.focus_auto, 3,0,1,2)
        side_grid.addWidget(hline2, 4,0,1,2)
        side_grid.addItem(QtGui.QSpacerItem(0, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum), 5,0,1,2)
        solver_label=QtGui.QLabel('Solver')
        side_grid.addWidget(solver_label, 6,0,1,2)
        side_grid.setAlignment(solver_label, QtCore.Qt.AlignCenter)
        hline3 = QtGui.QFrame()
        hline3.setFrameShape(QtGui.QFrame.HLine)
        hline3.setFrameShadow(QtGui.QFrame.Sunken)
        hline4 = QtGui.QFrame()
        hline4.setFrameShape(QtGui.QFrame.HLine)
        hline4.setFrameShadow(QtGui.QFrame.Sunken)
        side_grid.addWidget(hline3, 7,0,1,2)
        side_grid.addWidget(QtGui.QLabel('Scale:'),8,0)
        side_grid.addWidget(self.solve_scale, 8,1)
        side_grid.addWidget(self.solve, 9,0,1,2)
        self.ra_label=QtGui.QLabel('RA: --')
        self.dec_label=QtGui.QLabel('Dec: --')
        side_grid.addWidget(self.ra_label, 10,0,1,2)
        side_grid.addWidget(self.dec_label, 11,0,1,2)
        side_grid.addWidget(self.mount_ip, 12,0,1,2)
        side_grid.addWidget(self.mount, 13,0,1,2)
        side_grid.addWidget(hline4, 14,0,1,2)

        grid.addLayout(side_grid,2,9)
        grid.addWidget(self.connect,3,9)
        grid.setAlignment(side_grid, QtCore.Qt.AlignTop)
        self.setLayout(grid)
        self.setGeometry(50, 50, -1 ,-1)
        self.setWindowTitle('Camera Not Present')

        self.is_recording=False
        self.is_processing=False
        self.contour=False
        self.timer=0

        self.stop_event = threading.Event()

        self.show()

    def initConn(self):
        self.connect.clicked[bool].connect(self.initCamera)
        self.iso.activated[int].connect(self.setIso)
        self.live.stateChanged[int].connect(self.toggleLive)
        self.shoot.clicked[bool].connect(self.shootFunction)
        self.stop.clicked[bool].connect(self.stopAction)
        self.focus_in.clicked.connect(self.focusIn)
        self.focus_out.clicked.connect(self.focusOut)
        self.focus_auto.clicked.connect(self.focusAuto)
        self.solve.clicked.connect(self.solveImage)
        self.mount.clicked.connect(self.sendToMount)
        self.video.clicked.connect(self.recordVideo)

    def paintEvent(self, e):
        self.myImage = QtGui.QImage(self.rgbImage, 720, 480, QtGui.QImage.Format_RGB888)
        if self.live.checkState() != 0:
            vp, bm = get_live_view(self.camera)
            if self.reticle.checkState() == QtCore.Qt.Checked:
                cv2.line(bm, (720/2,0),(720/2,480),(255,255,255),1)
                cv2.line(bm, (0,480/2),(720,480/2),(255,255,255),1)
                for i in range(30, 500, 100):
                    cv2.circle(bm, (720/2,480/2), i, (255,255,255), 1)
            if self.contour:
                img_gray = cv2.cvtColor(vp, cv2.COLOR_BGR2GRAY)
                ret, thresh = cv2.threshold(img_gray, 120 , 255, cv2.THRESH_BINARY)
                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(bm, contours, -1, (255,255,255), 1)
                self.star_area = 0
                for i in contours:
                    self.star_area = self.star_area+cv2.contourArea(i)
                    self.star_area = self.star_area*cv2.Laplacian(img_gray, cv2.CV_64F).var()

            self.myImage=QtGui.QImage(numpy.clip(vp.astype(numpy.uint16)+
                    bm.astype(numpy.uint16),0,255).astype(numpy.uint8), 720, 480, QtGui.QImage.Format_RGB888)

        if self.is_recording:
            self.text_line.setText('Shooting picture '+str(self.photo_counter+1)+' of '+self.pics.text()+'. Time elapsed '+str(int(time.time())-self.timer)+' of '+self.shutter.text())

        if self.is_processing:
            self.text_line.setText('Processing...')

        self.image_label.setPixmap(QtGui.QPixmap.fromImage(self.myImage))

    def initCamera(self, button):
        try:
            device=chdkptp.list_devices()
            self.camera=chdkptp.ChdkDevice(device[0])
        except:
            self.text_line.setText('Error: camera not connected')
            return

        self.camera.switch_mode('record')
        self.camera.lua_execute('set_backlight(0)')
        self.setWindowTitle(self.camera.info.model_name)
        purge_files(self.camera)
        self.text_line.setText('Camera connected')
        self.connect.setEnabled(False)

        self.iso.setEnabled(True)
        self.shutter.setEnabled(True)
        self.pics.setEnabled(True)
        self.shoot.setEnabled(True)
        self.video.setEnabled(True)
        self.live.setEnabled(True)

        self.iso.setCurrentIndex(get_iso(self.camera))
        self.shutter.setText(str(get_camera_shutter_time(self.camera)))
        self.pics.setText('1')

        self.initFocuser()

    def initFocuser(self):
        self.focuserRUN = 80
        self.focuserIN = 73
        self.focuserOUT = 69
        gpio.setup(self.focuserRUN, gpio.OUT)
        gpio.setup(self.focuserIN, gpio.OUT)
        gpio.setup(self.focuserOUT, gpio.OUT)

    def toggleLive(self,value):
        if value!=0:
            self.reticle.setEnabled(True)
            self.focus_in.setEnabled(True)
            self.focus_out.setEnabled(True)
            self.focus_auto.setEnabled(True)
        else:
            self.reticle.setEnabled(False)
            self.focus_in.setEnabled(False)
            self.focus_out.setEnabled(False)
            self.focus_auto.setEnabled(False)

    def setIso(self,iso):
        self.iso.setCurrentIndex(set_iso(self.camera,iso))
        self.text_line.setText('New iso set')

    def stopAction(self):
        self.text_line.setText('Aborting...')
        self.stop_event.set()

    def shootFunction(self,value):
        self.shoot.setEnabled(False)
        self.video.setEnabled(False)
        self.stop.setEnabled(True)
        self.live.setCheckState(0)
        tr = threading.Thread(target=self.shootPic, args=(self.stop_event,))
        tr.start()

    def shootPic(self, stop_event):
        self.photo_counter = 0
        while self.photo_counter < int(self.pics.text()) and not stop_event.isSet():
            self.is_recording=True
            self.timer=int(time.time())
            shutter_time=str(int(numpy.rint(float(self.shutter.text())*1000000)))
            self.camera.lua_execute('set_tv96_direct(usec_to_tv96('+shutter_time+'))')
            self.camera.lua_execute("""press('shoot_half')
                                  repeat
                                      sleep(10)
                                  until get_shooting()
                                  press("shoot_full")
                                  return""")
            time.sleep(float(self.shutter.text()))
            self.is_recording=False
            self.is_processing=True
            while len(list_files(self.camera)) == 0:
                time.sleep(1)
            for i in list_files(self.camera):
                self.localfile=i.split('/')[3]
                self.camera.download_file(i,self.localfile)
            raw=rawpy.imread(self.localfile)
            self.rgbImage=cv2.resize(raw.postprocess(half_size=True), (720, 480))
            raw.close()
            self.is_processing=False
            purge_files(self.camera)
            self.photo_counter+=1

        stop_event.clear()
        self.shoot.setEnabled(True)
        self.video.setEnabled(True)
        self.stop.setEnabled(False)
        self.text_line.setText('Done')

    def recordVideo(self):
        if self.is_filming:
            self.camera.lua_execute('press("video")')
            self.video.setStyleSheet("background-color: None")
            while len(list_files(self.camera)) == 0:
                time.sleep(1)
            for i in list_files(self.camera):
                self.localfile=i.split('/')[3]
                self.camera.download_file(i,self.localfile)
            purge_files(self.camera)
            self.shoot.setEnabled(True)
            self.stop.setEnabled(True)
            self.live.setEnabled(True)
            self.is_filming=False
            self.text_line.setText('Done')
        else:
            self.live.setCheckState(0)
            self.live.setEnabled(False)
            self.shoot.setEnabled(False)
            self.stop.setEnabled(False)
            self.text_line.setText('Recording video')
            self.video.setStyleSheet("background-color: red")
            self.camera.lua_execute('press("video")')
            self.is_filming=True


    def focusIn(self):
        if gpio.read(self.focuserRUN) == 0:
            gpio.set(self.focuserRUN, 1)
            gpio.set(self.focuserIN, 1)
            gpio.set(self.focuserOUT, 0)
            self.focus_in.setStyleSheet("background-color: lime")
            self.focus_out.setStyleSheet("background-color: None")
            self.focus_auto.setEnabled(False)
        else:
            if gpio.read(self.focuserIN) == 1:
                gpio.set(self.focuserRUN, 0)
                gpio.set(self.focuserIN, 0)
                gpio.set(self.focuserOUT, 0)
                self.focus_in.setStyleSheet("background-color: None")
                self.focus_out.setStyleSheet("background-color: None")
                self.focus_auto.setEnabled(True)
            else:
                gpio.set(self.focuserOUT, 0)
                gpio.set(self.focuserIN, 1)
                self.focus_in.setStyleSheet("background-color: lime")
                self.focus_out.setStyleSheet("background-color: None")

    def focusOut(self):
        if gpio.read(self.focuserRUN) == 0:
            gpio.set(self.focuserRUN, 1)
            gpio.set(self.focuserIN, 0)
            gpio.set(self.focuserOUT, 1)
            self.focus_out.setStyleSheet("background-color: lime")
            self.focus_in.setStyleSheet("background-color: None")
            self.focus_auto.setEnabled(False)
        else:
            if gpio.read(self.focuserOUT) == 1:
                gpio.set(self.focuserRUN, 0)
                gpio.set(self.focuserIN, 0)
                gpio.set(self.focuserOUT, 0)
                self.focus_in.setStyleSheet("background-color: None")
                self.focus_out.setStyleSheet("background-color: None")
                self.focus_auto.setEnabled(True)
            else:
                gpio.set(self.focuserIN, 0)
                gpio.set(self.focuserOUT, 1)
                self.focus_out.setStyleSheet("background-color: lime")
                self.focus_in.setStyleSheet("background-color: None")

    def focusAuto(self):
        self.contour=True
        self.focus_in.setEnabled(False)
        self.focus_out.setEnabled(False)
	self.focus_auto.setEnabled(False)
        gpio.set(self.focuserRUN, 0)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 0)
        tr = threading.Thread(target=self.autoFocus)
        tr.start()

    def autoFocus(self):
	self.text_line.setText('Rewind for 10 seconds')
        gpio.set(self.focuserRUN, 1)
        gpio.set(self.focuserIN, 1)
        gpio.set(self.focuserOUT, 0)
        time.sleep(10)
        gpio.set(self.focuserRUN, 0)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 0)
        scan_area = []
        time_area = []
        start_time=time.time()
        gpio.set(self.focuserRUN, 1)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 1)
	self.text_line.setText('Scan for focal point for 20 seconds')
        for i in range(200):
                time.sleep(0.1)
                scan_area.append(self.star_area)
                time_area.append(time.time()-start_time)
        gpio.set(self.focuserRUN, 0)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 0)

        scan_area = numpy.array(scan_area)
        time_area = numpy.array(time_area)
        filtered_scan_area = scan_area[10*scan_area-(10*scan_area).astype(int)!=0]
        filtered_time_area = time_area[10*scan_area-(10*scan_area).astype(int)!=0]
        a, b, c = numpy.polyfit(filtered_time_area, filtered_scan_area, 2)
        focal_time = (time_area[199]+b/(2*a))
	if focal_time > 12 or focal_time < 0:
	    self.text_line.setText('Focal poiny not found')
	    return

	self.text_line.setText('Rewind for '+str(focal_time)+' seconds')
        gpio.set(self.focuserRUN, 1)
        gpio.set(self.focuserIN, 1)
        gpio.set(self.focuserOUT, 0)
        time.sleep(focal_time+2)
        gpio.set(self.focuserRUN, 0)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 0)

        scan_area = []
        time_area = []
        start_time=time.time()
	self.text_line.setText('Fine scan for 4 seconds')
        gpio.set(self.focuserRUN, 1)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 1)
        for i in range(100):
                time.sleep(0.04)
                scan_area.append(self.star_area)
                time_area.append(time.time()-start_time)
        gpio.set(self.focuserRUN, 0)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 0)

        scan_area = numpy.array(scan_area)
        time_area = numpy.array(time_area)
        filtered_scan_area = scan_area[10*scan_area-(10*scan_area).astype(int)!=0]
        filtered_time_area = time_area[10*scan_area-(10*scan_area).astype(int)!=0]
        a, b, c = numpy.polyfit(filtered_time_area, filtered_scan_area, 2)
        focal_time = (time_area[99]+b/(2*a))

	self.text_line.setText('Set to the focal point')
        gpio.set(self.focuserRUN, 1)
        gpio.set(self.focuserIN, 1)
        gpio.set(self.focuserOUT, 0)
        time.sleep(focal_time)
        gpio.set(self.focuserRUN, 0)
        gpio.set(self.focuserIN, 0)
        gpio.set(self.focuserOUT, 0)

	self.focus_auto.setEnabled(True)
        self.focus_in.setEnabled(True)
        self.focus_out.setEnabled(True)
	self.text_line.setText('Done')

    def solveImage(self):
        self.solve.setEnabled(False)
        filename = str(QtGui.QFileDialog.getOpenFileName(self, 'Open file', '', 'Supported Media (*.CR2 *.avi)'))
        tr = threading.Thread(target=self.solver, args=(filename,))
        tr.start()

    def solver(self, filename):
        name, extension = os.path.splitext(filename)
        if extension != '.CR2' and extension != '.avi':
            self.solve.setEnabled(True)
            return

        self.text_line.setText('Converting file into Tiff')
        if extension == '.CR2':
            raw=rawpy.imread(filename)
            image=raw.postprocess(half_size=True)
            raw.close()
            imageio.imsave(name+'.tif', image)
        if extension == '.avi':
            subprocess.call(["/usr/bin/avconv",  "-i", filename, "-vframes", "1", name+".jpeg"])
            image = imageio.imread(name+".jpeg")
            imageio.imsave(name+'.tif', image)
            os.remove(name+'.jpeg')

        center_x = image.shape[1]/2
        center_y = image.shape[0]/2

        self.text_line.setText('Solving image...')
        scale = float(self.solve_scale.currentText())
        subprocess.call(["/usr/bin/solve-field", "--downsample", "2", "--tweak-order", "2", "--no-plots", "--overwrite", name+".tif"])

        if not os.path.isfile(name+'.solved'):
            self.text_line.setText('Image not solved')
            clean_astrometry(name)
            self.solve.setEnabled(True)
            return

        wcs = astropy.wcs.WCS(astropy.io.fits.open(name+'.new')[0].header)
        RA, Dec = wcs.wcs_pix2world(center_x, center_y, 1)
        self.coords=[RA_to_text(RA), Dec_to_text(Dec)]

        clean_astrometry(name)
        self.solve.setEnabled(True)
        self.mount_ip.setEnabled(True)
        self.mount.setEnabled(True)
        self.ra_label.setText('Ra: '+self.coords[0])
        self.dec_label.setText('Dec: '+self.coords[1])
        self.text_line.setText('Done')

    def sendToMount(self):
        address = str(self.mount_ip.text())
        port = 4030
        bufsize = 1024
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((address, port))
            s.send(":Sr"+self.coords[0]+"#")
            data = s.recv(bufsize)
            s.send(":Sd"+self.coords[1]+"#")
            data = s.recv(bufsize)
            s.send(":CM#")
            data = s.recv(bufsize)
            s.close()
        except:
            self.text_line.setText('Connection to mount failed')

def RA_to_text(ra):
    ra_in_hours = ra/360.0*24.0
    hours = int(ra_in_hours // 1)
    minutes_decimal = (ra_in_hours - hours)*60.0
    minutes = int(minutes_decimal // 1)
    seconds_decimal = (minutes_decimal - minutes)*60.0
    seconds = int(seconds_decimal // 1)
    if hours < 10:
        hours = str(0)+str(hours)
    else:
        hours = str(hours)
    if minutes < 10:
        minutes = str(0)+str(minutes)
    else:
        minutes = str(minutes)
    if seconds < 10:
        seconds = str(0)+str(seconds)
    else:
        seconds = str(seconds)
    return hours+":"+minutes+":"+seconds

def Dec_to_text(dec):
    if dec >=0:
        dec_sign = 1
    else:
        dec_sign = -1
        dec = -dec
    hours = int(dec // 1)
    minutes_decimal = (dec - hours)*60.0
    minutes = int(minutes_decimal // 1)
    seconds_decimal = (minutes_decimal - minutes)*60.0
    seconds = int(seconds_decimal // 1)
    if hours < 10:
        hours = str(0)+str(hours)
    else:
        hours = str(hours)
    if minutes < 10:
        minutes = str(0)+str(minutes)
    else:
        minutes = str(minutes)
    if seconds < 10:
        seconds = str(0)+str(seconds)
    else:
        seconds = str(seconds)
    if dec_sign == 1:
        return "+"+hours+":"+minutes+":"+seconds
    else:
        return "-"+hours+":"+minutes+":"+seconds

def clean_astrometry(name):
    try:
        os.remove(name+'-indx.xyls')
        os.remove(name+'.axy')
        os.remove(name+'.corr')
        os.remove(name+'.match')
        os.remove(name+'.new')
        os.remove(name+'.tif')
        os.remove(name+'.rdls')
        os.remove(name+'.solved')
        os.remove(name+'.wcs')
    except:
        None

def list_files(camera):
    file_list=[]
    for i in camera.list_files():
        if 'CANONMSC' not in i:
            file_list+=camera.list_files(i[:-1])
    return file_list

def purge_files(camera):
    for i in list_files(camera):
        camera.delete_files(i)

def get_camera_shutter_time(camera):
    time = camera.lua_execute('tv96_to_usec(get_user_tv96())')
    if time < 1000000:
        return time/1000000.
    else:
        return time/1000000

def get_iso(camera):
    return camera.lua_execute('get_iso_mode()')

def set_iso(camera, iso):
    camera.lua_execute('set_iso_mode('+str(iso)+')')
    camera.lua_execute('press("shoot_half")')
    return get_iso(camera)


def get_live_view(camera):
    camera._lua.eval("""
        function()
            status, err = con:live_dump_start('/tmp/live_view_frame')
   	    for i=1,1 do
   	        status, err = con:live_get_frame(29)
     	        status, err = con:live_dump_frame()
   	    end
   	    status, err = con:live_dump_end()
            return err
        end
    """)()
    lv_aspect_ratio = {0:'LV_ASPECT_4_3', 1:'LV_ASPECT_16_9', 2:'LV_ASPECT_3_2'}
    fb_type = {0:12, 1:8, 2:16, 3:16, 4:8 }
    file_header_dtype = numpy.dtype([('magic','int32'),('header_size', 'int32'),('version_major', 'int32'),('version_minor','int32')])
    frame_length_dtype = numpy.dtype([('length','int32')])
    frame_header_dtype = numpy.dtype([('version_major','int32'),('version_minor', 'int32'),('lcd_aspect_ratio', 'int32'),
        ('palette_type','int32'), ('palette_data_start','int32'), ('vp_desc_start','int32'), ('bm_desc_start','int32'),
        ('bmo_desc_start','int32')])
    block_description_dtype = numpy.dtype([('fb_type','int32'),('data_start','int32'),('buffer_width','int32'),
        ('visible_width','int32'),('visible_height','int32'),('margin_left','int32'), ('margin_top','int32'),
        ('margin_right','int32'),('margin_bottom','int32')])

    myFile = open('/tmp/live_view_frame','r')

    file_header=numpy.fromfile(myFile, dtype=file_header_dtype, count=1)
    frame_length=numpy.fromfile(myFile, dtype=frame_length_dtype, count=1)
    frame_header=numpy.fromfile(myFile, dtype=frame_header_dtype, count=1)
    vp_description=numpy.fromfile(myFile, dtype=block_description_dtype, count=1)
    vp_bpp = fb_type[int(vp_description['fb_type'])]
    vp_frame_size=vp_description['buffer_width']*vp_description['visible_height']*vp_bpp/8 # in byte !

    bm_description=numpy.fromfile(myFile, dtype=block_description_dtype, count=1)
    bm_bpp = fb_type[int(bm_description['fb_type'])]
    bm_frame_size=bm_description['buffer_width']*bm_description['visible_height']*bm_bpp/8

    bmo_description=numpy.fromfile(myFile, dtype=block_description_dtype, count=1)
    bmo_frame_size=bmo_description['buffer_width']*bmo_description['visible_height']*fb_type[int(bmo_description['fb_type'])]/8

    if vp_description['data_start'] > 0:
        vp_raw_img=numpy.fromfile(myFile, dtype=numpy.uint8, count=vp_frame_size)
        y=vp_raw_img[1::2].reshape(int(vp_description['visible_height']),int(vp_description['buffer_width']))
        u=numpy.empty(vp_frame_size/2, dtype=numpy.uint8)
        u[0::2]=vp_raw_img[0::4]
        u[1::2]=vp_raw_img[0::4]
        u=u.reshape(int(vp_description['visible_height']),int(vp_description['buffer_width']))
        v=numpy.empty(vp_frame_size/2, dtype=numpy.uint8)
        v[0::2]=vp_raw_img[2::4]
        v[1::2]=vp_raw_img[2::4]
        v=v.reshape(int(vp_description['visible_height']),int(vp_description['buffer_width']))
        raw_yuv=numpy.dstack((y,u,v))[:,0:int(vp_description['visible_width']),:]
        vp_rgb=cv2.cvtColor(raw_yuv, cv2.COLOR_YUV2BGR)
    if bm_description['data_start'] > 0:
        bm_raw_img=numpy.fromfile(myFile, dtype=numpy.uint8, count=bm_frame_size)
        y=bm_raw_img[1::2].reshape(int(bm_description['visible_height']),int(bm_description['buffer_width']))
        u=numpy.empty(bm_frame_size/2, dtype=numpy.uint8)
        u[0::2]=bm_raw_img[0::4]
        u[1::2]=bm_raw_img[0::4]
        u=u.reshape(int(bm_description['visible_height']),int(bm_description['buffer_width']))
        v=numpy.empty(bm_frame_size/2, dtype=numpy.uint8)
        v[0::2]=bm_raw_img[2::4]
        v[1::2]=bm_raw_img[2::4]
        v=v.reshape(int(bm_description['visible_height']),int(bm_description['buffer_width']))
        raw_yuv=numpy.dstack((y,u,v))[:,0:int(bm_description['visible_width']),:]
        bm_rgb=cv2.cvtColor(raw_yuv, cv2.COLOR_YUV2BGR)
    if bmo_description['data_start'] >0:
        bmo_raw_img=numpy.fromfile(myFile, dtype=numpy.int32, count=bmo_frame_size)
    myFile.close()
    return vp_rgb, bm_rgb

def main():
    app = QtGui.QApplication(sys.argv)
    app.setStyle(QtGui.QStyleFactory.create('Plastique'))
    ex = EOSUI()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
