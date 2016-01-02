#!/usr/bin/python

import sys
import gphoto2
import time
import numpy
import rawpy
import astropy.io.fits
import astropy.wcs
import subprocess
import cv2
import threading
import os
import serial
import imageio
import socket
from PyQt4 import QtGui, QtCore

class solveImage:
    def __init__(self, filename, scale):
        name, extension = os.path.splitext(filename)
        self.cr2_to_ppm(name)
        self.astrometry(name, scale)
        self.clean(name)

    def cr2_to_ppm(self, name):
        raw = rawpy.imread(name+".CR2")
        img = raw.postprocess(no_auto_bright=True, user_flip=False, output_bps=16)
        imageio.imsave(name+'.ppm', img)
        self.center_x = img.shape[1]/2
        self.center_y = img.shape[0]/2

    def astrometry(self, name, scale):
        subprocess.call(["/usr/local/astrometry/bin/solve-field", "--cpulimit", "10", "--downsample", "2", "--tweak-order", "2", "--scale-units", "arcsecperpix", "--scale-low", str(numpy.floor(scale*100)/100), "--scale-high", str(numpy.ceil(scale*100)/100), "--no-plots", "--overwrite", name+".ppm"])
        if os.path.isfile(name+'.solved'):
           self.is_solved = True
           wcs = astropy.wcs.WCS(astropy.io.fits.open(name+'.new')[0].header)
           self.RA, self.Dec = wcs.wcs_pix2world(self.center_x, self.center_y, 1)
        else:
           self.is_solved = False

    def RA_to_text(self, ra):
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

    def Dec_to_text(self, dec):
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

    def clean(self, name):
        try:
           os.remove(name+'-indx.xyls')
           os.remove(name+'.axy')
           os.remove(name+'.corr')
           os.remove(name+'.match')
           os.remove(name+'.new')
           os.remove(name+'.ppm')
           os.remove(name+'.rdls')
           os.remove(name+'.solved')
           os.remove(name+'.wcs')
        except:
           print "Some file was not here."

    def update_mount(self):
        address = "10.0.1.251"
        port = 4030
        bufsize = 1024
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
           s.connect((address, port))
           s.send(":Sr"+self.RA_to_text(self.RA)+"#")
           data = s.recv(bufsize)
           s.send(":Sd"+self.Dec_to_text(self.Dec)+"#")
           data = s.recv(bufsize)
           s.send(":CM#")
           data = s.recv(bufsize)
           s.close()
           return True
        except:
           return False

class EOSUI(QtGui.QWidget):
    
    def __init__(self):
        super(EOSUI, self).__init__()
        
        self.initCamera()
        self.initUI()
        
    def initCamera(self):
        self.context = gphoto2.gp_context_new()
        error, self.camera = gphoto2.gp_camera_new()
        error = gphoto2.gp_camera_init(self.camera, self.context)
        if error != 0:
           print "Init error!"
           quit()
        self.camera_name = self.context.camera_autodetect()[0][0]

        error, self.config = gphoto2.gp_camera_get_config(self.camera, self.context)
        self.set_parameter('imageformat', 7) # 0 to 7
        self.set_parameter('iso', 0) # 0 to 5
        self.set_parameter('whitebalance', 7) # 0 to 7
        self.set_parameter('whitebalanceadjusta', 0) # 0 to 18
        self.set_parameter('whitebalanceadjustb', 0) # 0 to 18
        self.set_parameter('shutterspeed', 0) # 0 to 52
        self.set_parameter('picturestyle', 0) # 0 to 8
        self.set_parameter('drivemode', 0) # 0 to 4
        self.set_parameter('autoexposuremode', 3) # 0 to 15
        self.set_parameter('colorspace', 0) # 0 to 1
        self.set_parameter('meteringmode', 2) # 0 to 2

    def initUI(self):      

        self.go_button = QtGui.QPushButton('GO!', self)
        self.stop_button = QtGui.QPushButton('Stop', self)
        iso_select = QtGui.QComboBox(self)
        shutter_select = QtGui.QComboBox(self)
        self.bulb_time = QtGui.QLineEdit(self)
        self.bulb_time.setText('1')
        self.bulb_pics = QtGui.QLineEdit(self)
        self.bulb_pics.setText('1')
        self.video_flag = QtGui.QCheckBox('Video', self)
        self.record_flag = QtGui.QCheckBox('Record', self)
        self.record_file = QtGui.QLineEdit(self)
        self.record_file.setText('')
        self.zoom_select = QtGui.QComboBox(self)
        self.reticle_flag = QtGui.QCheckBox('Reticle', self)
        self.solve_button = QtGui.QPushButton('Solve', self)
        self.solve_scale = QtGui.QComboBox(self)
        self.mount_button = QtGui.QPushButton('Update Mount', self)
        self.contours_flag = QtGui.QCheckBox('Contours', self)
        self.contours_slider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.contours_slider.setRange(0, 255)
        focuser_slider = QtGui.QSlider(QtCore.Qt.Vertical, self)
        focuser_slider.setRange(-255,255)
        self.image_label = QtGui.QLabel(self)
        self.text_line = QtGui.QLabel("Camera is ready", self)
        try:
            self.arduino = serial.Serial( port='/dev/ttyACM0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            self.arduino.write('0')
        except:
            print "Focuser not found!"
            focuser_slider.setEnabled(False)

        self.myImage = QtGui.QImage((numpy.ones((768,512,3))*155).astype(numpy.uint8), 768, 512, QtGui.QImage.Format_RGB888)
        self.image_event = 1

        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        grid.addWidget(QtGui.QLabel('ISO'), 0,0)
        grid.addWidget(QtGui.QLabel('Shutter'), 0,1)
        grid.addWidget(QtGui.QLabel('Bulb Time'), 0,2)
        grid.addWidget(QtGui.QLabel('Bulb Pics'), 0,3)
        grid.addWidget(QtGui.QLabel('Video File'), 0,6)
        grid.addWidget(QtGui.QLabel('Zoom'), 0,7)

        grid.addWidget(iso_select, 1,0)
        grid.addWidget(shutter_select, 1,1)
        grid.addWidget(self.bulb_time, 1,2)
        grid.addWidget(self.bulb_pics, 1,3)
        grid.addWidget(self.video_flag, 1,4)
        grid.addWidget(self.record_flag, 1,5)
        grid.addWidget(self.record_file, 1,6)
        grid.addWidget(self.zoom_select, 1, 7)
        grid.addWidget(self.reticle_flag, 1, 8)

        grid.addWidget(self.go_button, 2,0)
        grid.addWidget(self.stop_button, 2,1)

        grid.addWidget(self.solve_button, 2,3)
        grid.addWidget(self.solve_scale, 2,4)
        grid.addWidget(self.mount_button, 2,5)
        
        grid.addWidget(self.image_label, 3, 0, 1, 9)

        grid.addWidget(self.contours_flag, 2, 7)
        grid.addWidget(self.contours_slider, 2, 8)
        grid.addWidget(focuser_slider, 3,9)

        grid.addWidget(self.text_line, 4, 0, 1, 10)
        self.setLayout(grid) 
        self.stop = threading.Event()
        
        iso_select.activated[str].connect(self.set_iso)        
        shutter_select.activated[str].connect(self.set_shutter)        
        self.video_flag.stateChanged.connect(self.set_video)
        self.go_button.clicked[bool].connect(self.go)
        self.stop_button.clicked[bool].connect(self.stop_actions)
        self.stop_button.setEnabled(False)
        self.record_flag.setEnabled(False)
        self.record_file.setEnabled(False)
        self.reticle_flag.setEnabled(False)
        self.contours_flag.setEnabled(False)
        self.contours_slider.setEnabled(False)
        self.zoom_select.setEnabled(False)
        self.solve_button.clicked[bool].connect(self.solve)
        self.mount_button.clicked[bool].connect(self.update_mount)
        self.solve_button.setEnabled(False)
        self.mount_button.setEnabled(False)
        self.solve_scale.setEnabled(False)
        focuser_slider.valueChanged[int].connect(self.focuser)

        iso_choiche = []
        shutter_choiche = []
        solve_scale_choiche = ['0.985', '0.255']
        self.current_picture='IMG_0000.CR2'
        
        error = 0
        set_value = 0
        while (error == 0):
            error, value = gphoto2.gp_widget_get_choice(gphoto2.gp_widget_get_child_by_name(self.config, 'iso')[1], set_value)
            iso_choiche.append(value)
            set_value = set_value + 1

        for i in iso_choiche[0:set_value-1]:
          iso_select.addItem(i)

        error=0
        set_value = 0
        while (error == 0):
            error, value = gphoto2.gp_widget_get_choice(gphoto2.gp_widget_get_child_by_name(self.config, 'shutterspeed')[1], set_value)
            shutter_choiche.append(value)
            set_value = set_value + 1

        for i in shutter_choiche[0:set_value-1]:
           shutter_select.addItem(i)

        for i in solve_scale_choiche:
            self.solve_scale.addItem(i)
        
        self.zoom_select.addItem(str(1))
        for i in range (2, 12, 2):
            self.zoom_select.addItem(str(i))

        self.setGeometry(50, 50, -1 ,-1)
        self.setWindowTitle(self.camera_name)
        self.show()

    def paintEvent(self, e):
        if self.image_event == 1:
            self.lock_paint = 1
            self.image_label.setPixmap(QtGui.QPixmap.fromImage(self.myImage))
            self.lock_paint = 0
            self.image_event=0
        
    def set_iso(self, value):
        self.set_parameter('iso',self.sender().currentIndex())
        self.text_line.setText('ISO set to '+value)

    def set_shutter(self, value):
        self.set_parameter('shutterspeed',self.sender().currentIndex())
        self.text_line.setText('Shutter set to '+value)
        if value == 'bulb':
            self.bulb_time.setEnabled(True)
            self.bulb_time.setText('1')
            self.bulb_pics.setEnabled(True)
            self.bulb_pics.setText('1')
        else:
            self.bulb_time.setEnabled(False)
            self.bulb_time.setText('')
            self.bulb_pics.setEnabled(False)
            self.bulb_pics.setText('')
        
    def set_video(self, state):
        if state == QtCore.Qt.Checked:
           self.record_flag.setEnabled(True)
           self.record_file.setEnabled(True)
           self.record_file.setText('video.avi')
           self.reticle_flag.setEnabled(True)
           self.zoom_select.setEnabled(True)
           self.contours_flag.setEnabled(True)
           self.contours_slider.setEnabled(True)
        else:
           self.record_flag.setEnabled(False)
           self.record_file.setEnabled(False)
           self.record_file.setText('')
           self.reticle_flag.setEnabled(False)
           self.zoom_select.setEnabled(False)
           self.contours_flag.setEnabled(False)
           self.contours_slider.setEnabled(False)
           self.reticle_flag.setCheckState(QtCore.Qt.Unchecked)
           self.record_flag.setCheckState(QtCore.Qt.Unchecked)

    def set_parameter(self, parameter_name, set_value):
        error, parameter = gphoto2.gp_widget_get_child_by_name(self.config, parameter_name)
        if error < 0:
            print "Parameter does not exist"
            return
        if parameter_name != 'bulb' and parameter_name != 'capture':
            error, value = gphoto2.gp_widget_get_choice(parameter, set_value)
        else:
            error = 0
            value = set_value
        if error < 0:
            print "Value out of range"
            return
        error = gphoto2.gp_widget_set_value(parameter, value)
        error = gphoto2.gp_camera_set_config(self.camera, self.config, self.context)

    def get_parameter(self, parameter_name):
        error, parameter = gphoto2.gp_widget_get_child_by_name(self.config, parameter_name)
        if error < 0:
            print "Parameter does not exist"
            return
        return gphoto2.gp_widget_get_value(parameter)

    def stop_actions(self):
        self.stop.set()

    def go(self, pressed):
       if self.video_flag.checkState() == QtCore.Qt.Checked:
            tr = threading.Thread(target=self.get_video_thread, args=(self.stop,))
       else:
            tr = threading.Thread(target=self.take_picture_thread, args=(self.stop,))
       tr.start()

    def take_picture_thread(self, stop):
       self.go_button.setEnabled(False)
       self.stop_button.setEnabled(True)
       if self.get_parameter('shutterspeed')[1] != 'bulb':
            self.text_line.setText('Capturing...')
            error, self.file_path = gphoto2.gp_camera_capture(self.camera, gphoto2.GP_CAPTURE_IMAGE, self.context)
            camera_file = gphoto2.check_result(gphoto2.gp_camera_file_get(self.camera, self.file_path.folder, self.file_path.name, gphoto2.GP_FILE_TYPE_NORMAL, self.context))
            error = gphoto2.gp_file_save(camera_file, self.file_path.name)
            img_num = 1
            while os.path.isfile(self.current_picture):
                self.current_picture = 'IMG_'+str(img_num).zfill(4)+'.CR2'
                img_num = img_num + 1
            if hasattr(self, 'file_path'):
                os.rename(self.file_path.name, self.current_picture)
            raw = rawpy.imread(self.current_picture)
            image = raw.postprocess(user_flip=False, output_bps=8)
            image = cv2.resize(image,(768, 512))
            self.myImage = QtGui.QImage(image.astype(numpy.uint8), 768, 512, QtGui.QImage.Format_RGB888)
            self.text_line.setText('Done')
            self.image_event = 1
            self.update()
       else:
           secs = int(self.bulb_time.text())
           i = 0
           while i < int(self.bulb_pics.text()) and not stop.isSet():
              self.file_path = self.get_bulb_picture(secs, stop)
              img_num = 1
              while os.path.isfile(self.current_picture):
                  self.current_picture = 'IMG_'+str(img_num).zfill(4)+'.CR2'
                  img_num = img_num + 1
              if hasattr(self, 'file_path'):
                  os.rename(self.file_path.name, self.current_picture)
              raw = rawpy.imread(self.current_picture)
              image = raw.postprocess(user_flip=False, output_bps=8)
              image = cv2.resize(image,(768, 512))
              self.myImage = QtGui.QImage(image.astype(numpy.uint8), 768, 512, QtGui.QImage.Format_RGB888)
              self.text_line.setText('Image '+str(i+1)+" of "+self.bulb_pics.text()+' done')
              self.image_event = 1
              self.update()
              i = i + 1
           if stop.isSet():
              self.text_line.setText('Interruped by user!')
           stop.clear()
       if hasattr(self, 'file_path'):
           self.solve_button.setEnabled(True)
           self.solve_scale.setEnabled(True)
       self.go_button.setEnabled(True)
       self.stop_button.setEnabled(False)

    def get_bulb_picture(self, secs, stop):
        self.set_parameter('bulb', 1)
        i = 0
        while i < secs and not stop.isSet():
            time.sleep(1)
            self.text_line.setText('Elapsed '+str(i+1)+' of '+str(secs)+' secs')
            i = i + 1
        self.set_parameter('bulb', 0)
        self.text_line.setText('Saving image...')
        time.sleep(2)
        status = 0
        i = 0
        while status != 2:
            error, status, self.file_path = gphoto2.gp_camera_wait_for_event(self.camera, i, self.context)
            i = i + 1
        camera_file = gphoto2.check_result(gphoto2.gp_camera_file_get(self.camera, self.file_path.folder, self.file_path.name, gphoto2.GP_FILE_TYPE_NORMAL, self.context))
        error = gphoto2.gp_file_save(camera_file, self.file_path.name)
        self.text_line.setText('Done')
        return self.file_path

    def get_video_thread(self, stop):
        self.go_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.record_flag.setEnabled(False)
        self.record_file.setEnabled(False)
        self.video_flag.setEnabled(False)
        if self.record_flag.checkState() == QtCore.Qt.Checked:
            fourcc = 0x00000000 #cv2.cv.CV_FOURCC('r', 'a', 'w', ' ')
            out = cv2.VideoWriter()
            file_counter = 1
            filename = str(self.record_file.text())
            name, extension = os.path.splitext(filename)
            while os.path.isfile(filename):
              filename = name+'-'+str(file_counter).zfill(4)+'.avi'
              file_counter = file_counter + 1
            out.open(filename,fourcc, 24, (768,512), True)

        framecount = 0
        start_time = time.time()
        while not stop.isSet():
            if framecount == 30:
                self.text_line.setText('Video '+str(int(30/(time.time()-start_time)))+' fps')
                start_time = time.time()
                framecount = 0
            error, image = gphoto2.gp_camera_capture_preview(self.camera, self.context)
            data = image.get_data_and_size()
            array = numpy.fromstring(memoryview(data).tobytes(), dtype=numpy.uint8)
            img = cv2.imdecode(array, 1)
            if  int(self.zoom_select.currentText()) != 1:
                x = img.shape[0]
                y = img.shape[1]
                zoom = int(self.zoom_select.currentText())
                img = cv2.resize(img[x*(zoom-1)/(2*zoom):x*(zoom+1)/(2*zoom),y*(zoom-1)/(2*zoom):y*(zoom+1)/(2*zoom),:], (768,512))
            if self.record_flag.checkState() == QtCore.Qt.Checked:
                out.write(img)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            if self.contours_flag.checkState() == QtCore.Qt.Checked:
                img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#                img_gray = cv2.medianBlur(img_gray, 5)
#                thresh = cv2.adaptiveThreshold(img_gray ,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
                ret, thresh = cv2.threshold(img_gray, self.contours_slider.value() , 255, cv2.THRESH_BINARY)
                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, contours, -1, (255,255,255), 1)
                area = 0
                for i in contours:
                    area = area+cv2.contourArea(i)
                print area

            if self.reticle_flag.checkState() == QtCore.Qt.Checked:
                cv2.line(img, (768/2,0),(768/2,512),(255,255,255),1)
                cv2.line(img, (0,512/2),(768,512/2),(255,255,255),1)
                for i in range(30, 500, 100):
                    cv2.circle(img, (768/2,512/2), i, (255,255,255), 1)


            if self.lock_paint == 0:
                self.myImage = QtGui.QImage(img, 768, 512, QtGui.QImage.Format_RGB888)
                framecount = framecount + 1
                self.image_event = 1
                self.update()

        if self.record_flag.checkState() == QtCore.Qt.Checked:
            out.release()
        gphoto2.gp_camera_exit(self.camera, self.context)
        self.record_flag.setEnabled(True)
        self.record_file.setEnabled(True)
        self.video_flag.setEnabled(True)
        stop.clear()
        self.text_line.setText('Video stopped')
        self.go_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def solve(self, pressed):
       solve_tr = threading.Thread(target=self.solve_thread)
       solve_tr.start()

    def solve_thread(self):
       self.text_line.setText('Solving Image')
       self.solved = solveImage(self.current_picture, float(self.solve_scale.currentText()))
       if self.solved.is_solved:
           self.text_line.setText('RA: '+str(self.solved.RA)+' Dec: '+str(self.solved.Dec))
           self.mount_button.setEnabled(True)
       else:
           self.text_line.setText('Solve failed')

    def update_mount(self, pressed):
       if self.solved.update_mount():
           self.text_line.setText('Mount updated')
       else:
           self.text_line.setText('Failed to update mount')

    def focuser(self, value):
        self.arduino.write('\n')
        self.arduino.write(str(value))
        self.text_line.setText('Focuser speed '+str(value))

def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = EOSUI()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
