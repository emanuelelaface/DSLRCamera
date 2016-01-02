'''
DSLRCamera -- Program to get and solve astronomical pictures from an DSLR camera.
Copyright (c) 2015-2016, Emanuele Laface (Emanuele.Laface at gmail.com)

All rights reserved.

Redistribution and use, with or without modification, are permitted provided that the following conditions are met:
Redistributions must retain the above copyright notice, this list of conditions and the following disclaimer.
Neither the name of the DSLRCamera Author nor the names of any contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

Installation:

DSLRCamera uses libgphoto and the python wrapper gphoto2 to work.
It uses also astrometry to solve the image and send an LX200 packet to a mount for position syncronization.
It comes with a small controller of my arduino focuser, but if it is not connected it isn't a problem.

Everything about astrometry can be found on their website, including the license.
You also need the index files. To establish which index you need to know the size of your FOV and compare with
the information provided by DSLRCamera here: http://astrometry.net/doc/readme.html

DSLRCamera assume that astrometry is installed within the following directories:
directory: /usr/local/astrometry
solve-field: /usr/local/astrometry/bin/solve-field
index files: anywhere where the solve-filed can find them. The default is /usr/local/astrometry/data
the ngc2000.fits: /usr/local/astrometry/extras/ngc2000.fits
these informations are hard-coded in the python. I know, this is not a good practice but it's life.

Python libraries that you should have if you have python installed:

sys
time
numpy
subprocess
serial
socket
threading
os

Python libraries that you probably have to install:

gphoto2
rawpy
astropy
opencv (cv2)
imageio
PyQt4

I have as python installer Anaconda and for extra packages I use pip. 90% of the packages installed with no problem.
I had some problem with some of them but I am not sure which one, so if unsure feel free to contact me.

Use:
run it and the interface should be simple enough. The video mode is done using the LiveView, so the raw file saved is with a lower
resolution than the camera picture. I know that the modern cameras has a real movie record capabilities but I own only a Canon EOS 1000D
so I cannot try it.
The only parameters availables for the camera in the interface are ISO and Speed because I use only that two for my pictures, but
nothing prevent to enable the others. The available features are in the initCamera method of the user interface, so it is
quite simple to enable them in a button or a choice.
Feel free to modify the code and contact me if you want to redistribute here.

Please consider that I am not a professinal coder, I did this program just for fun, so don't complain if it is not good.
