#!
# MIT License
#
# Copyright (c) 2026 Adrian Port
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


sudo stty -F /dev/ttyUSB1 115200

t=0.05

while true; do
  #echo '!1121144703-014+00003311250+01736+003-03+1013-033+110831245+01650023176A' >/dev/ttyUSB1
  #echo '!1121144703-014+00003311220+01736+003-03+1013-033+110831245+016500231767' >/dev/ttyUSB1
  #echo '!1121144703-014+00003310990+01736+003-03+1013-033+110831245+016500231774' >/dev/ttyUSB1
  #echo '!1121144703-014+00003310970+01736+003-03+1013-033+110831245+016500231772' >/dev/ttyUSB1
  #echo '!1121144703-014+00003310870+01736+003-03+1013-033+110831245+016500231771' >/dev/ttyUSB1
  #echo '!1121144703-014+00003310860+01736+003-03+1013-033+110831245+016500231770' >/dev/ttyUSB1
  echo '!1121144703-014+00003311400+01736+003-03+1013-033+110831245+016500231767' >/dev/ttyUSB1
  sleep $t
done
