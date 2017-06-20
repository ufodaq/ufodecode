## General Information

This directory contains libufodecode version 0.6. This package provides a
library to decode frames from the UFO camera developed at IPE/KIT. It is able to
decode frames produced with firmware versions 4, 5 and 5 with 12-bit support.
The library is a dependency of pcilib to decode frames on-the-fly.

To set the number of pixels in x-direction other than the default of 5120
pixels, you have to pass that number in the configuration step, i.e.

    $ cmake -DIPECAMERA_WIDTH=2048 <src-dir>

This package also contains a stand-alone offline decoder called `ipedec` to
decode raw frames acquired with the `pcitool` program. More information is
available by calling

    $ ipedec -h

## Installation

Please see the file called INSTALL.


## Licensing

Please see the file called COPYING.
