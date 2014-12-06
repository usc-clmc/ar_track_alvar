#!/usr/bin/env python

import os

n_markers = 32
max_id = 255
marker_id_offset = max_id/n_markers

for i in range(n_markers):
    print i
    os.system("rosrun ar_track_alvar createMarker -a -r 4 "+str(i*32))

