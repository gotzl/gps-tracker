import matplotlib.pyplot as plt
import numpy as np

import sys, serial, time, glob, os
import pynmea2
from pathlib import Path
from geopy import distance
from pathlib import Path

sys.path.append('.')
import gpstracker as tr

meta = tr._track_meta()
ref = tr.readTrack(os.path.join(str(Path.home()), 'Downloads/lap000.bin'), meta)
track = tr.readTrack(os.path.join(str(Path.home()), 'Downloads/lap001.bin'), meta)

tup = lambda x: (x.lon*1e-7,x.lat*1e-7)
# put data in a more convenient structure
class _track(object):
    def __init__(self, track):
        self.time = np.array([t.time*1e-3 for t in track])
        self.speed = np.array([t.speed*1e-3 for t in track])
        self.location = np.array([tup(t.location) for t in track])
        
        ds = []
        dist = []
        
        for i in range(len(self.location)):
            if i==0:
                _ds = 0
                _s = 0
            else:
                _ds = distance.distance(self.location[i], self.location[i-1]).m
                _s = dist[-1] + _ds
            ds.append(_ds)
            dist.append(_s)

        self.ds = np.array(ds)
        self.dist = np.array(dist)
        
ref = _track(ref)
track = _track(track)

meta.start_time,meta.finish_time,meta.finish_time-meta.start_time,meta.time

plt.figure(figsize=(8,8))
plt.subplot(2, 1, 1)
plt.plot(ref.location[:,0],ref.location[:,1],
         linestyle="None", markersize=1, marker='o')
plt.plot(track.location[:,0],track.location[:,1],
         linestyle="None", markersize=1, marker='o')
plt.plot(meta.start_finish.lon*1e-7, meta.start_finish.lat*1e-7, 
         marker='o', markersize=3, color="red")

plt.subplot(2, 1, 2)
plt.plot(ref.dist, ref.speed)
plt.plot(track.dist, track.speed)

plt.show()
