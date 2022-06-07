from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import os, argparse

import ctypes as cty
import subprocess
from threading import Thread


class monitor:

    def __init__(self, fname, refresh):
        self.fname = fname

        app = pg.mkQApp("Monitor")
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.decoder = cty.cdll.LoadLibrary(script_dir + '/process_raw.so')
        self.decoder.get_hitmap.argtypes = [cty.c_uint16,
                np.ctypeslib.ndpointer(cty.c_uint32, flags="C_CONTIGUOUS")]
        self.decoder.decode_file.argtypes = [cty.c_char_p, cty.c_int]

        # Create window with GraphicsView widget
        win = pg.GraphicsLayoutWidget()
        win.show()
        win.setWindowTitle('Monitor')
        view = win.addViewBox()
        view.setAspectLocked(True)

        # gradient editor
        self.ge = pg.GradientEditorItem()
        self.ge.loadPreset('magma')
        self.ge.setOrientation('left')
        win.addItem(self.ge)

        self.stave = pg.ImageItem(border='w')
        self.stave.setImage(np.zeros((1024*5, 512*2), dtype=np.uint32))
        self.stave.setRect(QtCore.QRect(0, 0, 1024*5, 512*2))
        view.addItem(self.stave)

        grid = pg.GridItem()
        grid.setTickSpacing(x=[1024], y=[512])
        view.addItem(grid)

        # start decoder thread
        dec_watcher = Thread(target=self.start_decoder, daemon=True)
        dec_watcher.start()

        # app timer for updater
        timer = QtCore.QTimer()
        timer.timeout.connect(self.imgupdater)
        timer.start(refresh)

        # start qt mainloop
        QtGui.QApplication.instance().exec_()


    def imgupdater(self):
        hmap_m1 = [None]*5
        hmap_m2 = [None]*5
        for index in range(5):
            # master 0x70
            hmap = np.zeros(512*1024, dtype=np.uint32)
            self.decoder.get_hitmap(cty.c_uint16(index), hmap)
            hmap = hmap.reshape(1024,512)
            hmap = np.rot90(hmap, 2)
            hmap_m1[4-index] = hmap
            #master 0x78
            hmap = np.zeros(512*1024, dtype=np.uint32)
            self.decoder.get_hitmap(cty.c_uint16(9-index), hmap)
            hmap = hmap.reshape(1024,512)
            hmap_m2[4-index] = hmap

        hmap_m1 = np.concatenate(hmap_m1, axis=0)
        hmap_m2 = np.concatenate(hmap_m2, axis=0)
        hmap_stave = np.concatenate([hmap_m1, hmap_m2], axis=1)
        clut = self.ge.getLookupTable(255)
        self.stave.setImage(hmap_stave, lut=clut)



    def start_decoder(self):
        fname_enc = bytes(self.fname, 'utf-8')
        fname = cty.c_char_p(fname_enc)
        self.decoder.decode_file(fname, 0)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', type=str, dest='raw_file', required=True,
            help='.raw file from DAQ')
    parser.add_argument('-t', type=int, dest='refresh_time', default=500,
            help='plot refresh timer in ns, suggested [100-1000], default 500')
    args = parser.parse_args()

    monitor(args.raw_file, refresh=args.refresh_time)
