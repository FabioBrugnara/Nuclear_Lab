#!/usr/bin/env python

import sys, os, argparse
import ctypes as cty
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.backends.backend_pdf
from matplotlib.colors import LogNorm

from scipy.optimize import curve_fit
from scipy.special import erf


#### Decoder defs
class opt_flags_s(cty.Structure):
    _fields_ = [('export_alice_csv', cty.c_int),
                ('export_csv', cty.c_int),
                ('chip_id_sel', cty.c_uint16)
    ]

class rawfile_type_s(cty.Structure):
    _fields_ = [('thresscan', cty.c_int),
                ('digiscan', cty.c_int),
                ('whitescan', cty.c_int),
                ('injs', cty.c_uint8),
                ('charge_steps', cty.c_uint8)
    ]

script_dir = os.path.dirname(os.path.realpath(__file__))
decoder = cty.cdll.LoadLibrary(script_dir + '/process_raw.so')
decoder.set_opt.argtypes = [cty.POINTER(opt_flags_s)]
decoder.get_hitmap.argtypes = [cty.c_uint16,
    np.ctypeslib.ndpointer(cty.c_uint32, flags='C_CONTIGUOUS')]
decoder.get_histo.argtypes = [cty.c_uint16,
    np.ctypeslib.ndpointer(cty.c_uint32, flags='C_CONTIGUOUS')]
decoder.decode_file.argtypes = [cty.c_char_p, cty.c_int]
####


def index_to_chipid(index):
    stave_id = index//10

    chip_id_lsb = index%10
    if (chip_id_lsb > 4):
        chip_id_lsb += 3

    chip_id = 0x70 | (stave_id<<8) | chip_id_lsb
    return str(hex(chip_id))


def plot_heatmap(logscale=False):

    print('Plotting hitmap...')
    heat_pdfdoc = matplotlib.backends.backend_pdf.PdfPages('heatplots.pdf')

    for i in range(30):

        # read chip data
        hmap = np.zeros(512*1024, dtype=np.uint32)
        decoder.get_hitmap(cty.c_uint16(i), hmap)
        if not np.any(hmap):
            continue
        hmap = hmap.reshape(1024,512)

        # plot
        fig = plt.figure()
        if logscale or (hmap.max() > 1000):
            plt.matshow(hmap, norm=LogNorm())
        else:
            plt.matshow(hmap)

        plt.colorbar()
        plt.title('chip id:' + index_to_chipid(i))

        heat_pdfdoc.savefig(dpi=600, bbox_inches='tight')
        plt.close('all')

    heat_pdfdoc.close()



def threshold_histogram(charge_steps, fout=None):

    def error_function(z, m, s, a, b):
        return b + a * erf((z-m)/s)

    E_PER_CHARGE_STEP=10

    print('Plotting histograms...')
    hist_pdfdoc = matplotlib.backends.backend_pdf.PdfPages('histograms.pdf')

    fit_out_file = None
    if fout != None:
        fit_out_file = open(fout, "a")

    for i in range(30):

        c_step = np.arange(1, charge_steps+1)
        histo = np.zeros(charge_steps, dtype=np.uint32)
        decoder.get_histo(cty.c_uint16(i), histo)
        if not np.any(histo):
            continue

        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8,8))
        fig.subplots_adjust(hspace=0)

        # hist data
        ax2.bar(c_step, histo, log=True)
        ax1.bar(c_step, histo)

        # fit data
        try:
            popt, _ = curve_fit(error_function, c_step, histo)

            x_fit = np.linspace(np.amin(c_step), np.amax(c_step), 1000)
            y_fit = error_function(x_fit, *popt)
            thresh = popt[0]*E_PER_CHARGE_STEP
            thresh_err = popt[1]*E_PER_CHARGE_STEP

            fit_result_string = "{:.0f} +/- {:.0f} ($e^-$)".format(thresh, thresh_err)
            ax1.plot(x_fit, y_fit, color='red',
                    label="erf(x)\nThreshold: " + fit_result_string)

            if fit_out_file != None:
                fit_out_file.write("{} {:.0f} {:.0f}\n".format(
                    index_to_chipid(i), thresh, thresh_err))

        except:
            print("Fit failed")

        ax2.set_xlabel("Charge step ($e^-$/10)")
        ax2.set_ylabel("Hits (log)")
        ax1.set_ylabel("Hits")
        ax1.legend()
        ax1.set_title("chip id:" + index_to_chipid(i))

        hist_pdfdoc.savefig()
        plt.close('all')

    hist_pdfdoc.close()
    if fit_out_file != None:
        fit_out_file.close()



def digiscan_count(digiscan_type):

    for i in range(30):
        # read chip data
        hmap = np.zeros(512*1024, dtype=np.uint32)
        decoder.get_hitmap(cty.c_uint16(i), hmap)
        if not np.any(hmap):
            continue

        if digiscan_type == 'digiscan':
            dead = np.count_nonzero(hmap!=10)
            print('chipid: {} dead pixel count: {}'.format(index_to_chipid(i), dead))
        elif digiscan_type == 'whitescan':
            dead = np.count_nonzero(hmap!=0)
            print('chipid: {} unmaskable pixel count: {}'.format(index_to_chipid(i), dead))


#def long_threshold_hist(data):
#
#    c_hmap = np.zeros((30,1024,512));
#
#    charge_level = data[:,0]
#    x = data[:,1]
#    y = data[:,2]
#    z = data[:,3]
#
#    # fill array
#    for n in range(len(charge_level)-1):
#        c_hmap[charge_level[n], x[n], y[n]] = z[n]
#
#    # hide hot and dead pixels otherwise average will fail with ZeroDivisionError..
#    stuck_map  = np.all(c_hmap[0]==c_hmap, axis=0)
#    stuck_pixs = np.where(stuck_map)
#    for x, y in zip(*stuck_pixs):
#        c_hmap[-1,x,y] = 1
#        c_hmap[0,x,y] = 0
#
#    # compute gaussian hist
#    gauss_hist = np.zeros((29,1024,512));
#    for index in range(29):
#        gauss_hist[index] = c_hmap[index+1] - c_hmap[index]
#
#
#    # handle other anomalies
#    testsum = np.sum(gauss_hist, axis=0)
#    anom_pixels = np.where(testsum == 0)
#    print("Anomalous pixels: ", len(anom_pixels[0]))
#    for x, y in zip(*anom_pixels):
#        line = gauss_hist[:,x,y]
#        line[line<0] = 0
#
#
#    bin_centers = np.arange(29)+0.5
#    bin_centers = np.repeat(bin_centers, 512*1024)
#    bin_centers = np.reshape(bin_centers, (29,1024,512))
#
#    thresh_avg = np.average(bin_centers, axis=0, weights=gauss_hist)
#    thresh_std = np.average((bin_centers-thresh_avg)**2, axis=0, weights=gauss_hist)
#
#    # "restore" dead pixels
#    for x, y in zip(*stuck_pixs):
#        thresh_avg[x,y] = 0
#        thresh_std[x,y] = 0
#
#
#    heat_pdfdoc = matplotlib.backends.backend_pdf.PdfPages("long_thresh.pdf")
#
#    # plot avg
#    plt.matshow(thresh_avg)
#    plt.colorbar()
#    plt.title("Threshold (ADC units)")
#    heat_pdfdoc.savefig(dpi=600, bbox_inches='tight')
#    plt.close()
#
#    # plot std
#    plt.matshow(thresh_std, norm=LogNorm())
#    plt.colorbar()
#    plt.title("Threshold std (ADC units)")
#    heat_pdfdoc.savefig(dpi=600, bbox_inches='tight')
#    plt.close()
#
#
#    #histogram thrsh values
#    fig, (ax1, ax2) = plt.subplots(2)
#    fig.subplots_adjust(hspace=0.5)
#
#    ax1.hist(thresh_avg.flatten(), bins=30, log=True)
#    ax1.set_title("Threshold average")
#    ax1.set_xlabel("Charge (ADC units)")
#    ax1.set_ylabel("Number of pixels")
#
#    ax2.hist(thresh_std.flatten(), log=True)
#    ax2.set_title("Threshold std")
#    ax2.set_xlabel("Charge (ADC units)")
#    ax2.set_ylabel("Number of pixels")
#
#    heat_pdfdoc.savefig()
#    plt.close()
#
#    heat_pdfdoc.close()



if __name__ == '__main__':

    def str_to_int(x):
        return int(x, 0)

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', type=str, dest='raw_file', required=True,
            help='Select raw datafile to analyze')
    parser.add_argument('-c', action='store_true', default=False,
            help='Export any data from the raw file in csv format')
    parser.add_argument('-A', metavar='chip_id', dest='acsv_chip_id', type=str_to_int,
            help='Export thresholdscan data in \'ALICE Mosaic\' csv format, implies -T')
    parser.add_argument('-l', action='store_true', default=False,
            help='use log scale for hit histograms')
    parser.add_argument('-S', type=str, dest='fit_out_file', default=None,
            help='Save fit results to file')
    args = parser.parse_args()


    decoder_opts = opt_flags_s()

    if args.c:
        decoder_opts.export_csv = 1
    if args.acsv_chip_id:
        decoder_opts.export_alice_csv = 1
        decoder_opts.chip_id_sel = args.acsv_chip_id

    if not os.path.isfile(args.raw_file):
        print("No such file: ", args.raw_file)
        sys.exit(-1)


    print("Decoding file...")
    fname_enc = bytes(args.raw_file, 'utf-8')
    fname = cty.c_char_p(fname_enc)
    decoder.set_opt(cty.byref(decoder_opts))

    ret = decoder.decode_file(fname, 1)
    if (ret != 0):
        print("Decoder Error")

    print("\nSummary:")
    decoder.print_summary()
    print("")


    rawfile_info = rawfile_type_s()
    decoder.get_rawfile_info(cty.byref(rawfile_info))


    digiscan_type = None
    if rawfile_info.digiscan == 1:
        digiscan_type = 'digiscan'
    elif rawfile_info.whitescan == 1:
        digiscan_type = 'whitescan'

    if not args.acsv_chip_id:
        plot_heatmap(args.l)

    if rawfile_info.thresscan == 1:
        threshold_histogram(rawfile_info.charge_steps, args.fit_out_file)
    if args.acsv_chip_id:
        decoder.write_alice_threshmap()
    if digiscan_type != None:
        digiscan_count(digiscan_type)
