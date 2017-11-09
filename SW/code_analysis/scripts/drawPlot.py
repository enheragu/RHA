# @Author: Enrique Heredia Aguado <quique>
# @Date:   09-Sep-2017
# @Project: RHA
# @Last modified by:   quique
# @Last modified time: 16-Sep-2017



#!/usr/bin/python

# needs matplotlib:
# (problems with python2.7)
# sudo apt-get install python3-matplotlib
# and get dependencies:
# sudo apt-get build-dep python-matplotlib

import datetime as DT


from matplotlib import dates as mpdates
from matplotlib import pyplot as plt
from matplotlib import axes as ax
from matplotlib.dates import date2num
from matplotlib import rc
from matplotlib import cm
import numpy as np

from pylab import savefig

from cloc_list import cloc
from cloc_list import cloc_test
from cloc_list import n_cloc

yell_c = "#f9c80e"
red_c = "#ff1e1e"
grey_c = "#454545"
light_grey_c = "#cecece"
blue_c = "#3c91bc"
green_c = "#5cc47b"

font = {'family' : 'roboto',
        'weight' : 'normal',
        'size'   : 9}

rc('font', **font)


def makeClocPlot ( info_list , name):
    data = []
    code_lines = []
    comment_lines = []
    comment_percentage_lines = []
    comment_percentage_lines_max = []
    comment_percentage_lines_min = []

    debug_lines = []
    debug_percentage_lines = []
    debug_percentage_lines_max = []
    debug_percentage_lines_min = []

    #[print(cloc[x][0]) for x in range(0,5)]

    try:
        [data.append( DT.datetime.strptime( info_list[index][0], "%Y_%m_%d" ))  for index in range(0,n_cloc)]
        #[print(cloc[x][2]) for x in range(0,5)]
    except Exception as ex:
        print(ex)

    [code_lines.append( info_list[index][2] ) for index in range(0,n_cloc)]
    [comment_lines.append( info_list[index][3] ) for index in range(0,n_cloc)]
    [comment_percentage_lines.append( info_list[index][5] ) for index in range(0,n_cloc)]
    [comment_percentage_lines_max.append( info_list[index][6] ) for index in range(0,n_cloc)]
    [comment_percentage_lines_min.append( info_list[index][8] ) for index in range(0,n_cloc)]


    [debug_lines.append( info_list[index][4] ) for index in range(0,n_cloc)]
    [debug_percentage_lines.append( info_list[index][10] ) for index in range(0,n_cloc)]
    [debug_percentage_lines_max.append( info_list[index][11] ) for index in range(0,n_cloc)]
    [debug_percentage_lines_min.append( info_list[index][13] ) for index in range(0,n_cloc)]


    lines = []
    labels_fig = []

    #dates = mpdates.date2num(data)
    dates = []
    [dates.append( index ) for index in range(0,n_cloc)]

    gs_top = plt.GridSpec(5, 1, top=0.95)
    fig, ((ax1, ax4), (ax2, ax5), (ax3, ax6)) = plt.subplots(3,2, sharex=True, facecolor=light_grey_c)#, sharey=True)

    ax1.plot_date(dates, code_lines, 'r-', label="Code lines", lw=2, color=red_c)
    ax1.set_title("Code lines:",weight = "bold")

    ax2.plot_date(dates, comment_lines, 'g-', label="Comment lines", lw=2, color=yell_c)
    ax2.set_title("Comment lines:",weight = "bold")

    ax3.plot_date(dates, comment_percentage_lines, 'b-', label="Comment % lines", color=blue_c)
    ax3.plot_date(dates, comment_percentage_lines_max, 'b--', label="Comment % lines (max)", color=blue_c)
    ax3.plot_date(dates, comment_percentage_lines_min, 'b--', label="Comment % lines (min)", color=blue_c)

    ax4.axis('off')

    ax5.plot_date(dates, debug_lines, 'm-', label="Debug lines", lw=2, color=grey_c)
    ax5.set_title("Debug lines:",weight = "bold")

    ax6.plot_date(dates, debug_percentage_lines, 'c-', label="Debug % lines", color=green_c)
    ax6.plot_date(dates, debug_percentage_lines_max, 'c--', label="Debug % lines (max)", color=green_c)
    ax6.plot_date(dates, debug_percentage_lines_min, 'c--', label="Debug % lines (min)", color=green_c)

    handles_all = []
    labels_all = []
    for ax in ax1, ax2, ax3, ax4, ax5, ax6:
        ax.grid(True)
        ax.margins(0.08) # 5% padding in all directions
        #ax.set_facecolor(light_grey_c)
        handles, labels = ax.get_legend_handles_labels()
        handles_all += handles
        labels_all += labels

    ax1.set_ylabel('Number of lines',weight = "bold")
    ax2.set_ylabel('Number of lines',weight = "bold")
    ax3.set_ylabel('Percentage',weight = "bold")

    ax3.set_xlabel('  -->  Time  -->  ',weight = "bold")
    ax6.set_xlabel('  -->  Time  -->  ',weight = "bold")

    ax4.legend( handles_all[::1], labels_all[::1], loc="upper left", bbox_to_anchor=[0.15,1.1], ncol=1, shadow=True, fancybox=True, fontsize=8)
    #fontsize : int or float or {‘xx-small’, ‘x-small’, ‘small’, ‘medium’, ‘large’, ‘x-large’, ‘xx-large’}
    #title="Legend",

    #fig.suptitle(name)
    #fig.autofmt_xdate()
    fig.subplots_adjust(left=0.13, bottom=0.11, right=0.93, top=0.92, wspace=0.15, hspace=0.25)

    plt.xticks([])
    plt.show()
    img_name = name
    img_name.replace(" ","")
    img_name += ".png"
    fig.savefig(img_name, bbox_inches='tight')


def makePercentageClocPlot(info_list , name):
    # Pie chart, where the slices will be ordered and plotted counter-clockwise:
    labels = 'Functional code', 'Comment', 'Debug'
    comment_percentage = int(float(info_list[-2][5]))
    debug_percentage = int(float(info_list[-2][10]))
    functional_code_percentage = (100-comment_percentage-debug_percentage)
    sizes = [functional_code_percentage, comment_percentage, debug_percentage] #funct code, comment, debug
    explode = (0, 0.00, 0.00)  # only "explode" the 2nd slice (i.e. 'Hogs')

    fig = plt.figure()
    ax1 = fig.add_subplot(111)

    cs = [red_c, yell_c, grey_c]
    pie = ax1.pie(sizes, explode=explode, labels=labels, autopct='%1.1f%%', shadow=False, startangle=90, colors=cs)

    for pie_iter in pie[0]:
        pie_iter.set_edgecolor('white')
        pie_iter.set_linewidth(1.3)

    ax1.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.

    img_name = name
    img_name.replace(" ","")
    img_name += ".png"
    fig.savefig(img_name, bbox_inches='tight')


makeClocPlot(cloc, "Analysis of SW code")
makeClocPlot(cloc_test, "Analysis of test code")

makePercentageClocPlot(cloc, "Percentagea SW code")
makePercentageClocPlot(cloc_test, "Percentagea test code")
