# @Author: Enrique Heredia Aguado <enheragu>
# @Date:   12-Sep-2017
# @Project: RHA
# @Last modified by:   enheragu
# @Last modified time: 12-Sep-2017


from matplotlib import pyplot as plt
from matplotlib import axes as ax
from matplotlib import rc
from matplotlib import cm
import numpy as np

from pylab import savefig


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


def makeRegulatorPlot (import_file, name):

    module = __import__(import_file, globals(), locals(), ['*'])
    #import_module(import_file)
    #from import_file import regulatorTest
    #from import_file import n_data
    #from import_file import speed_target
    #from import_file import regulatorTest2

    data = []
    speed_current1 = []
    speed_current2 = []
    speed_target_array = []
    torque = []
    time = []

    speed_normalizer = module.regulatorTest[1][0]
    torque_normalizer = module.regulatorTest[1][1]
    time_normalizer = module.regulatorTest[1][2]

    [speed_current1.append( module.regulatorTest[index][0] ) for index in range(1,module.n_data)]
    [speed_current2.append( module.regulatorTest2[index][0] ) for index in range(1,module.n_data)]
    [speed_target_array.append( module.speed_target ) for index in range(1,module.n_data)]
    [torque.append( module.regulatorTest[index][1] ) for index in range(1,module.n_data)]
    [time.append( module.regulatorTest[index][2] ) for index in range(1,module.n_data)]

    gs_top = plt.GridSpec(5, 1, top=0.95)
    fig, (ax1, ax2) = plt.subplots(2,1, sharex=True, facecolor=light_grey_c)#, sharey=True),

    label_ax1 = "Velocidad para Kp = " + module.regulatorTest[0][0] + ", torque_offset = " + torque_offset
    ax1.plot(time, speed_current1, 'r-o', label=label_ax1, lw=2, color=red_c)
    ax1.plot(time, speed_target_array, '-', label=label_ax1, lw=2, color=blue_c)
    ax1.set_title(label_ax1,weight = "bold")

    ax2.plot(time, speed_current2, 'r-o', label=label_ax1, lw=2, color=red_c)
    ax2.plot(time, speed_target_array, '-', label=label_ax1, lw=2, color=blue_c)
    #ax2.set_title("Code lines:",weight = "bold")

    handles_all = []
    labels_all = []
    for ax in ax1, ax2:
        ax.grid(True)
        ax.margins(0.08) # 5% padding in all directions
        #ax.set_facecolor(light_grey_c)
        handles, labels = ax.get_legend_handles_labels()
        handles_all += handles
        labels_all += labels

    ax1.set_ylabel('Speed (in rpm)',weight = "bold")
    ax2.set_ylabel('Speed (in rpm)',weight = "bold")


    fig.autofmt_xdate()
    fig.subplots_adjust(left=0.13, bottom=0.11, right=0.93, top=0.92, wspace=0.15, hspace=0.25)

    #plt.show()
    img_name = "data_files/" + name
    img_name.replace(" ","")
    img_name += ".png"
    fig.savefig(img_name, bbox_inches='tight')



torque_offset = "70"
makeRegulatorPlot("data_files.regulator_list(kp=16,66)(toffset=70)", "Datos regulador, Kp = 16,66, t_offset = 70")
makeRegulatorPlot("data_files.regulator_list(kp=25)(toffset=70)", "Datos regulador, Kp = 25, t_offset = 70")
makeRegulatorPlot("data_files.regulator_list(kp=100)(toffset=70)", "Datos regulador, Kp = 100, t_offset = 70")
torque_offset = "100"
makeRegulatorPlot("data_files.regulator_list(kp=100)(toffset=100)", "Datos regulador, Kp = 100, t_offset = 100")
torque_offset = "150"
makeRegulatorPlot("data_files.regulator_list(kp=100)(toffset=100)", "Datos regulador, Kp = 100, t_offset = 150")
makeRegulatorPlot("data_files.regulator_list(kp=150)(toffset=100)", "Datos regulador, Kp = 150, t_offset = 150")
