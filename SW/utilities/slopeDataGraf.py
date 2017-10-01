# @Author: Enrique Heredia Aguado <enheragu>
# @Date:   12-Sep-2017
# @Project: RHA
# @Last modified by:   quique
# @Last modified time: 30-Sep-2017


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
    torque_target_array1 = []
    torque = []
    time1 = []


    for index_test in range(1,module.n_data_slope0):
        slope_sum = 0
        time_slope_sum = 0
        torque_slope_sum = 0
        for index_all in module.slopeTest:
            slope_sum += int(index_all[index_test][0])
            torque_slope_sum += int(index_all[index_test][1])
            time_slope_sum += (int(index_all[index_test][2]) - int(index_all[1][2]))
        #for index_all in range(0,module.n_samples_slope):
        #    slope_sum += int(module.slopeTest[index_all][0][0])
        #    torque_slope_sum += int(module.slopeTest[index_all][index_test][1])
        #    time_slope_sum += (int(module.slopeTest[index_all][index_test][2]) - int(module.slopeTest[index_all][1][2]))
        speed_current1.append(slope_sum/(int(module.n_samples_slope)))
        time1.append(time_slope_sum/(int(module.n_samples_slope)))
        torque_target_array1.append((int(torque_slope_sum)/(int(module.n_samples_slope))))


    slope_sum = 0.0
    data = 0
    torquevsspeed = 0.0
    torque_start = 0
    for index in range(0,module.n_data_slope0-1):
    #    if (int(speed_current1[index]) > 0):
    #        data = data + 1
    #        slope_sum += (float(torque_target_array1[index])-float(torque_target_array1[index-1]))/(float(speed_current1[index])-float(speed_current1[index-1]))
    #    else:
         if (float(speed_current1[index]) == 0):
            torque_start = torque_target_array1[index]

    #torquevsspeed = slope_sum / data
    torquevsspeed = (float(torque_target_array1[30])-float(torque_target_array1[100]))/(float(speed_current1[30])-float(speed_current1[100]))


    gs_top = plt.GridSpec(5, 1, top=0.95)
    fig, (ax1) = plt.subplots(1,1, sharex=True, facecolor=light_grey_c)

    test = "Slope is " + str(torquevsspeed)
    ax1.text(0, 400, test, fontsize=15)

    test = "Torque start is " + str(torque_start)
    ax1.text(0, 450, test, fontsize=15)

    label_ax1 = "Slope Test (average of 20 tests)"
    ax1.plot(torque_target_array1, speed_current1, 'r-o', label=label_ax1, lw=2, color=red_c)
    #ax1.plot(time1, torque_target_array1, '-', label=label_ax1, lw=2, color=blue_c)
    ax1.set_title(label_ax1,weight = "bold")

    handles_all = []
    labels_all = []

    ax1.grid(True)
    ax1.margins(0.08) # 5% padding in all directions
    #ax.set_facecolor(light_grey_c)
    handles, labels = ax1.get_legend_handles_labels()
    handles_all += handles
    labels_all += labels

    ax1.set_ylabel('Speed (in rpm)',weight = "bold")

    ax1.set_xlabel('Torque',weight = "bold")

    fig.autofmt_xdate()
    fig.subplots_adjust(left=0.06, bottom=0.06, right=0.93, top=0.94, wspace=0.10, hspace=0.25)

    #plt.show()
    img_name = "data_files/" + name
    img_name.replace(" ","")
    img_name += ".png"
    fig.set_size_inches(18.5, 10.5)
    fig.savefig(img_name, bbox_inches='tight')



makeRegulatorPlot("slope_average_data_test_1", "Graficos Slope Average. Test 1")
