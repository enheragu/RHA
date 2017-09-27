# @Author: Enrique Heredia Aguado <enheragu>
# @Date:   12-Sep-2017
# @Project: RHA
# @Last modified by:   quique
# @Last modified time: 27-Sep-2017


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
    speed_current0 = []
    speed_current1 = []
    torque_target_array0 = []
    torque_target_array1 = []
    torque = []
    time0 = []
    time1 = []


    for index_test in range(1,module.n_data_step0):
        step_sum = 0
        time_step_sum = 0
        for index_all in range(0,module.n_samples_step):
            step_sum += int(module.stepTest[index_all][index_test][0])
            time_step_sum += (int(module.stepTest[index_all][index_test][2]) - int(module.stepTest[index_all][1][2]))
        speed_current0.append(step_sum/(int(module.n_samples_step)-1))
        time0.append(time_step_sum/(int(module.n_samples_step)-1))


    for index_test in range(1,module.n_data_slope0):
        slope_sum = 0
        time_slope_sum = 0
        torque_slope_sum = 0
        for index_all in range(0,module.n_samples_slope):
            step_sum += int(module.slopeTest[index_all][index_test][0])
            torque_slope_sum += int(module.slopeTest[index_all][index_test][1])
            time_slope_sum += (int(module.slopeTest[index_all][index_test][2]) - int(module.slopeTest[index_all][1][2]))
        speed_current1.append(slope_sum/(int(module.n_samples_slope)-1))
        time1.append(time_slope_sum/(int(module.n_samples_slope)-1))
        torque_target_array1.append((int(time_slope_sum)/(int(module.n_samples_slope)-1))/1.5)


    [torque_target_array0.append( int(module.stepTest[0][index][1])/1.5 ) for index in range(1,module.n_data_step0)]

    gs_top = plt.GridSpec(5, 1, top=0.95)
    fig, (ax0, ax1) = plt.subplots(2,1, facecolor=light_grey_c)

    label_ax0 = "Step test (average of 20 tests)"
    ax0.plot(time0, speed_current0, 'r-o', label=label_ax0, lw=2, color=red_c)
    ax0.plot(time0, torque_target_array0, '-', label=label_ax0, lw=2, color=blue_c)
    ax0.set_title(label_ax0,weight = "bold")

    label_ax1 = "Slope Test (average of 20 tests)"
    ax1.plot(time1, speed_current1, 'r-o', label=label_ax1, lw=2, color=red_c)
    ax1.plot(time1, torque_target_array1, '-', label=label_ax1, lw=2, color=blue_c)
    ax1.set_title(label_ax1,weight = "bold")

    handles_all = []
    labels_all = []
    for ax in ax0, ax1:
        ax.grid(True)
        ax.margins(0.08) # 5% padding in all directions
        #ax.set_facecolor(light_grey_c)
        handles, labels = ax.get_legend_handles_labels()
        handles_all += handles
        labels_all += labels

    ax0.set_ylabel('Speed (in rpm)',weight = "bold")
    ax1.set_ylabel('Speed (in rpm)',weight = "bold")

    ax1.set_xlabel('Time (in ms)',weight = "bold")

    fig.autofmt_xdate()
    fig.subplots_adjust(left=0.06, bottom=0.06, right=0.93, top=0.94, wspace=0.10, hspace=0.25)

    #plt.show()
    img_name = "data_files/" + name
    img_name.replace(" ","")
    img_name += ".png"
    fig.set_size_inches(18.5, 10.5)
    fig.savefig(img_name, bbox_inches='tight')



makeRegulatorPlot("step_slope_average_data_test_1", "Graficos Step_Slope Average. Test 1")
