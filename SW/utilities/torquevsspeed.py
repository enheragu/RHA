# @Author: Enrique Heredia Aguado <quique>
# @Date:   28-Sep-2017
# @Project: RHA
# @Last modified by:   quique
# @Last modified time: 28-Sep-2017



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
    speed_current2 = []
    speed_current3 = []
    speed_current4 = []
    speed_current5 = []
    speed_target_array = []
    torque = []
    time0 = []
    time1 = []
    time2 = []
    time3 = []
    time4 = []
    time5 = []

    time_normalizer0 = int(module.regulatorTest0[1][1])
    time_normalizer1 = int(module.regulatorTest1[1][1])
    time_normalizer2 = int(module.regulatorTest2[1][1])
    time_normalizer3 = int(module.regulatorTest3[1][1])
    time_normalizer4 = int(module.regulatorTest4[1][1])
    time_normalizer5 = int(module.regulatorTest5[1][1])

    [speed_current0.append( module.regulatorTest0[index][0] ) for index in range(1,module.n_data0)]
    [speed_current1.append( module.regulatorTest1[index][0] ) for index in range(1,module.n_data0)]
    [speed_current2.append( module.regulatorTest2[index][0] ) for index in range(1,module.n_data0)]
    [speed_current3.append( module.regulatorTest3[index][0] ) for index in range(1,module.n_data0)]
    [speed_current4.append( module.regulatorTest4[index][0] ) for index in range(1,module.n_data0)]
    [speed_current5.append( module.regulatorTest5[index][0] ) for index in range(1,module.n_data0)]
    [speed_target_array.append( module.speed_target0 ) for index in range(1,module.n_data0)]
    [time0.append( str(int(module.regulatorTest0[index][2])) ) for index in range(1,module.n_data0)]
    [time1.append( str(int(module.regulatorTest1[index][2])) ) for index in range(1,module.n_data0)]
    [time2.append( str(int(module.regulatorTest2[index][2])) ) for index in range(1,module.n_data0)]
    [time3.append( str(int(module.regulatorTest3[index][2])) ) for index in range(1,module.n_data0)]
    [time4.append( str(int(module.regulatorTest4[index][2])) ) for index in range(1,module.n_data0)]
    [time5.append( str(int(module.regulatorTest5[index][2])) ) for index in range(1,module.n_data0)]

    gs_top = plt.GridSpec(5, 1, top=0.95)
    fig, ((ax0, ax3), (ax1, ax4), (ax2, ax5)) = plt.subplots(3,2, sharex=True, facecolor=light_grey_c)

    label_ax0 = "Velocidad para Kp = " + module.regulatorTest0[0][0] + ", Ki = " + module.regulatorTest0[0][1] + ", Kd = " + module.regulatorTest0[0][2] + ", regulator_offset = " + str(module.regulator_offset0)
    ax0.plot(time0, speed_current0, 'r-o', label=label_ax0, lw=2, color=red_c)
    ax0.plot(time0, speed_target_array, '-', label=label_ax0, lw=2, color=blue_c)
    ax0.set_title(label_ax0,weight = "bold")

    label_ax1 = "Velocidad para Kp = " + module.regulatorTest1[0][0] + ", Ki = " + module.regulatorTest1[0][1] + ", Kd = " + module.regulatorTest1[0][2] + ", regulator_offset = " + str(module.regulator_offset1)
    ax1.plot(time1, speed_current1, 'r-o', label=label_ax1, lw=2, color=red_c)
    ax1.plot(time1, speed_target_array, '-', label=label_ax1, lw=2, color=blue_c)
    ax1.set_title(label_ax1,weight = "bold")

    label_ax2 = "Velocidad para Kp = " + module.regulatorTest2[0][0] + ", Ki = " + module.regulatorTest2[0][1] + ", Kd = " + module.regulatorTest2[0][2] + ", regulator_offset = " + str(module.regulator_offset2)
    ax2.plot(time2, speed_current2, 'r-o', label=label_ax2, lw=2, color=red_c)
    ax2.plot(time2, speed_target_array, '-', label=label_ax2, lw=2, color=blue_c)
    ax2.set_title(label_ax2,weight = "bold")

    label_ax3 = "Velocidad para Kp = " + module.regulatorTest3[0][0] + ", Ki = " + module.regulatorTest3[0][1] + ", Kd = " + module.regulatorTest3[0][2] + ", regulator_offset = " + str(module.regulator_offset3)
    ax3.plot(time3, speed_current3, 'r-o', label=label_ax3, lw=2, color=red_c)
    ax3.plot(time3, speed_target_array, '-', label=label_ax3, lw=2, color=blue_c)
    ax3.set_title(label_ax3,weight = "bold")


    label_ax4 = "Velocidad para Kp = " + module.regulatorTest4[0][0] + ", Ki = " + module.regulatorTest4[0][1] + ", Kd = " + module.regulatorTest4[0][2] + ", regulator_offset = " + str(module.regulator_offset4)
    ax4.plot(time4, speed_current4, 'r-o', label=label_ax4, lw=2, color=red_c)
    ax4.plot(time4, speed_target_array, '-', label=label_ax4, lw=2, color=blue_c)
    ax4.set_title(label_ax4,weight = "bold")


    label_ax5 = "Velocidad para Kp = " + module.regulatorTest5[0][0] + ", Ki = " + module.regulatorTest5[0][1] + ", Kd = " + module.regulatorTest5[0][2] + ", regulator_offset = " + str(module.regulator_offset5)
    ax5.plot(time5, speed_current5, 'r-o', label=label_ax5, lw=2, color=red_c)
    ax5.plot(time5, speed_target_array, '-', label=label_ax5, lw=2, color=blue_c)
    ax5.set_title(label_ax5,weight = "bold")


    handles_all = []
    labels_all = []
    for ax in ax0, ax1, ax2, ax3, ax4, ax5:
        ax.grid(True)
        ax.margins(0.08) # 5% padding in all directions
        #ax.set_facecolor(light_grey_c)
        handles, labels = ax.get_legend_handles_labels()
        handles_all += handles
        labels_all += labels

    ax0.set_ylabel('Speed (in rpm)',weight = "bold")
    ax1.set_ylabel('Speed (in rpm)',weight = "bold")
    ax2.set_ylabel('Speed (in rpm)',weight = "bold")

    ax2.set_xlabel('Time (in ms)',weight = "bold")
    ax5.set_xlabel('Time (in ms)',weight = "bold")



    fig.autofmt_xdate()
    fig.subplots_adjust(left=0.06, bottom=0.06, right=0.93, top=0.94, wspace=0.10, hspace=0.25)

    #plt.show()
    img_name = "data_files/" + name
    img_name.replace(" ","")
    img_name += ".png"
    fig.set_size_inches(18.5, 10.5)
    fig.savefig(img_name, bbox_inches='tight')



makeRegulatorPlot("regulator_data_test_15", "Graficos Regulador. Test 15")
