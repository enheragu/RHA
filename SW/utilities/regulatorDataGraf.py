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
    speed_current0 = []
    speed_current1 = []
    speed_current2 = []
    speed_target_array = []
    torque = []
    time0 = []
    time1 = []
    time2 = []
    torque_sent0 = []
    torque_sent1 = []
    torque_sent2 = []
    speedvstorque_current0 = []
    speedvstorque_current1 = []
    speedvstorque_current2 = []

    time_normalizer0 = int(module.regulatorTest0[1][1])
    time_normalizer1 = int(module.regulatorTest1[1][1])
    time_normalizer2 = int(module.regulatorTest2[1][1])

    [speed_current0.append( module.regulatorTest0[index][0] ) for index in range(1,module.n_data0)]
    [speed_current1.append( module.regulatorTest1[index][0] ) for index in range(1,module.n_data0)]
    [speed_current2.append( module.regulatorTest2[index][0] ) for index in range(1,module.n_data0)]
    #for comparison with torque. Torque gets compared against next speed value
    [speedvstorque_current0.append( module.regulatorTest0[index][0] ) for index in range(2,module.n_data0)]
    [speedvstorque_current1.append( module.regulatorTest1[index][0] ) for index in range(2,module.n_data0)]
    [speedvstorque_current2.append( module.regulatorTest2[index][0] ) for index in range(2,module.n_data0)]
    [torque_sent0.append( module.regulatorTest0[index][2] ) for index in range(1,module.n_data0-1)]
    [torque_sent1.append( module.regulatorTest1[index][2] ) for index in range(1,module.n_data0-1)]
    [torque_sent2.append( module.regulatorTest2[index][2] ) for index in range(1,module.n_data0-1)]
    [speed_target_array.append( module.speed_target0 ) for index in range(1,module.n_data0)]
    [time0.append( (int(module.regulatorTest0[index][1])-time_normalizer0) ) for index in range(1,module.n_data0)]
    [time1.append( (int(module.regulatorTest1[index][1])-time_normalizer1) ) for index in range(1,module.n_data0)]
    [time2.append( (int(module.regulatorTest2[index][1])-time_normalizer2) ) for index in range(1,module.n_data0)]


    gs_top = plt.GridSpec(5, 1, top=0.95)
    fig, ((ax0, ax1), (ax2, ax3), (ax4, ax5)) = plt.subplots(3,2, sharex=False, facecolor=light_grey_c)

    label_ax0 = "Velocidad para Kp = " + module.regulatorTest0[0][0] + ", Ki = " + module.regulatorTest0[0][1] + ", Kd = " + module.regulatorTest0[0][2] + ",\n regulator_offset = " + str(module.regulator_offset0) + ", regulator_prealimentation = " + str(float(module.regulator_prealimentation0)) + "*speedTarget"
    ax0.plot(time0, speed_current0, 'r-o', label=label_ax0, lw=2, color=red_c)
    ax0.plot(time0, speed_target_array, '-', label=label_ax0, lw=2, color=blue_c)
    ax0.set_title(label_ax0,weight = "bold")

    label_ax1 = "Torque (N-1) vs speed (N) in same conditions"
    ax1.plot(torque_sent0, speedvstorque_current0, 'ro', label=label_ax1, lw=2, color=red_c)
    #ax1.plot(time0, torque_sent0, '-', label=label_ax1, lw=2, color=blue_c)
    ax1.set_title(label_ax1,weight = "bold")

    label_ax2 = "Velocidad para Kp = " + module.regulatorTest1[0][0] + ", Ki = " + module.regulatorTest1[0][1] + ", Kd = " + module.regulatorTest1[0][2] + ",\n regulator_offset = " + str(module.regulator_offset1) + ", regulator_prealimentation = " + str(module.regulator_prealimentation1) + "*speedTarget"
    ax2.plot(time1, speed_current1, 'r-o', label=label_ax2, lw=2, color=red_c)
    ax2.plot(time1, speed_target_array, '-', label=label_ax2, lw=2, color=blue_c)
    ax2.set_title(label_ax2,weight = "bold")

    label_ax3 = "Torque (N-1) vs speed (N) in same conditions"
    ax3.plot(torque_sent1, speedvstorque_current1, 'ro', label=label_ax3, lw=2, color=red_c)
    #ax3.plot(time1, torque_sent1, '-', label=label_ax3, lw=2, color=blue_c)
    ax3.set_title(label_ax3,weight = "bold")


    label_ax4 = "Velocidad para Kp = " + module.regulatorTest2[0][0] + ", Ki = " + module.regulatorTest2[0][1] + ", Kd = " + module.regulatorTest2[0][2] + ",\n regulator_offset = " + str(module.regulator_offset2) + ", regulator_prealimentation = " + str(module.regulator_prealimentation2) + "*speedTarget"
    ax4.plot(time2, speed_current2, 'r-o', label=label_ax4, lw=2, color=red_c)
    ax4.plot(time2, speed_target_array, '-', label=label_ax4, lw=2, color=blue_c)
    ax4.set_title(label_ax4,weight = "bold")


    label_ax5 = "Torque (N-1) vs speed (N) in same conditions"
    ax5.plot(torque_sent2, speedvstorque_current2, 'ro', label=label_ax5, lw=2, color=red_c)
    #ax5.plot(time2, torque_sent2, '-', label=label_ax5, lw=2, color=blue_c)
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
    ax2.set_ylabel('Speed (in rpm)',weight = "bold")
    ax4.set_ylabel('Speed (in rpm)',weight = "bold")

    ax4.set_xlabel('Time (in ms)',weight = "bold")
    ax5.set_xlabel('Torque (1023 base)',weight = "bold")

    #fig.autofmt_xdate()
    fig.subplots_adjust(left=0.06, bottom=0.06, right=0.93, top=0.94, wspace=0.10, hspace=0.25)

    #plt.show()
    img_name = "data_files/" + name
    img_name.replace(" ","")
    img_name += ".png"
    fig.set_size_inches(18.5, 10.5)
    fig.savefig(img_name, bbox_inches='tight')



makeRegulatorPlot("regulator_data_3_test_13", "GraficosRegulador_SinglePacket_Test_13(2 knifes)")
