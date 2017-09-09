#!/usr/bin/python
n_cloc_test = 12
n_cloc = 12

#List for Cloc data
# file = [date (index 0), whole_blank_lines (index 1), whole_code_lines (index 2), whole_comment_lines (index 3), whole_debug_lines (index 4), whole_comment_percentage (index 5), max_comment_percentage (index 6), max_comment_name (index 7), min_comment_percentage (index 8), min_comment_name (index 9), whole_debug_percentage (index 10), max_debug_percentage (index 11), max_debug_name (index 12), min_debug_percentage (index 13), min_debug_name (index 14)]
cloc = [['2017_08_23', '209', '831', '232', '38', '27.9181708785', '92.8571428571', 'joint_rha.cpp', '12.6984126984', 'servo_rha.h', '2.3602484472', '16.5217391304', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_24', '218', '867', '241', '38', '27.7970011534', '92.8571428571', 'joint_rha.cpp', '11.7647058824', 'servo_rha.h', '2.01058201058', '14.0740740741', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_25', '227', '893', '253', '38', '28.3314669653', '92.8571428571', 'joint_rha.cpp', '8.77192982456', 'servo_rha.h', '1.75925925926', '14.0740740741', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_26', '237', '882', '250', '39', '28.3446712018', '38.0952380952', 'joint_rha.cpp', '6.25', 'joint_rha.h', '1.6142384106', '12.9139072848', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_27', '237', '882', '257', '40', '29.1383219955', '38.0952380952', 'joint_rha.cpp', '6.25', 'joint_rha.h', '1.61812297735', '12.9449838188', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_28', '243', '836', '230', '28', '27.5119617225', '37.9807692308', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.34615384615', '13.4615384615', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_29', '276', '918', '287', '28', '31.2636165577', '40.2255639098', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_30', '288', '996', '288', '28', '28.9156626506', '40.2255639098', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_31', '288', '996', '288', '28', '28.9156626506', '40.2255639098', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_07', '291', '967', '340', '28', '35.1602895553', '48.5074626866', 'main.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_08', '305', '1057', '470', '58', '44.4654683065', '58.0', 'joint_rha.cpp', '18.4549356223', 'cytron_g15_servo.cpp', '2.77422422858', '20.4081632653', 'utilities.h', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_09', '316', '1203', '507', '89', '42.144638404', '69.2307692308', 'main.cpp', '15.6156156156', 'utilities.h', '2.97004402268', '17.7177177177', 'utilities.h', '0.0', 'cytron_g15_servo.cpp'] ,\
[0]]

cloc_test = [['2017_08_23', '69', '353', '43', '17', '12.1813031161', '16.7597765363', 'test_cytron_g15_servo.cpp', '5.8064516129', 'test_servo_mock.cpp', '6.06355499489', '12.9032258065', 'test_servo_real.cpp', '1.93548387097', 'test_servo_mock.cpp'] ,\
['2017_08_24', '83', '454', '45', '17', '9.91189427313', '16.7597765363', 'test_cytron_g15_servo.cpp', '3.84615384615', 'test_servo_real.cpp', '3.43647624272', '5.12820512821', 'test_servo_real.cpp', '1.82926829268', 'test_servo_mock.cpp'] ,\
['2017_08_25', '81', '436', '45', '17', '10.3211009174', '17.3410404624', 'test_cytron_g15_servo.cpp', '4.0', 'test_servo_real.cpp', '3.56675853434', '5.33333333333', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_08_26', '102', '525', '54', '38', '10.2857142857', '17.4418604651', 'test_cytron_g15_servo.cpp', '5.73248407643', 'test_servo_mock.cpp', '5.9664000395', '12.5', 'test_servo_real.cpp', '1.91082802548', 'test_servo_mock.cpp'] ,\
['2017_08_27', '95', '511', '55', '38', '10.7632093933', '17.4418604651', 'test_cytron_g15_servo.cpp', '6.32911392405', 'test_servo_real.cpp', '5.87849567944', '12.2362869198', 'test_servo_real.cpp', '1.91082802548', 'test_servo_mock.cpp'] ,\
['2017_08_28', '82', '424', '48', '20', '11.320754717', '18.2926829268', 'test_cytron_g15_servo.cpp', '5.29801324503', 'test_servo_real.cpp', '4.28471094092', '7.28476821192', 'test_servo_real.cpp', '1.91082802548', 'test_servo_mock.cpp'] ,\
['2017_08_29', '79', '393', '48', '20', '12.213740458', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_08_30', '79', '393', '48', '20', '12.213740458', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_08_31', '79', '393', '48', '20', '12.213740458', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_09_07', '79', '393', '48', '20', '12.213740458', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_09_08', '87', '393', '72', '20', '18.320610687', '22.0930232558', 'test_cytron_g15_servo.cpp', '11.1801242236', 'test_servo_mock.cpp', '4.56168648787', '8.33333333333', 'test_servo_real.cpp', '1.86335403727', 'test_servo_mock.cpp'] ,\
['2017_09_09', '87', '382', '73', '20', '19.109947644', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
[0]]

