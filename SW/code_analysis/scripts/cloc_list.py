#!/usr/bin/python
n_cloc_test = 45
n_cloc = 45

#List for Cloc data
# file = [date (index 0), whole_blank_lines (index 1), whole_code_lines (index 2), whole_comment_lines (index 3), whole_debug_lines (index 4), whole_comment_percentage (index 5), max_comment_percentage (index 6), max_comment_name (index 7), min_comment_percentage (index 8), min_comment_name (index 9), whole_debug_percentage (index 10), max_debug_percentage (index 11), max_debug_name (index 12), min_debug_percentage (index 13), min_debug_name (index 14)]
cloc = [['2017_08_23', '209', '831', '232', '38', '21.8250235183', '92.8571428571', 'joint_rha.cpp', '12.6984126984', 'servo_rha.h', '2.3602484472', '16.5217391304', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_24', '218', '867', '241', '38', '21.7509025271', '92.8571428571', 'joint_rha.cpp', '11.7647058824', 'servo_rha.h', '2.01058201058', '14.0740740741', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_25', '227', '893', '253', '38', '22.0767888307', '92.8571428571', 'joint_rha.cpp', '8.77192982456', 'servo_rha.h', '1.75925925926', '14.0740740741', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_26', '237', '882', '250', '39', '22.0848056537', '38.0952380952', 'joint_rha.cpp', '6.25', 'joint_rha.h', '1.6142384106', '12.9139072848', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_27', '237', '882', '257', '40', '22.5636523266', '38.0952380952', 'joint_rha.cpp', '6.25', 'joint_rha.h', '1.61812297735', '12.9449838188', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_28', '243', '836', '230', '28', '21.5759849906', '37.9807692308', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.34615384615', '13.4615384615', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_29', '276', '918', '287', '28', '23.8174273859', '40.2255639098', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_30', '288', '996', '288', '28', '22.4299065421', '40.2255639098', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_08_31', '288', '996', '288', '28', '22.4299065421', '40.2255639098', 'servo_rha.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_07', '291', '967', '340', '28', '26.0137719969', '48.5074626866', 'main.cpp', '0.0', 'servo_rha.h', '1.05263157895', '10.5263157895', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_08', '305', '1057', '470', '58', '30.7793058284', '58.0', 'joint_rha.cpp', '18.4549356223', 'cytron_g15_servo.cpp', '2.77422422858', '20.4081632653', 'utilities.h', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_09', '316', '1203', '507', '89', '29.649122807', '69.2307692308', 'main.cpp', '15.6156156156', 'utilities.h', '2.97004402268', '17.7177177177', 'utilities.h', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_11', '314', '1203', '507', '89', '29.649122807', '69.2307692308', 'main.cpp', '15.6156156156', 'utilities.h', '2.97004402268', '17.7177177177', 'utilities.h', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_12', '323', '1282', '507', '30', '28.3398546674', '67.4418604651', 'main.cpp', '15.5405405405', 'utilities.h', '1.30975094106', '9.75609756098', 'servo_rha.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_13', '364', '1411', '541', '103', '27.7151639344', '70.4545454545', 'main.cpp', '15.3209109731', 'utilities.cpp', '2.42076408956', '14.6997929607', 'utilities.cpp', '0.0', 'cytron_g15_servo.cpp'] ,\
['2017_09_14', '253', '1022', '407', '103', '28.4814555633', '70.4545454545', 'main.cpp', '15.3209109731', 'utilities.cpp', '2.90491690747', '14.6997929607', 'utilities.cpp', '0.0', 'joint_handler.cpp'] ,\
['2017_09_15', '258', '1132', '427', '105', '27.3893521488', '70.4545454545', 'main.cpp', '15.3209109731', 'utilities.cpp', '3.18563876735', '14.6997929607', 'utilities.cpp', '0.0', 'joint_handler.h'] ,\
['2017_09_16', '267', '1153', '426', '102', '26.9791006966', '70.4545454545', 'main.cpp', '15.4958677686', 'utilities.cpp', '2.90386640643', '14.6694214876', 'utilities.cpp', '0.0', 'joint_handler.h'] ,\
['2017_09_17', '276', '1244', '445', '102', '26.3469508585', '70.4545454545', 'main.cpp', '15.4958677686', 'utilities.cpp', '2.83120460806', '14.6694214876', 'utilities.cpp', '0.0', 'joint_handler.h'] ,\
['2017_09_19', '289', '1241', '388', '115', '23.8182934316', '54.5454545455', 'joint_rha.cpp', '14.1843971631', 'utilities.h', '3.41242072855', '14.6694214876', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_20', '295', '1298', '443', '126', '25.4451464675', '49.3150684932', 'joint_rha.cpp', '14.1843971631', 'utilities.h', '4.33754182656', '14.6694214876', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_21', '329', '1276', '436', '94', '25.4672897196', '49.3150684932', 'joint_rha.cpp', '7.60869565217', 'rha_types.h', '4.12057624515', '15.2284263959', 'utilities.h', '0.0', 'servo_rha.h'] ,\
['2017_09_22', '327', '1279', '437', '95', '25.4662004662', '45.5696202532', 'joint_rha.cpp', '7.52688172043', 'rha_types.h', '4.08233507849', '15.2284263959', 'utilities.h', '0.0', 'servo_rha.h'] ,\
['2017_09_23', '305', '1138', '445', '94', '28.1111813013', '61.6666666667', 'main.cpp', '7.52688172043', 'rha_types.h', '3.92640624896', '18.2692307692', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_24', '303', '1139', '452', '95', '28.409805154', '68.8524590164', 'main.cpp', '7.52688172043', 'rha_types.h', '3.92585179466', '18.1818181818', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_25', '303', '1139', '614', '95', '35.0256702795', '68.8524590164', 'main.cpp', '18.3006535948', 'servo_rha.h', '3.55047742698', '18.1818181818', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_26', '324', '1220', '644', '95', '34.5493562232', '58.8235294118', 'main.cpp', '17.9104477612', 'utilities.cpp', '3.18092275451', '14.1791044776', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_28', '328', '1228', '650', '96', '34.6112886049', '60.2941176471', 'main.cpp', '17.6895306859', 'utilities.cpp', '3.16404504442', '14.0794223827', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_29', '327', '1223', '666', '96', '35.256749603', '60.2941176471', 'main.cpp', '16.7664670659', 'servo_rha.h', '2.91229335002', '14.1818181818', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_09_30', '327', '1222', '673', '97', '35.5145118734', '60.2941176471', 'main.cpp', '16.7664670659', 'servo_rha.h', '2.90184553962', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_10_31', '335', '1245', '688', '97', '35.5923435075', '59.1549295775', 'main.cpp', '16.7664670659', 'servo_rha.h', '2.67428310804', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_11_01', '404', '1445', '773', '101', '34.8512173129', '61.0526315789', 'main.cpp', '0.0', 'robot_rha.cpp', '2.34365904212', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_11_02', '421', '1507', '773', '101', '33.9035087719', '61.0526315789', 'main.cpp', '0.0', 'chuck_handler.h', '2.19718035198', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_11_03', '420', '1513', '820', '101', '35.1478782683', '61.0526315789', 'main.cpp', '16.7664670659', 'servo_rha.h', '2.13870081982', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_11_07', '423', '1523', '863', '110', '36.1693210394', '80.5084745763', 'main.cpp', '16.7664670659', 'servo_rha.h', '3.0450933744', '14.5161290323', 'robot_rha.cpp', '0.0', 'servo_rha.h'] ,\
['2017_11_21', '424', '1534', '865', '110', '36.0566902876', '77.2357723577', 'main.cpp', '16.5680473373', 'servo_rha.h', '2.96504119224', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_12_21', '427', '1533', '879', '110', '36.4427860697', '70.8955223881', 'main.cpp', '16.5680473373', 'servo_rha.h', '2.96504119224', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_12_22', '451', '1594', '918', '110', '36.5445859873', '70.8955223881', 'main.cpp', '16.5680473373', 'servo_rha.h', '2.73158313643', '13.986013986', 'utilities.cpp', '0.0', 'servo_rha.h'] ,\
['2017_12_23', '453', '1640', '881', '78', '34.9464498215', '69.9248120301', 'main.cpp', '16.5680473373', 'servo_rha.h', '2.13336765706', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
['2017_12_25', '453', '1639', '884', '78', '35.037653587', '69.9248120301', 'main.cpp', '16.5680473373', 'servo_rha.h', '2.13336765706', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
['2017_12_26', '443', '1636', '856', '84', '34.3499197432', '60.9756097561', 'main_utilities.cpp', '16.4705882353', 'servo_rha.h', '2.21003887151', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
['2017_12_27', '443', '1634', '863', '84', '34.5614737685', '60.2409638554', 'main_utilities.cpp', '16.4705882353', 'servo_rha.h', '2.20692150905', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
['2018_01_01', '470', '1767', '873', '84', '33.0681818182', '59.649122807', 'joint_rha.cpp', '15.6327543424', 'utilities.cpp', '2.20537704072', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
['2018_01_07', '470', '1778', '875', '84', '32.981530343', '59.649122807', 'joint_rha.cpp', '15.3846153846', 'utilities.cpp', '2.20537704072', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
['2018_01_18', '481', '1792', '912', '84', '33.7278106509', '71.5384615385', 'main.cpp', '15.3846153846', 'utilities.cpp', '2.17771373005', '14.4927536232', 'robot_rha.cpp', '0.0', 'utilities.cpp'] ,\
[0]]

cloc_test = [['2017_08_23', '69', '353', '43', '17', '10.8585858586', '16.7597765363', 'test_cytron_g15_servo.cpp', '5.8064516129', 'test_servo_mock.cpp', '6.06355499489', '12.9032258065', 'test_servo_real.cpp', '1.93548387097', 'test_servo_mock.cpp'] ,\
['2017_08_24', '83', '454', '45', '17', '9.01803607214', '16.7597765363', 'test_cytron_g15_servo.cpp', '3.84615384615', 'test_servo_real.cpp', '3.43647624272', '5.12820512821', 'test_servo_real.cpp', '1.82926829268', 'test_servo_mock.cpp'] ,\
['2017_08_25', '81', '436', '45', '17', '9.35550935551', '17.3410404624', 'test_cytron_g15_servo.cpp', '4.0', 'test_servo_real.cpp', '3.56675853434', '5.33333333333', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_08_26', '102', '525', '54', '38', '9.32642487047', '17.4418604651', 'test_cytron_g15_servo.cpp', '5.73248407643', 'test_servo_mock.cpp', '5.9664000395', '12.5', 'test_servo_real.cpp', '1.91082802548', 'test_servo_mock.cpp'] ,\
['2017_08_27', '95', '511', '55', '38', '9.71731448763', '17.4418604651', 'test_cytron_g15_servo.cpp', '6.32911392405', 'test_servo_real.cpp', '5.87849567944', '12.2362869198', 'test_servo_real.cpp', '1.91082802548', 'test_servo_mock.cpp'] ,\
['2017_08_28', '82', '424', '48', '20', '10.1694915254', '18.2926829268', 'test_cytron_g15_servo.cpp', '5.29801324503', 'test_servo_real.cpp', '4.28471094092', '7.28476821192', 'test_servo_real.cpp', '1.91082802548', 'test_servo_mock.cpp'] ,\
['2017_08_29', '79', '393', '48', '20', '10.8843537415', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_08_30', '79', '393', '48', '20', '10.8843537415', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_08_31', '79', '393', '48', '20', '10.8843537415', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_09_07', '79', '393', '48', '20', '10.8843537415', '18.2926829268', 'test_cytron_g15_servo.cpp', '6.45161290323', 'test_servo_real.cpp', '4.83009621368', '8.87096774194', 'test_servo_real.cpp', '1.96078431373', 'test_servo_mock.cpp'] ,\
['2017_09_08', '87', '393', '72', '20', '15.4838709677', '22.0930232558', 'test_cytron_g15_servo.cpp', '11.1801242236', 'test_servo_mock.cpp', '4.56168648787', '8.33333333333', 'test_servo_real.cpp', '1.86335403727', 'test_servo_mock.cpp'] ,\
['2017_09_09', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_11', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_12', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_13', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_14', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_15', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_16', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_17', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_19', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_20', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_21', '87', '382', '73', '20', '16.043956044', '22.4852071006', 'test_cytron_g15_servo.cpp', '12.0253164557', 'test_servo_mock.cpp', '4.6809266784', '8.59375', 'test_servo_real.cpp', '1.89873417722', 'test_servo_mock.cpp'] ,\
['2017_09_22', '96', '274', '62', '2', '18.4523809524', '29.0697674419', 'test_joint_rha.cpp', '12.8834355828', 'test_servo_rha.cpp', '0.766283524904', '2.29885057471', 'test_joint_handler.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_23', '112', '344', '64', '10', '15.6862745098', '26.5957446809', 'test_joint_rha.cpp', '11.1111111111', 'test_servo_rha.cpp', '3.06006050687', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_24', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_25', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_26', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_28', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_29', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_09_30', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_10_31', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_11_01', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_11_02', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_11_03', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_11_07', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_11_21', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_12_21', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_12_22', '130', '389', '72', '10', '15.6182212581', '26.5957446809', 'test_joint_rha.cpp', '12.5683060109', 'test_joint_handler.cpp', '2.85625702438', '6.3829787234', 'test_joint_rha.cpp', '0.0', 'test_servo_rha.cpp'] ,\
['2017_12_23', '189', '571', '116', '10', '16.885007278', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.665259798749', '2.45901639344', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
['2017_12_25', '190', '541', '159', '10', '22.7142857143', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.647077478532', '2.34375', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
['2017_12_26', '190', '541', '159', '10', '22.7142857143', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.647077478532', '2.34375', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
['2017_12_27', '191', '543', '159', '10', '22.6495726496', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.566192793716', '2.34375', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
['2018_01_01', '191', '543', '159', '10', '22.6495726496', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.566192793716', '2.34375', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
['2018_01_07', '191', '543', '159', '10', '22.6495726496', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.566192793716', '2.34375', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
['2018_01_18', '191', '543', '159', '10', '22.6495726496', '39.3939393939', 'test_fuzzy_regulator.cpp', '0.0', 'perform_test.sh', '0.566192793716', '2.34375', 'test_servo_rha.cpp', '0.0', 'test_pid_regulator.cpp'] ,\
[0]]

