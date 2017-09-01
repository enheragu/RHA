#!/usr/bin/python
n_debug_test = 9
n_debug_code = 9
n_cloc_test = 9
n_cloc = 9
#List for Cloc data
# file = [date, whole_blank_lines, whole_code_lines, whole_comment_lines, whole_comment_percentage, max_comment_percentage, max_comment_name, min_comment_percentage, min_comment_name]
# file = [  0 ,         1        ,        2        ,          3         ,            4            ,           5           ,        6        ,           7           ,        8        ]

cloc = [['2017_08_23','209','831','232','27.9181708785','92.8571428571','joint_rha.cpp','12.6984126984','servo_rha.h'],\
['2017_08_24','218','867','241','27.7970011534','92.8571428571','joint_rha.cpp','11.7647058824','servo_rha.h'],\
['2017_08_25','227','893','253','28.3314669653','92.8571428571','joint_rha.cpp','8.77192982456','servo_rha.h'],\
['2017_08_26','237','882','250','28.3446712018','38.0952380952','joint_rha.cpp','6.25','joint_rha.h'],\
['2017_08_27','237','882','257','29.1383219955','38.0952380952','joint_rha.cpp','6.25','joint_rha.h'],\
['2017_08_28','243','836','230','27.5119617225','37.9807692308','servo_rha.cpp','0.0','servo_rha.h'],\
['2017_08_29','276','918','287','31.2636165577','40.2255639098','servo_rha.cpp','0.0','servo_rha.h'],\
['2017_08_30','288','996','288','28.9156626506','40.2255639098','servo_rha.cpp','0.0','servo_rha.h'],\
['2017_08_31','288','996','288','28.9156626506','40.2255639098','servo_rha.cpp','0.0','servo_rha.h'],\
[0]]
cloc_test = [['2017_08_23','69','353','43','12.1813031161','16.7597765363','test_CYTRON_G15_SERVO.cpp','5.8064516129','test_servo_mock.cpp'],\
['2017_08_24','83','454','45','9.91189427313','16.7597765363','test_CYTRON_G15_SERVO.cpp','3.84615384615','test_servo_real.cpp'],\
['2017_08_25','81','436','45','10.3211009174','17.3410404624','test_CYTRON_G15_SERVO.cpp','4.0','test_servo_real.cpp'],\
['2017_08_26','102','525','54','10.2857142857','17.4418604651','test_CYTRON_G15_SERVO.cpp','5.73248407643','test_servo_mock.cpp'],\
['2017_08_27','95','511','55','10.7632093933','17.4418604651','test_cytron_g15_servo.cpp','100.0','0'],\
['2017_08_28','82','424','48','11.320754717','18.2926829268','test_cytron_g15_servo.cpp','5.29801324503','test_servo_real.cpp'],\
['2017_08_29','79','393','48','12.213740458','18.2926829268','test_cytron_g15_servo.cpp','6.45161290323','test_servo_real.cpp'],\
['2017_08_30','79','393','48','12.213740458','18.2926829268','test_cytron_g15_servo.cpp','6.45161290323','test_servo_real.cpp'],\
['2017_08_31','79','393','48','12.213740458','18.2926829268','test_cytron_g15_servo.cpp','6.45161290323','test_servo_real.cpp'],\
[0]]
 
#List for Debug data
# file = [date, debug_total, debug_max, debug_max_name, debug_min, debug_min_name]
# file = [  0 ,      1     ,     2    ,       3       ,    4     ,       5       ]
debug_code = [['2017_08_23', '38', '38', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_24', '38', '38', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_25', '38', '38', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_26', '39', '39', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_27', '40', '40', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_28', '28', '28', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_29', '28', '28', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_30', '28', '28', 'servo_rha.cpp', '0', 'main.cpp'] ,\
['2017_08_31', '28', '28', 'servo_rha.cpp', '0', 'main.cpp'] ,\
[0]]
debug_test = [['2017_08_23', '17', '6', 'test_CYTRON_G15_SERVO.cpp', '8', 'test_servo_real.cpp'] ,\
['2017_08_24', '17', '6', 'test_CYTRON_G15_SERVO.cpp', '8', 'test_servo_real.cpp'] ,\
['2017_08_25', '17', '6', 'test_CYTRON_G15_SERVO.cpp', '8', 'test_servo_real.cpp'] ,\
['2017_08_26', '38', '6', 'test_cytron_g15_servo.cpp', '29', 'test_servo_real.cpp'] ,\
['2017_08_27', '38', '6', 'test_cytron_g15_servo.cpp', '29', 'test_servo_real.cpp'] ,\
['2017_08_28', '20', '6', 'test_cytron_g15_servo.cpp', '11', 'test_servo_real.cpp'] ,\
['2017_08_29', '20', '6', 'test_cytron_g15_servo.cpp', '11', 'test_servo_real.cpp'] ,\
['2017_08_30', '20', '6', 'test_cytron_g15_servo.cpp', '11', 'test_servo_real.cpp'] ,\
['2017_08_31', '20', '6', 'test_cytron_g15_servo.cpp', '11', 'test_servo_real.cpp'] ,\
[0]]
