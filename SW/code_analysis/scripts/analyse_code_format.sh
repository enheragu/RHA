#!/bin/bash 

# needs cpplint.py script

(cd $HOME/cpplint/ && ./cpplint.py --extensions=cpp --headers=h --linelength=120 --counting=detailed --filter=-build/include,-build/header_guard,-legal/copyright,-whitespace/line_length,-readability/casting,-runtime/references,-runtime/arrays,-readability/braces,-whitespace/newline  /home/quique/Documentos/RHA/SW/lib/chuck_handler/chuck_handler.h /home/quique/Documentos/RHA/SW/lib/cytron_g15_servo/* /home/quique/Documentos/RHA/SW/lib/debug/* /home/quique/Documentos/RHA/SW/lib/joint_handler/* /home/quique/Documentos/RHA/SW/lib/joint_rha/* /home/quique/Documentos/RHA/SW/lib/rha_types/* /home/quique/Documentos/RHA/SW/lib/robot_rha/* /home/quique/Documentos/RHA/SW/lib/servo_rha/* /home/quique/Documentos/RHA/SW/lib/utilities/* /home/quique/Documentos/RHA/SW/src/* /home/quique/Documentos/RHA/SW/test/a_test_rha_types/* /home/quique/Documentos/RHA/SW/test/b_test_pid_regulator/* /home/quique/Documentos/RHA/SW/test/c_test_fuzzy_regulator/* /home/quique/Documentos/RHA/SW/test/d_test_servo_rha/* /home/quique/Documentos/RHA/SW/test/e_test_joint_rha/* /home/quique/Documentos/RHA/SW/test/f_test_joint_handler_mock/* )