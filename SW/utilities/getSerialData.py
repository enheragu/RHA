# @Author: Enrique Heredia Aguado <enheragu>
# @Date:   12-Sep-2017
# @Project: RHA
# @Last modified by:   quique
# @Last modified time: 30-Sep-2017


import serial

ser = serial.Serial('/dev/ttyACM0', 921600)
file_object = open('log_execution.py', "w") #('step_test_0_load.py', "w")
print "Serial conection established, file opened"

while True:
     try:
         line = ser.readline()
     except serial.SerialException:
         print "Error with serial, closing file"
         file_object.close()

     file_object.write(line)
