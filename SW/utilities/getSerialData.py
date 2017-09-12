# @Author: Enrique Heredia Aguado <enheragu>
# @Date:   12-Sep-2017
# @Project: RHA
# @Last modified by:   enheragu
# @Last modified time: 12-Sep-2017


import serial

ser = serial.Serial('/dev/ttyACM0', 9600)
file_object = open('regulator_list.py', "w")
print "Serial conection established, file opened"

while True:
     try:
         line = ser.readline()
     except serial.SerialException:
         print "Error with serial, closing file"
         file_object.close()

     file_object.write(line)
