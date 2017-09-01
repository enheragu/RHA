#!/usr/bin/python

import os
import re
import sys, getopt

inputfile = ''
outputfile = ''
n_files_processed = 0
file_name_arg = ''

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"hi:o:n:f:",["ifile=","ofile=","number=","file_name="])
    except getopt.GetoptError:
        print 'Error with args for extractClocTable.py'
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-i", "--ifile"):
            inputfile = arg
        if opt in ("-n", "--number"):
            n_files_processed = arg
        if opt in ("-f", "--file_name"):
            file_name_arg = arg

    results = []
    index_init = inputfile.find("_201")
    index_end = inputfile.find(".txt")
    date = inputfile[index_init+1 : index_end]
    results.append(date)

    debug_in_files = []
    with open(inputfile) as inputfile:
        for line in inputfile:
            debug_in_files.append(line.strip().split(':'))

    for element in debug_in_files:
        element[0] = os.path.basename(element[0])

    result_list = [element for element in debug_in_files if ".gch" not in element[0]]
    result_list = [element for element in debug_in_files if "debug.h" not in element[0]]

    debug_total = 0
    debug_max = 0
    debug_max_name = ''
    debug_min = 1000
    debug_min_name = ''

    #print result_list
    for element in result_list:
        debug_total = debug_total + int(element[1])
        if int(element[1]) > debug_max:
            debug_max = element[1]
            debug_max_name = element[0]
        if int(element[1]) < debug_min:
            debug_min = element[1]
            debug_min_name = element[0]

    results.append(str(debug_total))
    results.append(debug_max)
    results.append(debug_max_name)
    results.append(debug_min)
    results.append(debug_min_name)


    #print ("Results:")
    #print (results)
    #print ("Result list without gch:")
    print (results),
    print str(',\\')

if __name__ == "__main__":
   main(sys.argv[1:])
###########################################################################################################
