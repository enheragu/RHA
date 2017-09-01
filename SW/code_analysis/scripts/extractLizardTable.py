#!/usr/bin/python
try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET


inputfile = ''
outputfile = ''
n_files_processed = 0
file_name_arg = ''

def parseXmlFile():
    tree = ET.ElementTree(file=inputfile)
    root = tree.getroot()

    for elem in tree.iter(tag='measure'):
        if elem.text == 'File':
            print ("File measure found")

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

    parseXmlFile()

if __name__ == "__main__":
   main(sys.argv[1:])
###########################################################################################################
