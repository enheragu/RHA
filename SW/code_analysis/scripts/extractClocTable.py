#!/usr/bin/python

import xml.sax
import sys, getopt
import os

from debug_list import debug_code
from debug_list import debug_test

class ClocHandler( xml.sax.ContentHandler ):
    def __init__(self, inputfile, debug_list):
        self.min_comment_name = str(0)
        self.max_comment_name = str(0)
        self.min_debug_name = str(0)
        self.max_debug_name = str(0)
        self.name = str(0)
        self.date = str(0)
        self.returnDateInput(inputfile)
        self.num_lcomment = int(0)
        self.num_lcode = int(0)
        self.num_lblank = int(0)
        self.num_ldebug = int(0)
        self.percentage_lcomment = float(0.0)
        self.percentage_ldebug = float(0.0)
        self.whole_percentage_lcomment = float(0.0)
        self.whole_percentage_ldebug = float(0.0)
        self.average_whole_percentage_lcomment = float(0.0)
        self.average_whole_percentage_ldebug = float(0.0)
        self.max_whole_percentage_lcomment = float(0.0)
        self.min_whole_percentage_lcomment = float(100.0)
        self.max_whole_percentage_ldebug = float(0.0)
        self.min_whole_percentage_ldebug = float(100.0)
        self.whole_num_lcode = float(0.0)
        self.whole_num_lcomment = float(0.0)
        self.whole_num_lblank = float(0.0)
        self.whole_num_ldebug = float(0.0)
        self.n_evaluated = int(0)
        self.final_list = []
        self.debuglist = debug_list

    # Call when an element starts
    def startElement(self, tag, attributes):
        self.CurrentData = tag
        if tag == "total":
            self.whole_num_lblank = int(attributes["blank"])
            self.whole_num_lcode = int(attributes["code"])
            self.whole_num_lcomment = int(attributes["comment"])
            self.whole_percentage_lcomment = float(self.whole_num_lcomment*100) / float(self.whole_num_lcode)

        if tag == "file":
            #print "*****File*****"
            filename = attributes["name"]
            #if filename == "output_export.cpp":
            #    return;
            self.name = str(attributes["name"])
            self.name = os.path.basename(self.name)

            for element in self.debuglist:
                if (self.name in element[0]):
                    self.num_ldebug = float(element[1])
                    #print ("Element name is: ") + element[0]
                    #print ("Element debug code lines is: ") + str(self.num_ldebug)

            self.whole_num_ldebug += int(self.num_ldebug)

            blank = int(attributes["blank"])
            self.num_lblank = blank
            comment = int(attributes["comment"])
            self.num_lcomment = comment
            code = int(attributes["code"])
            self.num_lcode = code

            self.percentage_lcomment = (float(comment) * 100 / float(code+comment))
            self.percentage_ldebug = (float(self.num_ldebug) * 100 / float(code+comment))
            #print "Percentage of comment:", self.percentage_lcomment

            #################
            # comment_lines #
            #################
            if self.percentage_lcomment < 100:
                #print "Makeing average, previous value: ", self.average_whole_percentage_lcomment
                self.average_whole_percentage_lcomment =\
                (self.average_whole_percentage_lcomment*self.n_evaluated + \
                self.percentage_lcomment) / (self.n_evaluated+1)
                #print "Makeing average, actual value: ", self.average_whole_percentage_lcomment
                #print "numer", self.n_evaluated

            if self.percentage_lcomment > 100:
                self.whole_num_lcode = self.whole_num_lcode - self.num_lcode
                self.whole_num_lcomment = self.whole_num_lcomment - self.num_lcomment

            if self.percentage_lcomment < 100 and  self.percentage_lcomment > self.max_whole_percentage_lcomment:
                self.max_whole_percentage_lcomment = self.percentage_lcomment
                self.max_comment_name = self.name

            if self.percentage_lcomment < self.min_whole_percentage_lcomment:
                self.min_whole_percentage_lcomment = self.percentage_lcomment
                self.min_comment_name = self.name

            ###############
            # debug_lines #
            ###############

            if self.percentage_ldebug < 100:
                #print "Makeing average, previous value: ", self.average_whole_percentage_ldebug
                self.average_whole_percentage_ldebug =\
                (self.average_whole_percentage_ldebug*self.n_evaluated + \
                self.percentage_ldebug) / (self.n_evaluated+1)
                #print "Makeing average, actual value: ", self.average_whole_percentage_ldebug
                #print "numer", self.n_evaluated

            if self.percentage_ldebug > 100:
                self.whole_num_lcode = self.whole_num_lcode - self.num_lcode
                self.whole_num_ldebug = self.whole_num_ldebug - self.num_ldebug

            if self.percentage_ldebug < 100 and  self.percentage_ldebug > self.max_whole_percentage_ldebug:
                self.max_whole_percentage_ldebug = self.percentage_ldebug
                self.max_debug_name = self.name

            if self.percentage_ldebug < self.min_whole_percentage_ldebug:
                self.min_whole_percentage_ldebug = self.percentage_ldebug
                self.min_debug_name = self.name
            #language = attributes["language"]
            #print "language:", language
            self.n_evaluated += 1

    def printWholeResult(self):
        print"========================================="
        print"Date of this file:", self.date
        print"Whole black lines:", self.whole_num_lblank
        print"Whole code lines:", self.whole_num_lcode
        print"Whole comment lines:", self.whole_num_lcomment
        print"Whole comment percentage:", self.whole_percentage_lcomment
        print"Whole max comment percentage:", self.max_whole_percentage_lcomment
        print"Whole min comment percentage:", self.min_whole_percentage_lcomment
        #print"Whole comment percentage average:", self.average_whole_percentage_lcomment
        print"Number of files evaluated:", self.n_evaluated
        print"========================================="

    def printListComponent(self,num,file_name):
        print "["+\
                "\'"+self.date+"\'"+","+\
                "\'"+str(self.whole_num_lblank)+"\'"+","+\
                "\'"+str(self.whole_num_lcode)+"\'"+","+\
                "\'"+str(self.whole_num_lcomment)+"\'"+","+\
                "\'"+str(self.whole_percentage_lcomment)+"\'"+","+\
                "\'"+str(self.max_whole_percentage_lcomment)+"\'"+","+\
                "\'"+str(self.max_comment_name)+"\'"+","+\
                "\'"+str(self.min_whole_percentage_lcomment)+"\'"+","+\
                "\'"+str(self.min_comment_name)+"\'"+"],"+str('\\')
                #self.average_whole_percentage_lcomment+"]"

    def setlist(self):
        self.final_list.append(str(self.date))
        self.final_list.append(str(self.whole_num_lblank))
        self.final_list.append(str(self.whole_num_lcode))
        self.final_list.append(str(self.whole_num_lcomment))
        self.final_list.append(str(int(self.whole_num_ldebug)))
        #comment
        self.final_list.append(str(self.whole_percentage_lcomment))
        self.final_list.append(str(self.max_whole_percentage_lcomment))
        self.final_list.append(str(self.max_comment_name))
        self.final_list.append(str(self.min_whole_percentage_lcomment))
        self.final_list.append(str(self.min_comment_name))
        #debug
        self.final_list.append(str(self.average_whole_percentage_ldebug))
        self.final_list.append(str(self.max_whole_percentage_ldebug))
        self.final_list.append(str(self.max_debug_name))
        self.final_list.append(str(self.min_whole_percentage_ldebug))
        self.final_list.append(str(self.min_debug_name))

        print (self.final_list),
        print str(',\\')


    def returnWholeRestult(self):
        return [self.n_evaluated,\
        self.date,\
        self.whole_num_lblank,\
        self.whole_num_lcode,\
        self.whole_num_lcomment,\
        self.whole_percentage_lcomment,\
        self.max_whole_percentage_lcomment,\
        self.min_whole_percentage_lcomment,\
        #self.average_whole_percentage_lcomment,\
        ]

    def returnDateInput(self, a):
        index_init = a.find("_201")
        index_end = a.find(".xml")
        self.date = a[index_init+1 : index_end]


def main(argv):
   inputfile = ''
   outputfile = ''
   n_files_processed = 0
   file_name_arg = ''
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
      #elif opt in ("-o", "--ofile"):
          #outputfile = arg
   #print 'Input file is "', inputfile
   #print 'Output file is "', outputfile

   # create an XMLReader
   parser = xml.sax.make_parser()
   # turn off namepsaces
   parser.setFeature(xml.sax.handler.feature_namespaces, 0)

   # override the default ContextHandler
   index_init = inputfile.find("_201")
   index_end = inputfile.find(".xml")
   date = inputfile[index_init+1 : index_end]
   if (file_name_arg == "cloc"):
       list_to_parse = debug_code
   elif (file_name_arg == "cloc_test"):
       list_to_parse = debug_test

   debug_list = []
   for element in list_to_parse:
       if date in element:
           debug_list = element
           #print (debug_list)
           #print ("element 0 is:")
           #print (debug_list[0])
   #debug_list = [element for element in debug_code if date in element[0]]

   Handler = ClocHandler(inputfile, debug_list)
   parser.setContentHandler( Handler )
   try:
       #parser.parse("/home/gmv/test_parsing/code_analysis/xml/cloc_2017_08_23.xml")
       parser.parse(inputfile)
   except getopt.GetoptError:
       print 'Error parsing'
       sys.exit(2)

   #Handler.printWholeResult()
   #Handler.printListComponent(n_files_processed,file_name_arg)
   Handler.setlist()

if __name__ == "__main__":
   main(sys.argv[1:])
###########################################################################################################
