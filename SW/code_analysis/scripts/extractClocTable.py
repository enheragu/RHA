#!/usr/bin/python

import xml.sax
import sys, getopt
import os

class ClocHandler( xml.sax.ContentHandler ):
    def __init__(self, inputfile):
        self.min_comment_name = str(0)
        self.max_comment_name = str(0)
        self.name = str(0)
        self.date = str(0)
        self.returnDateInput(inputfile)
        self.num_lcomment = int(0)
        self.num_lcode = int(0)
        self.num_lblank = int(0)
        self.percentage_lcomment = float(0.0)
        self.whole_percentage_lcomment = float(0.0)
        self.average_whole_percentage_lcomment = float(0.0)
        self.max_whole_percentage_lcomment = float(0.0)
        self.min_whole_percentage_lcomment = float(100.0)
        self.whole_num_lcode = float(0.0)
        self.whole_num_lcomment = float(0.0)
        self.whole_num_lblank = float(0.0)
        self.n_evaluated = int(0)

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
            #print "Name:", filename
            blank = int(attributes["blank"])
            #print "blank:", blank
            self.num_lblank = blank
            comment = int(attributes["comment"])
            #print "comment:", comment
            self.num_lcomment = comment
            code = int(attributes["code"])
            #print "code:", code
            self.num_lcode = code

            self.percentage_lcomment = (float(comment) * 100 / float(code+comment))
            #print "Percentage of comment:", self.percentage_lcomment

            if self.percentage_lcomment < 100:
                #print "Makeing average, previous value: ", self.average_whole_percentage_lcomment
                self.average_whole_percentage_lcomment =\
                (self.average_whole_percentage_lcomment*self.n_evaluated + \
                self.percentage_lcomment) / (self.n_evaluated+1)
                #print "Makeing average, actual value: ", self.average_whole_percentage_lcomment
                #print "numer", self.n_evaluated

            if self.percentage_lcomment < 100:
                self.whole_num_lcode = self.whole_num_lcode - self.num_lcode
                self.whole_num_lcomment = self.whole_num_lcomment - self.num_lcomment

            if self.percentage_lcomment < 100 and  self.percentage_lcomment > self.max_whole_percentage_lcomment:
                self.max_whole_percentage_lcomment = self.percentage_lcomment
                self.max_comment_name = self.name

            elif self.percentage_lcomment < 100 and  self.percentage_lcomment < self.min_whole_percentage_lcomment:
                self.min_whole_percentage_lcomment = self.percentage_lcomment
                self.min_comment_name = self.name
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
   Handler = ClocHandler(inputfile)
   parser.setContentHandler( Handler )

   #parser.parse("/home/gmv/test_parsing/code_analysis/xml/cloc_2017_08_23.xml")
   parser.parse(inputfile)

   #Handler.printWholeResult()
   Handler.printListComponent(n_files_processed,file_name_arg)

if __name__ == "__main__":
   main(sys.argv[1:])
###########################################################################################################
