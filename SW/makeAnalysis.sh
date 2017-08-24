#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# needs https://github.com/terryyin/lizard
# cd
# git clone https://github.com/terryyin/lizard.git
# cd lizard
# sudo python setup.py install
# cd $CURRENT_DIR

# needs https://github.com/AlDanial/cloc
# sudo apt-get install cloc


###################################################################
echo "Generating complexity of code:"
echo -n "Working."
#Generate complexity of code in txt
lizard --languages cpp --exclude "$CURRENT_DIR/test/*"  /home/gmv/RHA/SW  > $CURRENT_DIR/code_analysis/txt/ca_$1.txt
echo -n "."
#Generate complexity of code in xml
lizard --languages cpp --exclude "$CURRENT_DIR/test/*"  /home/gmv/RHA/SW --xml > $CURRENT_DIR/code_analysis/xml/ca_$1.xml
echo "."
#Generate complexity of code in csv
lizard --languages cpp --exclude "$CURRENT_DIR/test/*"  /home/gmv/RHA/SW --csv > $CURRENT_DIR/code_analysis/csv/ca_$1.csv
echo "Done."

###################################################################
echo "Generating complexity of test code:"
echo -n "Working."
#Generate complexity of test in txt
lizard --languages cpp --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*"  $CURRENT_DIR > $CURRENT_DIR/code_analysis/txt/ca_test_$1.txt
echo -n "."
#Generate complexity of test in xml
lizard --languages cpp --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*"  $CURRENT_DIR --xml > $CURRENT_DIR/code_analysis/xml/ca_test_$1.xml
echo "."
#Generate complexity of test in csv
lizard --languages cpp --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*"  $CURRENT_DIR --csv > $CURRENT_DIR/code_analysis/csv/ca_test_$1.csv
echo "Done."

###################################################################
echo "Generating cloc of code:"
echo -n "Working."
#generate Cloc in txt
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)'  $CURRENT_DIR/lib/*  $CURRENT_DIR/src/*  > code_analysis/txt/cloc_$1.txt
echo -n "."
#generate Cloc in xml
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --xml   $CURRENT_DIR/lib/*  $CURRENT_DIR/src/*  > code_analysis/xml/cloc_$1.xml
echo "."
#generate Cloc in csv
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --csv   $CURRENT_DIR/lib/*  $CURRENT_DIR/src/*  > code_analysis/csv/cloc_$1.csv
echo "Done."

###################################################################
echo "Generating cloc of test code:"
echo -n "Working."
#generate Cloc of test in txt
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)'  $CURRENT_DIR/test/* > code_analysis/txt/cloc_test_$1.txt
echo -n "."
#generate Cloc of test  in xml
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --xml $CURRENT_DIR/test/* > code_analysis/xml/cloc_test_$1.xml
echo "."
#generate Cloc of test  in csv
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --csv $CURRENT_DIR/test/* > code_analysis/csv/cloc_test_$1.csvecho "Done."

###################################################################
echo -n "Code analysis generated succesfully for "
echo -n $1
echo " date."
