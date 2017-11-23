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
lizard --languages cpp --exclude "$CURRENT_DIR/test/*"  $CURRENT_DIR  > $CURRENT_DIR/code_analysis/txt/ca_$1.txt
echo -n "."
#Generate complexity of code in xml
lizard --languages cpp --exclude "$CURRENT_DIR/test/*"  $CURRENT_DIR --xml > $CURRENT_DIR/code_analysis/xml/ca_$1.xml
echo "."
#Generate complexity of code in csv
lizard --languages cpp --exclude "$CURRENT_DIR/test/*"  $CURRENT_DIR --csv > $CURRENT_DIR/code_analysis/csv/ca_$1.csv
echo "Done."

###################################################################
echo "Generating complexity of test code:"
echo -n "Working."
#Generate complexity of test in txt
lizard --languages cpp --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*" --exclude "$CURRENT_DIR/RHAspberry/*" $CURRENT_DIR > $CURRENT_DIR/code_analysis/txt/ca_test_$1.txt
echo -n "."
#Generate complexity of test in xml
lizard --languages cpp --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*" --exclude "$CURRENT_DIR/RHAspberry/*" $CURRENT_DIR --xml > $CURRENT_DIR/code_analysis/xml/ca_test_$1.xml
echo "."
#Generate complexity of test in csv
lizard --languages cpp --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*" --exclude "$CURRENT_DIR/RHAspberry/*" $CURRENT_DIR --csv > $CURRENT_DIR/code_analysis/csv/ca_test_$1.csv
echo "Done."

###################################################################

#echo "Generating Tag Cloud for test code in html"
#lizard --languages cpp -EWordCount --exclude "$CURRENT_DIR/test/*" $CURRENT_DIR > $CURRENT_DIR/delete.txt
#sleep 3
#mv $CURRENT_DIR/codecloud.html $CURRENT_DIR/code_analysis/code_cloud/code_codecloud_$1.html
#echo "Done."
#echo "Generating Tag Cloud for test code in html"
#lizard --languages cpp -EWordCount --exclude "$CURRENT_DIR/lib/*" --exclude "$CURRENT_DIR/src/*" $CURRENT_DIR > $CURRENT_DIR/delete.txt
#sleep 3
#mv $CURRENT_DIR/codecloud.html $CURRENT_DIR/code_analysis/code_cloud/test_codecloud_$1.html
#echo "Done."
#rm $CURRENT_DIR/delete.txt

###################################################################
echo "Generating cloc of code:"
echo -n "Working."
#generate Cloc in txt
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)'  $CURRENT_DIR/lib/*  $CURRENT_DIR/src/* $CURRENT_DIR/RHAspberry/* > code_analysis/txt/cloc_$1.txt
echo -n "."
#generate Cloc in xml
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --xml   $CURRENT_DIR/lib/*  $CURRENT_DIR/src/* $CURRENT_DIR/RHAspberry/* > code_analysis/xml/cloc_$1.xml
echo "."
#generate Cloc in csv
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --csv   $CURRENT_DIR/lib/*  $CURRENT_DIR/src/* $CURRENT_DIR/RHAspberry/* > code_analysis/csv/cloc_$1.csv
echo "Done."

echo "Counting Debug calls in code:"
grep -c -r "Debug*" lib/* src/* > code_analysis/debug_count/code_$1.txt
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
cloc --by-file --skip-archive='(txt|csv|xml|py|zip|tar(.(gz|Z|bz2|xz|7z))?)' --csv $CURRENT_DIR/test/* > code_analysis/csv/cloc_test_$1.csv
echo "Done."

echo "Counting Debug calls in test code:"
grep -c -r "Debug*" test/* > code_analysis/debug_count/test_$1.txt
echo "Done."

###################################################################
echo -n "Code analysis generated succesfully for "
echo -n $1
echo " date."


rm *.py~
rm *.sh~
