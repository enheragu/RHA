#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

GREEN='\033[1;32m'
BLUE='\033[1;34m'
RED='\033[1;31m'
NC='\033[0m' # No Color

############################# Function ###############################
parseClocFiles()
{
  n="0"
  echo -e "${GREEN}Start loop for parsing files${NC}"
  echo -n "$2 = [" >> $CURRENT_DIR/cloc_list.py
  for file in $1*; do
      #[ -f "$i" ] || break
      echo -e -n "${BLUE}Processing file:${NC}"
      echo $file
      echo -e -n "${BLUE}Now called:${NC}"
      echo ${file##*/}
      sed '1,4d' $file > $CURRENT_DIR/temp_${file##*/}
      echo -e "${BLUE}Lines 1 to 4 deleted${NC}"
      echo -e -n "${BLUE}Calling extractClocTable script to analyze ${NC}"
      echo "temp_${file##*/}"
      python extractClocTable.py -i temp_${file##*/} -n $n -f $2 >> $CURRENT_DIR/cloc_list.py
      python extractClocTable.py -i temp_${file##*/} -n $n -f $2
      RC=$?
      if [ $RC != 0 ]
      then
          echo -e "${RED}Exit code $RC  ${NC}"
          #alias print_date="echo ${file##*/} | grep -Eo '[[:digit:]]{4}_[[:digit:]]{2}_[[:digit:]]{2}'"
          num=$( echo "${file##*/}" | grep -Eo '[[:digit:]]{4}_[[:digit:]]{2}_[[:digit:]]{2}' )
          #num = $print_date
          echo "['$num','0','0','0','0','0','0','0','0','0','0','0','0','0'],\\" >> $CURRENT_DIR/cloc_list.py

          echo "['$num','0','0','0','0','0','0','0','0','0','0','0','0','0'],\\"
      fi
      echo -e "${BLUE}extractClocTable script finished${NC}"
      ((n++))
  done
  echo "[0]]" >> $CURRENT_DIR/cloc_list.py
  echo "" >> $CURRENT_DIR/cloc_list.py
  sed -i "1in_$2 = $n" $CURRENT_DIR/cloc_list.py
  echo -e "${GREEN}End loop for parsing files${NC}"
}
############################# Function ###############################
############################# Function ###############################
parseDebugFiles()
{
  n="0"
  echo -e "${GREEN}Start loop for parsing files${NC}"
  echo -n "$2 = [" >> $CURRENT_DIR/debug_list.py
  for file in $1*; do
      echo -e -n "${BLUE}Processing file:${NC}"
      echo $file
      echo -e -n "${BLUE}Calling extractDebugTable script to analyze ${NC}"
      python extractDebugTable.py -i ${file} >> $CURRENT_DIR/debug_list.py
      python extractDebugTable.py -i ${file}
      echo -e "${BLUE}extractDebugTable script finished${NC}"
      ((n++))
  done
  echo "[0]]" >> $CURRENT_DIR/debug_list.py
  echo "" >> $CURRENT_DIR/debug_list.py
  sed -i "1in_$2 = $n" $CURRENT_DIR/debug_list.py
  echo -e "${GREEN}End loop for parsing files${NC}"
}
############################# Function ###############################





rm $CURRENT_DIR/cloc_list.py
rm $CURRENT_DIR/debug_list.py

echo " " >>  $CURRENT_DIR/debug_list.py
echo "#List for Debug data" >>  $CURRENT_DIR/debug_list.py
echo "# file = [date, debug_total, debug_max, debug_max_name, debug_min, debug_min_name]" >> $CURRENT_DIR/debug_list.py
echo "# file = [  0 ,      1     ,     2    ,       3       ,    4     ,       5       ]" >> $CURRENT_DIR/debug_list.py

echo -e "${GREEN}Search all $CURRENT_DIR/..debug_count/code_* files${NC}"
parseDebugFiles $CURRENT_DIR/../debug_count/code_[0-9] debug_code
echo -e "${GREEN}Search all $CURRENT_DIR/..debug_count/test_* files${NC}"
parseDebugFiles $CURRENT_DIR/../debug_count/test_[0-9] debug_test


echo "" >> $CURRENT_DIR/cloc_list.py
echo "#List for Cloc data" >>  $CURRENT_DIR/cloc_list.py
echo "# file = [date (index 0), whole_blank_lines (index 1), whole_code_lines (index 2), whole_comment_lines (index 3), whole_debug_lines (index 4), whole_comment_percentage (index 5), max_comment_percentage (index 6), max_comment_name (index 7), min_comment_percentage (index 8), min_comment_name (index 9), whole_debug_percentage (index 10), max_debug_percentage (index 11), max_debug_name (index 12), min_debug_percentage (index 13), min_debug_name (index 14)]" >> $CURRENT_DIR/cloc_list.py


echo -e "${GREEN}Search all $CURRENT_DIR/../xml/cloc_* files${NC}"
#invoque parseFIles function
parseClocFiles $CURRENT_DIR/../xml/cloc_[0-9] cloc
echo -e "${GREEN}Search all $CURRENT_DIR/../xml/cloc_test_* files${NC}"
#invoque parseFIles function
parseClocFiles $CURRENT_DIR/../xml/cloc_test_ cloc_test


sed -i '1i#!/usr/bin/python' $CURRENT_DIR/cloc_list.py

rm $CURRENT_DIR/*.py~
rm $CURRENT_DIR/*.sh~
rm $CURRENT_DIR/temp_*.xml
rm $CURRENT_DIR/*.pyc


echo -e "${GREEN}New list created for cloc information:${NC}"
#more cloc_list.py

echo -e "${GREEN}Creating plot:${NC}"
python3 drawPlot.py
