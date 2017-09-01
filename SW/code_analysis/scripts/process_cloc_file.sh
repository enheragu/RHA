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
      python extractClocTable.py -i temp_${file##*/} -n $n
      echo -e "${BLUE}extractClocTable script finished${NC}"
      ((n++))
  done
  echo "[0]]" >> $CURRENT_DIR/cloc_list.py
  sed -i "1in_$2 = $n" $CURRENT_DIR/cloc_list.py
  echo -e "${GREEN}End loop for parsing files${NC}"
}
############################# Function ###############################
############################# Function ###############################
parseDebugFiles()
{
  n="0"
  echo -e "${GREEN}Start loop for parsing files${NC}"
  echo -n "$2 = [" >> $CURRENT_DIR/cloc_list.py
  for file in $1*; do
      echo -e -n "${BLUE}Processing file:${NC}"
      echo $file
      echo -e -n "${BLUE}Calling extractDebugTable script to analyze ${NC}"
      python extractDebugTable.py -i ${file} >> $CURRENT_DIR/cloc_list.py
      python extractDebugTable.py -i ${file}
      echo -e "${BLUE}extractDebugTable script finished${NC}"
      ((n++))
  done
  echo "[0]]" >> $CURRENT_DIR/cloc_list.py
  sed -i "1in_$2 = $n" $CURRENT_DIR/cloc_list.py
  echo -e "${GREEN}End loop for parsing files${NC}"
}
############################# Function ###############################





rm $CURRENT_DIR/cloc_list.py
echo "" >> $CURRENT_DIR/cloc_list.py
sed -i '1i# file = [  0 ,         1        ,        2        ,          3         ,            4            ,           5           ,        6        ,           7           ,        8        ]' $CURRENT_DIR/cloc_list.py
sed -i '1i# file = [date, whole_blank_lines, whole_code_lines, whole_comment_lines, whole_comment_percentage, max_comment_percentage, max_comment_name, min_comment_percentage, min_comment_name]' $CURRENT_DIR/cloc_list.py
sed -i '1i#List for Cloc data'  $CURRENT_DIR/cloc_list.py

echo -e "${GREEN}Search all $CURRENT_DIR/../xml/cloc_* files${NC}"
#invoque parseFIles function
parseClocFiles $CURRENT_DIR/../xml/cloc_[0-9] cloc
echo -e "${GREEN}Search all $CURRENT_DIR/../xml/cloc_test_* files${NC}"
#invoque parseFIles function
parseClocFiles $CURRENT_DIR/../xml/cloc_test_ cloc_test


echo " " >>  $CURRENT_DIR/cloc_list.py
echo "#List for Debug data" >>  $CURRENT_DIR/cloc_list.py
echo "# file = [date, debug_total, debug_max, debug_max_name, debug_min, debug_min_name]" >> $CURRENT_DIR/cloc_list.py
echo "# file = [  0 ,      1     ,     2    ,       3       ,    4     ,       5       ]" >> $CURRENT_DIR/cloc_list.py

echo -e "${GREEN}Search all $CURRENT_DIR/..debug_count/code_* files${NC}"
parseDebugFiles $CURRENT_DIR/../debug_count/code_[0-9] debug_code
echo -e "${GREEN}Search all $CURRENT_DIR/..debug_count/test_* files${NC}"
parseDebugFiles $CURRENT_DIR/../debug_count/test_[0-9] debug_test


sed -i '1i#!/usr/bin/python' $CURRENT_DIR/cloc_list.py

rm $CURRENT_DIR/*.py~
rm $CURRENT_DIR/*.sh~
rm $CURRENT_DIR/temp_*.xml


echo -e "${GREEN}New list created for cloc information:${NC}"
more cloc_list.py

echo -e "${GREEN}Creating plot:${NC}"
python3 drawPlot.py
