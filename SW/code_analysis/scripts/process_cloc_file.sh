#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

GREEN='\033[1;32m'
BLUE='\033[1;34m'
RED='\033[1;31m'
NC='\033[0m' # No Color

############################# Function ###############################
parseClocFiles(){
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
      echo -e -n "${BLUE}Calling drawPlotTable script to analyze ${NC}"
      echo "temp_${file##*/}"
      python extractClocTable.py -i temp_${file##*/} -n $n -f $2 >> $CURRENT_DIR/cloc_list.py
      python extractClocTable.py -i temp_${file##*/} -n $n
      echo -e "${BLUE}drawPlotTable script finished${NC}"
      ((n++))
  done
  echo "[0]]" >> $CURRENT_DIR/cloc_list.py
  sed -i "1in_$2 = $n" $CURRENT_DIR/cloc_list.py
  echo -e "${GREEN}End loop for parsing files${NC}"
}
############################# Function ###############################



rm $CURRENT_DIR/cloc_list.py
echo "" >> $CURRENT_DIR/cloc_list.py
sed -i '1i# file[numer] = [  0 ,         1        ,        2        ,          3         ,            4            ,           5           ,        6        ,           7           ,        8        ]' $CURRENT_DIR/cloc_list.py
sed -i '1i# file[numer] = [date, whole_blank_lines, whole_code_lines, whole_comment_lines, whole_comment_percentage, max_comment_percentage, max_comment_name, min_comment_percentage, min_comment_name]' $CURRENT_DIR/cloc_list.py
sed -i '1i#List for Cloc data'  $CURRENT_DIR/cloc_list.py

echo -e "${GREEN}Search all $CURRENT_DIR/code_analysis/xml/cloc_* files${NC}"
#invoque parseFIles function
parseClocFiles $CURRENT_DIR/code_analysis/xml/cloc_[0-9] cloc
echo -e "${GREEN}Search all $CURRENT_DIR/code_analysis/xml/cloc_test_* files${NC}"
#invoque parseFIles function
parseClocFiles $CURRENT_DIR/code_analysis/xml/cloc_test_ cloc_test

sed -i '1i#!/usr/bin/python' $CURRENT_DIR/cloc_list.py

rm $CURRENT_DIR/*.py~
rm $CURRENT_DIR/*.sh~
rm $CURRENT_DIR/temp_*.xml


echo -e "${GREEN}New list created for cloc information:${NC}"
more cloc_list.py
