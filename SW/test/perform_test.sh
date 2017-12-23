#!/bin/bash

NOW=$(date +'%F_%R')
(cd .. && pio test -v )#>test/log/log_test_${NOW}.log 2>&1)
