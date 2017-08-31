#!/usr/bin/python

# needs matplotlib:
# (problems with python2.7)
# sudo apt-get install python3-matplotlib
# and get dependencies:
# sudo apt-get build-dep python-matplotlib

import datetime as DT


from matplotlib import dates as mpdates
from matplotlib import pyplot as plt
from matplotlib import axes as ax
from matplotlib.dates import date2num

from cloc_list import cloc
from cloc_list import cloc_test
from cloc_list import n_cloc

data = []
code_lines = []
comment_lines = []
comment_percentage_lines = []
comment_percentage_lines_max = []
comment_percentage_lines_min = []

#[print(cloc[x][0]) for x in range(0,5)]

try:
    [data.append( DT.datetime.strptime( cloc[index][0], "%Y_%m_%d" ))  for index in range(0,n_cloc)]
    #[print(cloc[x][2]) for x in range(0,5)]
except Exception as ex:
    print(ex)

[code_lines.append( cloc[index][2] ) for index in range(0,n_cloc)]
[comment_lines.append( cloc[index][3] ) for index in range(0,n_cloc)]
[comment_percentage_lines.append( cloc[index][4] ) for index in range(0,n_cloc)]
[comment_percentage_lines_max.append( cloc[index][5] ) for index in range(0,n_cloc)]
[comment_percentage_lines_min.append( cloc[index][7] ) for index in range(0,n_cloc)]




dates = mpdates.date2num(data)

fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True)

ax1.plot_date(dates, code_lines, 'r-o', label="Code lines", lw=2)
ax2.plot_date(dates, comment_lines, 'g-o', label="Comment lines", lw=2)

ax3.plot_date(dates, comment_percentage_lines, 'b-o', label="Comment percentage lines")
ax3.plot_date(dates, comment_percentage_lines_max, 'b--', label="Comment percentage lines (max)")
ax3.plot_date(dates, comment_percentage_lines_min, 'b--', label="Comment percentage lines (min)")


for ax in ax1, ax2, ax3:
    ax.grid(True)
    ax.margins(0.05) # 5% padding in all directions

ax1.set_ylabel('Number of lines')
ax2.set_ylabel('Number of lines')
ax3.set_ylabel('%')
#for label in ax2.get_yticklabels():
#    label.set_visible(False)

fig.suptitle('Blabla')
fig.autofmt_xdate()
plt.show()
