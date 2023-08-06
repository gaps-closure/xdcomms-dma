#!/usr/bin/env python3
import argparse
import matplotlib.pyplot as plt
import sys
import csv
import os
from pprint import pprint

# Plot app_req_rep performance: (Auguest, 2023) from csv file input
# Frist line in csv has time; second line has column legends:
COL_L = 5   # Length (in Bytes) per request packet column
COL_M = 4   # Messages per second column
COL_B = 6   # Bytes per second column


# Read results from input file (CSV format) into 'rows' and 'fields' lists
def read_values(filename_prefix):
  rows = []
  filename_list = [filename for filename in os.listdir() if filename.startswith(filename_prefix)]
  if args.verbose: print('processing:', filename_list)
  for input_filename in sorted(filename_list):
    with open(input_filename, 'r') as csvfile:
      csvreader = csv.reader(csvfile)  # creating a csv reader object
      title = next(csvreader)          # extract plot title
      fields = next(csvreader)         # extract field names from second row
      rows += [row for row in csvreader if len(row)]     # non-empty rows only
    csvfile.close()
  if args.verbose: print('Fields:', fields)
  return(title, fields, rows)

# Initialize plot
def plot_init(col_x, col_y):
  plt.clf()               # allow multiple plots by clearing old one
  plt.grid(True)
  plt.xlabel(fields[col_x])
  plt.ylabel(fields[col_y])
  plt.xscale("log")
#  plt.ticklabel_format(useMathText=True, style='sci', axis='x', scilimits=(0,0))

def plot_xy(col_x, col_y):
  plot_init(col_x, col_y)
  y_array = [float(row[col_y]) for row in rows]
  x_array = [  int(row[col_x]) for row in rows]
  if args.verbose: print (x_array, y_array)
  plt.plot(x_array, y_array, linestyle='-', marker='o')  #, label=unique_value_col_z
  plt.ylim(bottom=0)
  plt.xlim(left=int(rows[0][col_x]))
  plt.title(title[0])
  plt.savefig("fig1.png");
  if args.display: plt.show()

if __name__=='__main__':
  parser = argparse.ArgumentParser(description='Plot memory copy throughput against copy size')
  parser.add_argument('-d', '--display',          help='Display results', action='store_true')
  parser.add_argument('-i', '--in_file_prefixes', help='Input filename prefixes', type=str, default='results_')
  parser.add_argument('-v', '--verbose',          help='Print debug', action='store_true')
  args = parser.parse_args()
  title, fields, rows = read_values(args.in_file_prefixes)
  print(rows[0])
#  plot_xy(COL_L, COL_M)  # x=data length, y=Msgs/sec
  plot_xy(COL_L, COL_B)  # x=data length,  y=Bytes/sec
