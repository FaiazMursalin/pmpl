#!/usr/bin/python

# This program is to collect data stats file given a directory or a file
# containing the directories to look at.

import sys

import fs_handler as fs
import math_tools as mt


def setup():
  normalized = ""
  if not len(sys.argv) > 1:
    print "python pmplcollect ([Director] | [filename]) [Normalized]" 
    sys.exit(1)
  name = sys.argv[1] 
  
  print len(sys.argv)
  if len(sys.argv) == 3:
    normalized = sys.argv[2]

  dirs = []

  if fs.isFile(name):
    f = open(name, 'r')
    for line in f:
      dirs.append(line)
  else:
    dirs.append(name)
  return (dirs, normalized)

def collect(dirs):
  data = {}
  statData = {}
  for d in dirs:
    data[d] = fs.accumulate_data(d)

  for name, d in data.iteritems():
    lists = {}
    for t, l in d.iteritems():
      avg = mt.average(l)
      std = mt.std_dev(l)
      size = len(l)
      lists[t] = (avg, std, size)
  statData[name] = lists
  return statData

def print_stats(stats):
  for key in stats:
    print "".ljust(62 + 15 + 15 + 15 + 5, '-')
    print "|{0:110s}|".format(key)
    print "".ljust(62 + 15 + 15 + 15 + 5, '-')
    print"|{0:62s}|{1:15s}|{2:15s}|{3:15s}|".format("Statistic", "Avg", "Std Dev", "Num")
    print "".ljust(62 + 15 + 15 + 15 + 5, '-')
    for keys in stats[key]:
      (avg, std, num) = stats[key][keys]
      print"|{0:62s}|{1:15.5f}|{2:15.5f}|{3:15.5f}|".format(keys, avg, std, num)
    print "".ljust(62 + 15 + 15 + 15 + 5, '-')

def main():
  (files, normalized) = setup()
  stats = collect(files)

  print normalized
  
  if not normalized == "":
    if not stats.has_key(normalized):
      print "normalized variable must be a directory name"
      sys.exit(1)

    norm_data = stats[normalized]
    for nkey, ndata in norm_data.iteritems():
      print ndata
      (navg, nstd, nnum) = ndata
      for exp in stats:
        if stats[exp].has_key(nkey):
          t = stats[exp]
          (avg, std, num) = t[nkey]
          t[nkey] = (mt.normalize(avg, navg), mt.normalize(std, nstd), num)
  print stats
  print_stats(stats)


main()
