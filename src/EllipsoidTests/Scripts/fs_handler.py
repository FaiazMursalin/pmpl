import os
import re
import os.path as io

def isDir(path):
  return io.isdir(path)

def isFile(path):
  return io.isfile(path)

def exists(path):
  return io.exists(path)

def get_cwdir():
  return os.getcwd()

def get_cdir():
  dir = get_cwdir()
  dir = dir.split('/')
  return dir[len(dir) - 1]




def get_files(path):
  files = []
  if not isDir(path):
    print "{} must be a directory".format(path)
    return files

  for file in os.listdir(path):
    if file.endswith(".stat"):
      files.append(io.join(path, file))
  return files


def collect_data(file):
  sections = ['Roadmap', 'Collision', 'Clock', 'Other']
  noSections = ['There', 'Map', 'Local']
  validSection = False
  f = open(file, "r")
  results = {}

  # I do not like this method but it will get everything
  name = ""
  value = 0.0
  for line in f:
    words = line.split();
    if not len(words) > 0:
      continue
    if 'Success' in words[0]:
      name = words[0] + ' ' + words[1]
      temp = words[len(words) - 1]
      temp = temp[0:len(temp) - 2]
      value = float(temp)
      results[name] = value
      continue
    if words[0] in noSections:
      validSection = False
      continue
    if words[0] in sections:
      validSection = True
      continue
    if validSection:
      value = float(words[len(words) - 1])
      if len(words) == 2:
        name = words[0]
      else:
        for x in range(0, len(words) - 1):
          if not name == "":
            name += " "
          name += words[x]

      results[name] = value
  return results


def accumulate_data(path):
  files = get_files(path)
  data = {}
  for f in files:
    r = collect_data(f)
    for key in r:
      if not data.has_key(key):
        data[key] = []
      data[key].append(r[key])
  
  return data
