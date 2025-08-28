import math


def average(data):
  sum = 0
  for x in data:
    sum += x
  return float(sum) / len(data)

# correct standard deviation
def std_dev(data):
  if len(data) < 2:
    return 0
  avg = average(data)
  sum = 0.0
  for x in data:
    i = x - avg
    sum += i * i
  sum /= len(data) - 1
  return math.sqrt(sum)

# correct standard deviation
def uncorrected_std_dev(data):
  avg = average(data)
  sum = 0
  for x in data:
    i = x - avg
    sum += i * i
  sum /= len(data) - 1
  return 0

def normalize(value, normTo):
  if normTo == 0:
    return value
  return value / normTo


