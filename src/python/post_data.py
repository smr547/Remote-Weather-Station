#!/usr/bin/env python
#

import requests
import fileinput
import os
import argparse
import serial


parser = argparse.ArgumentParser(description='Post weather observations to ThingSpeak website')
parser.add_argument('--filename', default="/dev/ttyACM0", help='file from which to read input')

args = parser.parse_args()


# get the API key

home_dir = os.environ['HOME']
with open(home_dir+"/.thingSpeak", 'r') as f:
    api_key = f.readline().rstrip()
params = {"key": api_key}



h = 577.0 # altitude in metres
url = "https://api.thingspeak.com/update.json"


# open the input file and process each NMEA sentence read

with serial.Serial(args.filename) as infile:
    while True:
        line = infile.readline()
        if (len(line) == 0):
            continue
        try :
       
	    if (line is None):
                break

            line = line.rstrip()
            print(line)
            values = line.split(',')


            params["field1"] = values[4]       #  temp
            params["field2"] = values[3]       #  humidity %
            params["field3"] = values[6]       #  dew point degC

            # compute MSL pressure

            t = float(values[4])               # temp deg_C
            p = float(values[2])/100.0         # pressure at site (hPa)

            p_msl = p * pow(1.0 - ((0.0065 * h)/(t + (0.0065 * h) + 273.15)), -5.257)

            params["field4"] = str(round(p_msl, 2))   # msl pressure
            params["field5"] = values[8]              # wind speed
            params["field6"] = values[10]             # wind gust (knots)
            params["field7"] = values[9]              # wind direction
            params["field8"] = values[7]              # rainfall


            r = requests.post(url=url, params=params)
            print(r.json()) 

        except Exception as e:
            print("Oops!", e.__class__, ", (", e.message, ") occurred.")
