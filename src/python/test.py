#!/usr/bin/env python
#

import argparse


parser = argparse.ArgumentParser(description='Post weather observations to ThingSpeak website')
parser.add_argument('--filename', default="/dev/ttyACM0", help='file from which to read input')

args = parser.parse_args()
print args.filename


# open the input file and process each NMEA sentence read

with open(args.filename, 'r') as infile:
    while (True):
        line = infile.readline()
        if (line is None):
            break
        print(line)


