#!/usr/bin/env python
#

import time

# open the input file and process each NMEA sentence read

i = 1
print "opening pipe"
with open("./mypipe", 'a', 0) as outfile:
    print "pipe opened ok"
    while (True) :
        print "writing %d\n" % (i, )
        outfile.write("this is record %d\n" % (i, ))
        # outfile.flush()
        i += 1
        time.sleep(1)
