#!/usr/bin/env python3
#
# Interface with the command line provided by the weather stations
#-----------------------------------------------------------------

from sys import stdin, stdout, exit

if __name__ == '__main__':
    while True:
        stdout.write("Say something > ")
        stdout.flush()
        userinput = stdin.readline()
        if userinput == "\n":
            exit(0)
        userinput = userinput.rstrip("\n")
        stdout.write("you said '" + userinput + "'\n")
        stdout.flush()
