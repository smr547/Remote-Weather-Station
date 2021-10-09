TARGET=arduino:avr:mega
DEV=/dev/ttyACM0
BAUD=115200

all:	compile

compile:
	arduino-cli compile --fqbn $(TARGET) .

upload:
	arduino-cli upload -p $(DEV) --fqbn $(TARGET) .
	stty -F $(DEV) $(BAUD) cs8 -cstopb -parenb
	cat $(DEV)
