TARGET=arduino:avr:uno
DEV=/dev/ttyACM0
BAUD=9600

all:	compile

compile:
	arduino-cli compile --fqbn $(TARGET) .

upload:
	arduino-cli upload -p $(DEV) --fqbn $(TARGET) .
	stty -F $(DEV) $(BAUD) cs8 -cstopb -parenb
	cat $(DEV) &
	echo "use: echo 'text' > $(DEV)"
	echo "then: make end"

end:
	pkill tail

