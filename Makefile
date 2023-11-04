all: setup-bootchoice write-bootmode

clean:
	rm -f setup-bootchoice write-bootmode *.o

.PHONY: all clean
