all: setup-bootchoice write-bootmode read-gps

clean:
	rm -f setup-bootchoice write-bootmode read-gps *.o

.PHONY: all clean
