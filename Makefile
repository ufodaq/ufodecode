CC     = gcc
CFLAGS = -g -std=c99

all: ipedec 

ipedec: ipedec.o
	$(CC) $(LDFLAGS) -o $@ $^

ipedec.o: ipedec.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f ipedec ipedec.o

.PHONY: clean

