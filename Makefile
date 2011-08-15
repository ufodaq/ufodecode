CC     = gcc
CFLAGS = -g -ggdb -std=c99 -Wall -Werror

all: ipedec deinterlace

ipedec: ipedec.o
	$(CC) $(LDFLAGS) -o $@ $^

ipedec.o: ipedec.c
	$(CC) $(CFLAGS) -c -o $@ $<

deinterlace: deinterlace.o
	$(CC) $(LDFLAGS) -o $@ $^

deinterlace.o: deinterlace.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f ipedec ipedec.o deinterlace deinterlace.o

.PHONY: clean

