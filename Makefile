

CC=gcc

CFLAGS=-pthread -Wall

all:
	$(CC) $(CFLAGS) -o dgt3000 dgt3000.c
	$(CC) $(CFLAGS) -shared -fPIC -o dgt3000.so dgt3000.c
	
debug:
	$(CC) $(CFLAGS) -DDEBUG=1 -o dgt3000 dgt3000.c
	$(CC) $(CFLAGS) -DDEBUG=1 -shared -fPIC -o dgt3000.so dgt3000.c
