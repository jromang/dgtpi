

CC=gcc

CFLAGS=-pthread -Wall

all:
	$(CC) $(CFLAGS) -o dgtpicom dgtpicom.c
	$(CC) $(CFLAGS) -shared -fPIC -o dgtpicom.so dgtpicom.c
	
debug:
	$(CC) $(CFLAGS) -Ddebug -o dgtpicom dgtpicom.c
	$(CC) $(CFLAGS) -Ddebug -shared -fPIC -o dgtpicom.so dgtpicom.c

debug2:
	$(CC) $(CFLAGS) -Ddebug -Ddebug2 -o dgtpicom dgtpicom.c
	$(CC) $(CFLAGS) -Ddebug -Ddebug2 -shared -fPIC -o dgtpicom.so dgtpicom.c
