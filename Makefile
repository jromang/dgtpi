

CC=gcc

CFLAGS=-pthread -Wall

all:
	$(CC) $(CFLAGS) -o dgt3000 dgt3000.c
	$(CC) $(CFLAGS) -shared -fPIC -o dgt3000.so dgt3000.c
	
debug:
	$(CC) $(CFLAGS) -Ddebug -o dgt3000 dgt3000.c
	$(CC) $(CFLAGS) -Ddebug -shared -fPIC -o dgt3000.so dgt3000.c

debug2:
	$(CC) $(CFLAGS) -Ddebug -Ddebug2 -o dgt3000 dgt3000.c
	$(CC) $(CFLAGS) -Ddebug -Ddebug2 -shared -fPIC -o dgt3000.so dgt3000.c
