# dgtpi
DGTPi python module

Dgt3000 module

to compile use:
$ make

to compile with debug info use
$ make debug

to compile with lost of debug info use
$ make debug2



the library dgt3000.so can be used as described in dgt3000.h



the application dgt3000 can be used in three ways:

To display a message:
$ sudo ./dgt3000 "a message"
you can add a beep and icons/dots:
$ sudo ./dgt3000 "a message" 1 31 15

To run a clock:
$ sudo ./dgt3000 r 0 10 0 0 10 0
you can run Left and Right up and down with L,R,l and r

to run some tests:
$ sudo ./dgt3000
lever will pause, off button wil stop te app


