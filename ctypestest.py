#!/usr/bin/python3
import ctypes
import time
import sys

lib = ctypes.cdll.LoadLibrary("/home/pi/20151124/dgt3000.so")

#but=c_char()
#time=c_char()
but=ctypes.c_byte(0)
buttime=ctypes.c_byte(0)
run=1

res = lib.dgt3000Init()
if(res < 0):
	sys.exit(res)
res = lib.dgt3000Configure()
if(res < 0):
	sys.exit(res)
	
while(run):
	if(lib.dgt3000GetButton(ctypes.pointer(but),ctypes.pointer(buttime))==1):
		if (but.value==1):
			run=0
		else:
			print(but,buttime)
	lib.dgt3000Display(b"Hello World",0,0,0)
	print(lib.dgt3000GetButtonState())
	time.sleep(0.1)
print(but,buttime)
print(lib.dgt3000Off(5))
lib.dgt3000Stop()
