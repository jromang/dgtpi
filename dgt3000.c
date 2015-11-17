/* functions to communicate to a DGT3000 using I2C
 * version 0.5
 *
 * Copyright (C) 2015 DGT
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>

#include "dgt3000.h"

int ww;

void *wl(void *x) {
	while (1)
		if (ww) {
			dgt3000Configure();
			dgt3000Display("Goeindag",0,0,0);
			dgt3000SetNRun(1,0,10,0,2,0,10,0);
			bug.sendTotal++;
		}
	return 0;
}

int main (int argc, char *argv[]) {
	//int e;
	//char message[255];
	//char t[6]
	char but,tim;

	// get direct acces to the perhicels
	if (dgt3000Init()) return -1;


	// configure dgt3000 for mode 25
	dgt3000Configure();

	// 7 arguments -> setnrun
	if (argc==8) {
		unsigned char leftrun, rightrun=0;
		if (argv[1][0]=='l')
			leftrun=1;
		else if (argv[1][0]=='L')
			leftrun=2;
		else if (argv[1][0]=='r')
			rightrun=1;
		else if (argv[1][0]=='R')
			rightrun=2;

		dgt3000SetNRun(leftrun,
				atoi(argv[2]),
				atoi(argv[3]),
				atoi(argv[4]),
				rightrun,
				atoi(argv[5]),
				atoi(argv[6]),
				atoi(argv[7]) );
	} else if (argc>1) {
		unsigned char beep=0, ldots=0, rdots=0;
		if (argc>2)
			beep=atoi(argv[2]);
		if (argc>4) {
			ldots=atoi(argv[3]);
			rdots=atoi(argv[4]);
		}


		// try three times to end and set de display
		dgt3000Display(argv[1],beep,ldots,rdots);
	} else {
		printf("%.3f ",(float)*timer/1000000);
		printf("started\n");
		ww=1;
		pthread_t w;
		pthread_create(&w, NULL, wl, NULL);
		but=tim=0;
		while(1) {


		//	printf("rxMaxBuf=%d  ",bug.rxMaxBuf);

		//	dgt3000GetTime(t);
		//	printf("time=%d:%02d.%02d %d:%02d.%02d\n",t[0],t[1],t[2],t[3],t[4],t[5]);

			if (dgt3000GetButton(&but,&tim)) {
				printf("%.3f ",(float)*timer/1000000);
				printf("button=%02x, time=%d\n",but,tim);
				if (but==0x20) {
					break;
				}
			}

			usleep(10000);
		}
	}

	dgt3000Stop();

	#ifdef debug
	printf("%.3f ",(float)*timer/1000000);
	printf("After %d messages:\n",bug.sendTotal);
	printf("Send failed: display=%d, endDisplay=%d, changeState=%d, setCC=%d, setNRun=%d\n",
				bug.displaySF, bug.endDisplaySF, bug.changeStateSF, bug.setCCSF, bug.setNRunSF);
	printf("Ack failed : display=%d, endDisplay=%d, changeState=%d, setCC=%d, setNRun=%d\n",
				bug.displayAF, bug.endDisplayAF, bug.changeStateAF, bug.setCCAF, bug.setNRunAF);
	printf("Recieve Errors: timeout=%d, wrongAdr=%d, bufferFull=%d, sizeMismatch=%d, CRCFault=%d\n",
			bug.rxTimeout, bug.rxWrongAdr, bug.rxBufferFull, bug.rxSizeMismatch, bug.rxCRCFault);
	printf("Max recieve buffer size=%d\n",bug.rxMaxBuf);
	#endif

	// succes?
	return 0;
}

// Get direct access to BCM2708/9
int dgt3000Init() {
	int memfd, base;
	void *gpio_map, *timer_map, *i2c_slave_map, *i2c_master_map;
	struct sched_param params;

	memset(&dgtRx,0,sizeof(dgtReceive_t));
	#ifdef debug
	memset(&bug,0,sizeof(debug_t));
	#endif

	if (checkPiModel()==1)
		base=0x20000000;
	else
		base=0x3f000000;

	memfd = open("/dev/mem",O_RDWR|O_SYNC);
	if(memfd < 0) {
		printf("Mem open error\n");
		return 1;
	}

	gpio_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, GPIO_BASE+base);
	timer_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, TIMER_BASE+base);
	i2c_slave_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, I2C_SLAVE_BASE+base);
	i2c_master_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, I2C_MASTER_BASE+base);

	close(memfd);

	if( gpio_map == MAP_FAILED || timer_map == MAP_FAILED || i2c_slave_map == MAP_FAILED || i2c_master_map == MAP_FAILED) {
		printf("Map failed\n");
		return 1;
	}

	// GPIO pointers
	gpio =  (volatile unsigned *)gpio_map;
	gpioset = gpio + 7;     // set bit register offset 28
	gpioclr = gpio + 10;    // clr bit register
	gpioin = gpio + 13;     // read all bits register

	// timer pointer
	timer = (long long int *)((char *)timer_map + 4);

	// i2c slave pointers
	i2cSlave = (volatile unsigned *)i2c_slave_map;
	i2cSlaveRSR = i2cSlave + 1;
	i2cSlaveSLV = i2cSlave + 2;
	i2cSlaveCR = i2cSlave + 3;
	i2cSlaveFR = i2cSlave + 4;

	// i2c master pointers
	i2cMaster = (volatile unsigned *)i2c_master_map;
	i2cMasterS = i2cMaster + 1;
	i2cMasterDLEN = i2cMaster + 2;
	i2cMasterA = i2cMaster + 3;
	i2cMasterFIFO = i2cMaster + 4;
	i2cMasterDiv = i2cMaster + 5;

	// first run (pinmode GPIO17,18 != ALT3)? setup i2c
	//if((*(gpio+1) & 0x3f000000) != 0x3f000000)
		i2cReset();

	// set to I2CMaster destination adress
	*i2cMasterA=8;

	dgtRx.on=1;

	pthread_create(&receiveThread, NULL, dgt3000Receive, NULL);

	// give thread max priority
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(receiveThread, SCHED_FIFO, &params);

	return 0;
}

// configure dgt3000 to on, central controll and mode 25
int dgt3000Configure() {
	int e;

	// get the clock into the right state
	while (1) {
		// set to mode 25 and run
		e=dgt3000Mode25();
		if (e==-1) {
			// no postive ack, not in cc
			// set central controll
			while (1) {
				// try 3 times
				setCCCount++;
				// setCC>3?
				if (setCCCount>3) {
					printf("%.3f ",(float)*timer/1000000);
					printf("sending setCentralControll failed three times\n");
					return -3;
				}
				if (dgt3000SetCC()==0) break;
			}
		} else if (e==-2) {
			// timeout, line stay low -> reset i2c
			resetCount++;
			if (resetCount>1) {
				printf("%.3f ",(float)*timer/1000000);
				printf("I2C error, remove jack plug\n");
				return -2;
			}
			i2cReset();
			continue;
		} else if (e==-3) {
			// message not acked, clock off or collision
			if(dgt3000Wake()==-3) {
				printf("%.3f ",(float)*timer/1000000);
				printf("unable to wake the dgt3000\n");
				return -3;
			}
			continue;
		} else {
			// succes!
			usleep(5000);
			break;
		}
	}
	return 0;
}

// send a wake command to the dgt3000
int dgt3000Wake() {
	int e;
	long long int t;

	// turnOn#++
	wakeCount++;

	// turnOn#>3? -> error
	if (wakeCount>3) return -3;

	dgtRx.hello=0;

	// send wake
	*i2cMasterA=40;
	e=i2cSend(ping);
	*i2cMasterA=8;

	// succes? -> error. Wake messages should never get an Ack
	if (e==0) return -3;

	// Get Hello message (in max 10ms, usualy 5ms)
	t=*timer+10000;
	while (*timer<t) {
		if (dgtRx.hello==1)
			return 0;
		usleep(100);
	}

	return -1;
}

// send set central controll command to dgt3000
int dgt3000SetCC() {
	int e;

	// send setCC, error? retry
	e=i2cSend(centralControll);

	// send succedfull?
	if (e<0) {
		#ifdef debug
		bug.setCCSF++;
		#endif
		return e;
	}

	// listen to our own adress and get Reply

	e=dgt3000GetAck(0x10,0x0f,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		bug.setCCAF++;
		#endif
		return e;
	}

	// is positive ack?
	if ((dgtRx.ack[1]&8) == 8)
		return 0;

	// nack clock running
	return -1;
}

// send set mode 25 to dgt3000
int dgt3000Mode25() {
	int e;

	mode25[4]=57;
	crc_calc(mode25);

	// send mode 25 message
	e=i2cSend(mode25);

	// send succesful?
	if (e<0) {
		#ifdef debug
		bug.changeStateSF++;
		#endif
		return e;
	}

	// listen to our own adress an get Reply
	e=dgt3000GetAck(0x10,0x0b,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		bug.changeStateAF++;
		#endif
		return e;
	}

	if (dgtRx.ack[1]==8) return 0;

	// negetive ack not in CC
	return -1;
}

// send end display to dgt3000 to clear te display
int dgt3000EndDisplay() {
	int e;

	// send end Display
	e=i2cSend(endDisplay);

	// send succesful?
	if (e<0) {
		#ifdef debug
		bug.endDisplaySF++;
		#endif
		return e;
	}

	// get fast Reply = already empty
	e=dgt3000GetAck(0x10,0x07,1200);

	// display already empty
	if (e==0) {
		if ((dgtRx.ack[1]&0x07) == 0x05)
			return 0;
		else
			return -1;
	}

	//get slow broadcast Reply = display changed
	e=dgt3000GetAck(0x00,0x07,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		bug.endDisplayAF++;
		#endif
		return e;
	}

	// display emptied
	if ((dgtRx.ack[1]&0x07) == 0x00)
		return 0;

	return -1;
}

// send display command to dgt3000
int dgt3000SetDisplay(char dm[]) {
	int e;

	// send the message
	e=i2cSend(dm);

	// send succesful?
	if (e<0) {
		#ifdef debug
		bug.displaySF++;
		#endif
		return e;
	}

	// get (broadcast) reply
	e=dgt3000GetAck(0x00,0x06,10000);

	// no reply
	if (e<0) {
		#ifdef debug
		bug.displayAF++;
		#endif
		return e;
	}

	// nack, already displaying message
	if ((dgtRx.ack[1]&0xf3)==0x23) {
		printf("%.3f ",(float)*timer/1000000);
		printf("sending display command failed three times\n");
		return -1;
	}

	return 0;
}

// try three times to end and set de display
int dgt3000Display(char text[], char beep, char ld, char rd) {
	int i;

	for (i=0;i<11;i++) {
		if(text[i]==0) break;
		setDisplay[i+4]=text[i];
	}

	setDisplay[16]=beep;
	setDisplay[18]=ld;
	setDisplay[19]=rd;

	crc_calc(setDisplay);

	sendCount=0;
	while (1) {
		sendCount++;
		if (sendCount>3) {
			printf("%.3f ",(float)*timer/1000000);
			printf("sending clear display failed three times\n");
			return -3;
		}
		// succes?
		if (dgt3000EndDisplay()==0) break;
	}

	sendCount=0;
	while (1) {
		sendCount++;
		if (sendCount>3) {
			printf("%.3f ",(float)*timer/1000000);
			printf("sending display command failed three times\n");
			return -3;
		}
		// succes?
		if (dgt3000SetDisplay(setDisplay)==0) break;
	}
	return 0;
}

// send set and run command to dgt3000
int dgt3000SetNRun(char lr, char lh, char lm, char ls,
					char rr, char rh, char rm, char rs) {
	int e;

	setnrun[4]=lh;
	setnrun[5]=((lm/10)<<4) | (lm%10);
	setnrun[6]=((ls/10)<<4) | (ls%10);
	setnrun[7]=rh;
	setnrun[8]=((rm/10)<<4) | (rm%10);
	setnrun[9]=((rs/10)<<4) | (rs%10);
	setnrun[10]=lr | (rr<<2);

	crc_calc(setnrun);

	e=i2cSend(setnrun);

	// send succesful?
	if (e<0) {
		#ifdef debug
		bug.setNRunSF++;
		#endif
		return e;
	}

	// listen to our own adress an get Reply
	e=dgt3000GetAck(0x10,0x0a,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		bug.setNRunAF++;
		#endif
		return e;
	}

	// Positive Ack?
	if (dgtRx.ack[4]==8)
		return 0;

	return -1;
}

// check for messages from dgt3000
void *dgt3000Receive(void *a) {
	char rm[255];
	int e;
	#ifdef debug2
	int i;
	#endif

	while (dgtRx.on) {
		if ( (*i2cSlaveFR&0x20) != 0 || (*i2cSlaveFR&2) == 0 ) {
			pthread_mutex_lock(&receiveMutex);

			#ifdef debug
			PINKHI;
			#endif
			e=i2cReceive(rm);
			#ifdef debug
			PINKLO;
			#endif

			#ifdef debug2
			if (e>0) {
				printf("<- ");
			//	printf("maxs=%d  ",maxs);
			//	printf("length=%d  ",e);
				for (i=0;i<e;i++)
					printf("%02x ", rm[i]);
			} else if (e<0) {
				printf("<- ");
				for (i=0;i<16;i++)
					printf("%02x ", rm[i]);
			}
			#endif

			if (e>0) {
				switch (rm[3]) {
					case 1:		// ack
						dgtRx.ack[0]=rm[4];
						dgtRx.ack[1]=rm[5];
						pthread_cond_signal(&receiveCond);
						#ifdef debug2
						printf("= Ack %s\n",packetDescriptor[rm[4]-1]);
						#endif
						break;
					case 2:		// hello
						dgtRx.hello=1;
						#ifdef debug2
						printf("= Hello\n");
						#endif
						break;
					case 4:		// time
						dgtRx.time[0]=rm[5]&0x0f;
						dgtRx.time[1]=rm[6];
						dgtRx.time[2]=rm[7];
						dgtRx.time[3]=rm[11]&0x0f;
						dgtRx.time[4]=rm[12];
						dgtRx.time[5]=rm[13];
						// store (initial) lever state
						if ((rm[19]&1) == 1)
							dgtRx.lastButtonState |= 0x40;
						else
							dgtRx.lastButtonState &= 0xbf;
						#ifdef debug2
						printf("= Time: %02x:%02x.%02x %02x:%02x.%02x\n",rm[5]&0xf,rm[6],rm[7],rm[11]&0xf,rm[12],rm[13]);
						#endif
						if (rm[20]==1) ; // no update
						break;
					case 5:		// button
						// new button pressed
						if (rm[4]&0x1f)
							dgtRx.buttonState |= rm[4]&0x1f;
							dgtRx.lastButtonState = rm[4];

						// turned off/on
						if((rm[4]&0x20) != (rm[5]&0x20)) {
							// buffer full?
							if ((dgtRx.buttonEnd+1)%DGTRX_BUTTON_BUFFER_SIZE == dgtRx.buttonStart) {
								printf("%.3f ",(float)*timer/1000000);
								printf("Button buffer full, on/off ignored\n");
							} else {
								dgtRx.buttonPres[dgtRx.buttonEnd]=0x20 | ((rm[5]&0x20)<<2);
								dgtRx.buttonTime[dgtRx.buttonEnd]=0;
								dgtRx.buttonEnd=(dgtRx.buttonEnd+1)%DGTRX_BUTTON_BUFFER_SIZE;
							}
						}

						// lever change?
						if((rm[4]&0x40) != (rm[5]&0x40)) {
							if (ww)
								ww=0;
							else
								ww=1;
							// buffer full?
							if ((dgtRx.buttonEnd+1)%DGTRX_BUTTON_BUFFER_SIZE == dgtRx.buttonStart) {
								printf("%.3f ",(float)*timer/1000000);
								printf("Button buffer full, lever change ignored\n");
							} else {
								dgtRx.buttonPres[dgtRx.buttonEnd]=0x40 | ((rm[4]&0x40)<<1);
								dgtRx.buttonTime[dgtRx.buttonEnd]=0;
								dgtRx.buttonEnd=(dgtRx.buttonEnd+1)%DGTRX_BUTTON_BUFFER_SIZE;
							}
						}

						// buttons released
						if((rm[4]&0x1f) == 0 && dgtRx.buttonState != 0) {
							// buffer full?
							if ((dgtRx.buttonEnd+1)%DGTRX_BUTTON_BUFFER_SIZE == dgtRx.buttonStart) {
								printf("%.3f ",(float)*timer/1000000);
								printf("Button buffer full, buttons ignored\n");
							} else {
								dgtRx.buttonPres[dgtRx.buttonEnd]=dgtRx.buttonState;
								dgtRx.buttonTime[dgtRx.buttonEnd]=rm[8];
								dgtRx.buttonEnd=(dgtRx.buttonEnd+1)%DGTRX_BUTTON_BUFFER_SIZE;
							}
							dgtRx.buttonState=0;
						}
						#ifdef debug2
						printf("= Button: 0x%02x>0x%02x\n",rm[5]&0x7f,rm[4]&0x7f);
						#endif
						break;
					default:
						#ifdef debug
						printf("%.3f ",(float)*timer/1000000);
						printf("unknown message from clock\n");
						#endif
				}
			} else  if (e<0) {
				#ifdef debug2
				printf(" = Error: %d\n",e);
				#endif
				#ifdef debug
				printf("%.3f ",(float)*timer/1000000);
				printf("Receive Error:%d\n",e);
				#endif
			}
			pthread_mutex_unlock(&receiveMutex);
		} else {
			usleep(400);
		}
	}
	return 0;
}

// wait for an Ack message
int dgt3000GetAck(char adr, char cmd, long long int timeOut) {
	struct timespec receiveTimeOut;


	pthread_mutex_lock(&receiveMutex);

	// clear so we can receive a new ack.
	dgtRx.ack[0]=0;

	// listen to given adress
//	while ((*i2cSlaveFR&0x20) != 0 );
	*i2cSlaveSLV=adr;

	// check until timeout
	timeOut+=*timer;
	receiveTimeOut.tv_sec=timeOut/1000000;
	receiveTimeOut.tv_nsec=timeOut%1000000;

	while (*timer<timeOut) {
		if (dgtRx.ack[0]==cmd) {
			// listen for broadcast again
			*i2cSlaveSLV=0x00;
			pthread_mutex_unlock(&receiveMutex);
			return 0;
		}
		pthread_cond_timedwait(&receiveCond, &receiveMutex, &receiveTimeOut);
	}
	//printf("no ack\n");
	// listen for broadcast again
	*i2cSlaveSLV=0x00;
	pthread_mutex_unlock(&receiveMutex);
	return -3;
}

// return last received time
void dgt3000GetTime(char time[]) {
	time[0]=dgtRx.time[0];
	time[1]=((dgtRx.time[1]&0xf0)>>4)*10 + (dgtRx.time[1]&0x0f);
	time[2]=((dgtRx.time[2]&0xf0)>>4)*10 + (dgtRx.time[2]&0x0f);
	time[3]=dgtRx.time[3];
	time[4]=((dgtRx.time[4]&0xf0)>>4)*10 + (dgtRx.time[4]&0x0f);
	time[5]=((dgtRx.time[5]&0xf0)>>4)*10 + (dgtRx.time[5]&0x0f);
}

// return buttons pressed
int dgt3000GetButton(char *buttons, char *time) {
	//button availible?
	if(dgtRx.buttonStart != dgtRx.buttonEnd) {
		*buttons=dgtRx.buttonPres[dgtRx.buttonStart];
		*time=dgtRx.buttonTime[dgtRx.buttonStart];
		dgtRx.buttonStart=(dgtRx.buttonStart+1)%DGTRX_BUTTON_BUFFER_SIZE;
		return 1;
	} else {
		return 0;
	}
}


// return current button state
int dgt3000GetButtonState() {
	return dgtRx.lastButtonState;
}

// turn off dgt3000
int dgt3000Off(char returnMode) {
	int e;

	mode25[4]=32+returnMode;
	crc_calc(mode25);


	// send mode 25 message
	e=i2cSend(mode25);

	// send succesful?
	if (e<0) {
		#ifdef debug
		bug.changeStateSF++;
		#endif
		return e;
	}

	// listen to our own adress an get Reply
	e=dgt3000GetAck(0x10,0x0b,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		bug.changeStateAF++;
		#endif
		return e;
	}

	// negetive ack not in CC
	if (dgtRx.ack[1]!=8) return -1;

	mode25[4]=0;
	crc_calc(mode25);


	// send mode 25 message
	e=i2cSend(mode25);

	// send succesful?
	if (e<0) {
		#ifdef debug
		bug.changeStateSF++;
		#endif
		return e;
	}

	return 0;
}

// stop receiving
void dgt3000Stop() {
	// stop listening to broadcasts
	*i2cSlaveSLV=16;

	// stop thread
	dgtRx.on=0;

	// wait for thread to finish
	pthread_join(receiveThread, NULL);

	// disable i2cSlave device
	*i2cSlaveCR=0;
}



// send message using I2CMaster
int i2cSend(char message[]) {
	int i, n;
	long long int timeOut;

	// set length
	*i2cMasterDLEN = message[2]-1;

	// clear buffer
	*i2cMaster = 0x10;
	
	#ifdef debug2
	printf("-> %02x ", message[0]);
	#endif
		

	// fill the buffer
	for (n=1;n<message[2] && *i2cMasterS&0x10;n++) {
		#ifdef debug2
		printf("%02x ", message[n]);
		if(n == message[2]-1)
			printf("= %s\n",packetDescriptor[message[3]-1]);
		#endif
		*i2cMasterFIFO=message[n];
	}

	// check 256 times if the bus is free. At least for 50us because the clock will send waiting messages 50 us after the previeus one.
	timeOut=*timer + 10000;   // bus should be free in 10ms
	#ifdef debug
	GREENHI;
	#endif
	for(i=0;i<256;i++) {
		// lines low (data is being send, or plug half inserted, or PI I2C peripheral crashed or ...)
		if (SCL1IN==0 || SDA1IN==0) {
			i=0;
		}
		if ((*i2cSlaveFR&0x20) != 0 || (*i2cSlaveFR&2) == 0) {
//			dgt3000Receive();
			i=0;
		}
		// timeout waiting for bus free, I2C Error (or someone pushes 500 buttons/seccond)
		if (*timer>timeOut) return -2;
	}
	#ifdef debug
	GREENLO;
	#endif

	// dont let the slave listen to 0 (wierd errors)
	*i2cSlaveSLV = 0x10;

	// start sending
	*i2cMasterS = 0x302;
	*i2cMaster = 0x8080;

	// write the rest of the message
	for (;n<message[2];n++) {
		// wait for space in the buffer
		// TODO: timeout (I2CError)
		while((*i2cMasterS&0x10)==0) if (*i2cMasterS&2) break;
		#ifdef debug2
		printf("%02x ", message[n]);
		if(n == message[2]-1)
			printf("= %s\n",packetDescriptor[message[3]-1]);
		#endif
		*i2cMasterFIFO=message[n];
	}
		
	// wait for done
	// TODO; timeout (I2CError)
	while ((*i2cMasterS&2)==0);

	// let the slave listen to 0 again
	*i2cSlaveSLV = 0x0;

	// succes?
	if ((*i2cMasterS&0x300)==0) return 0;

	// collision or clock off
	#ifdef debug
//	PINKHI;
//	for(i=0;i<200;i++);
//	PINKLO;
	#endif

	// reset error flags
	*i2cMasterS=0x300;

	return -3;
}

// get message from I2C receive buffer
int i2cReceive(char m[]) {
	// todo implement end of packet check
	int i=1;
	long long int timeOut;

	m[0]=*i2cSlaveSLV*2;

	// a message should be finished receiving in 10ms
	timeOut=*timer+10000;

	#ifdef debug
	if (bug.rxMaxBuf<(*i2cSlaveFR&0xf800)>>11)
		bug.rxMaxBuf=(*i2cSlaveFR&0xf800)>>11;
	#endif

	// while I2CSlave is receiving or byte availible
	while( ((*i2cSlaveFR&0x20) != 0) || ((*i2cSlaveFR&2) == 0) ) {

		// timeout
		if (timeOut<*timer) {
			#ifdef debug
			bug.rxTimeout++;
			#endif
			pthread_mutex_unlock(&receiveMutex);
			return -2;
		}

		// when a byte is availible, store it
		if((*i2cSlaveFR&2) == 0) {
			m[i]=*i2cSlave & 0xff;
			i++;
			// complete packet
			if (i>2 && i>=m[2]) break;
		} else
		// no byte availible receiving a new one will take 70us
			usleep(70);
	}
	m[i]=-1;

	// nothing?
	if (i==1)
		return 0;

	// dgt3000 sends to 0 bytes after some packets
	if (i==3 && m[1]==0 && m[2]==0)
		return 0;

	// not from clock?
	if (m[1]!=16) {
		#ifdef debug
		bug.rxWrongAdr++;
		#endif
		return -2;
	}

	// errors?
	if (*i2cSlaveRSR&1 || i<5 || i!=m[2] )  {
		#ifdef debug
		if(*i2cSlaveRSR&1)
			bug.rxBufferFull++;
		else
			bug.rxSizeMismatch++;
		#endif
		*i2cSlaveRSR=0;
		return -5;
	}

	if (crc_calc(m)) {
		#ifdef debug
		bug.rxCRCFault++;
		#endif
		return i;
	}

	return i;
}

// configure IO pins and I2C Master and Slave
void i2cReset() {
	// pinmode GPIO2,GPIO3=ALT0 (togle via input to reset i2C master(sometimes hangs))
	*gpio &= 0xfffff03f;
	usleep(1000);	// not tested! some delay needed
	*gpio |= 0x900;

	// pinmode GPIO18,GPIO19=ALT3 (togle via input to reset)
	*(gpio+1) &= 0x00ffffff;
	usleep(1000);
	*(gpio+1) |= 0x3f000000;

	#ifdef debug
	// pinmode GPIO17,GPIO27=output for debugging
	*(gpio+1) = (*(gpio+1)&0xff1fffff) | 0x00200000;
	*(gpio+2) = (*(gpio+2)&0xff1fffff) | 0x00200000;
	#endif

	// set i2c slave control register to break and off
	*i2cSlaveCR = 0x80;
	// set i2c slave control register to enable: receive, i2c, device
	*i2cSlaveCR = 0x205;
	// set i2c slave address 0x00 to listen to broadcasts
	*i2cSlaveSLV = 0x0;
	// reset errors
	*i2cSlaveRSR = 0;

	// set i2c master to 100khz
	*i2cMasterDiv = 0x9c4;
}



// calculate checksum and put it in the last byte
char crc_calc(char *buffer) {
	int i;
	char crc_result = 0;
	char length = buffer[2]-1;

	for (i = 0; i < length; i++)
		crc_result = crc_table[ crc_result ^ buffer[i] ]; // new CRC will be the CRC of (old CRC XORed with data byte) - see http://sbs-forum.org/marcom/dc2/20_crc-8_firmware_implementations.pdf

	if (buffer[i]==crc_result)
		return 0;
	buffer[i]=crc_result;
	return -1;
}

// find out wich pi
int checkPiModel() {
	FILE *cpuFd ;
	char line [120] ;

	if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
		printf("Unable to open /proc/cpuinfo") ;

	// Start by looking for the Architecture, then we can look for a B2 revision....
	while (fgets (line, 120, cpuFd) != NULL)
		if (strncmp (line, "Hardware", 8) == 0) {
			// See if it's BCM2708 or BCM2709
			if (strstr (line, "BCM2709") != NULL) {
				fclose(cpuFd);
				return 2;	// PI 2
			} else {
				fclose(cpuFd);
				return 1;	// PI B+
			}
		}
	fclose(cpuFd);
	return 0;
}
