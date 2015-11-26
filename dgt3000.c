/* functions to communicate to a DGT3000 using I2C
 * version 0.7
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
	int e;
	int i=0;
	while (1)
		if (ww) {
			e=dgt3000Configure();
			if (e<0)
				printf("%d: Configure failed!\n",i);
			e=dgt3000Display("Goeindag",0,0,0);
			if (e<0)
				printf("%d: Display failed!\n",i);
			e=dgt3000SetNRun(1,0,10,0,2,0,10,0);
			if (e<0)
				printf("%d: SetNRun failed!\n",i);
			i++;
			#ifdef debug
			bug.sendTotal++;
			#endif
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
		#ifdef debug
		printf("/dev/mem open error, run as root\n");
		#endif
		return -1;
	}

	gpio_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, GPIO_BASE+base);
	timer_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, TIMER_BASE+base);
	i2c_slave_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, I2C_SLAVE_BASE+base);
	i2c_master_map = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, I2C_MASTER_BASE+base);

	close(memfd);

	if( gpio_map == MAP_FAILED || timer_map == MAP_FAILED || i2c_slave_map == MAP_FAILED || i2c_master_map == MAP_FAILED) {
		#ifdef debug
		printf("Map failed\n");
		#endif
		return -1;
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

	// check wiring
	// configured as an output? probably in use for something else
	if ((*gpio & 0x1c0) == 0x40) {
		#ifdef debug
		printf("Error, GPIO02 configured as output, in use? We asume not a DGTPI\n");
		#endif
		return -2;
	}
	if ((*gpio & 0xe00) == 0x200) {
		#ifdef debug
		printf("Error, GPIO03 configured as output, in use? We asume not a DGTPI\n");
		#endif
		return -2;
	}
	if ((*(gpio+1) & 0x07000000) == 0x01000000) {
		#ifdef debug
		printf("Error, GPIO18 configured as output, in use? We asume not a DGTPI\n");
		#endif
		return -2;
	}
	if ((*(gpio+1) & 0x38000000) == 0x08000000) {
		#ifdef debug
		printf("Error, GPIO19 configured as output, in use? We asume not a DGTPI\n");
		#endif
		return -2;
	}
	// pinmode GPIO2,GPIO3=input
	*gpio &= 0xfffff03f;
	// pinmode GPIO18,GPIO19=input
	*(gpio+1) &= 0xc0ffffff;
	usleep(1);
	// all pins hi through pullup?
	if ((*gpioin & 0xc000c)!=0xc000c) {
		#ifdef debug
		printf("Error, pin(s) low, shortcircuit, or no connection?\n");
		#endif
		return -2;
	}
	// check SDA connection
	// gpio18 low
	*gpioclr = 1<<18;
	// gpio18 output
	*(gpio+1) |= 0x01000000;
	usleep(1);
	// check gpio 18 and 2
	if ((*gpioin & 0x40004)!=0) {
		#ifdef debug
		printf("Error, SDA not connected\n");
		#endif
		// gpio18 back to input
		*(gpio+1) &= 0xf8ffffff;
		return -2;
	}
	if ((*gpioin & 0x80008)!=0x80008) {
		#ifdef debug
		printf("Error, SDA connected to SCL\n");
		#endif
		// gpio18 back to input
		*(gpio+1) &= 0xf8ffffff;
		return -2;
	}
	// gpio18 back to input
	*(gpio+1) &= 0xf8ffffff;
	// check SCL connection
	// gpio19 low
	*gpioclr = 1<<19;
	// gpio18 output
	*(gpio+1) |= 0x08000000;
	usleep(1);
	// check gpio 19 and 3
	if ((*gpioin & 0x80008)!=0) {
		#ifdef debug
		printf("Error, SCL not connected\n");
		#endif
		// gpio19 back to input
		*(gpio+1) &= 0xc7ffffff;
		return -2;
	}
	// gpio19 back to input
	*(gpio+1) &= 0xc7ffffff;	
	
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
	int wakeCount = 0;
	int setCCCount = 0;
	int resetCount = 0;


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
					#ifdef debug
					ERROR_PIN_HI;
					printf("%.3f ",(float)*timer/1000000);
					printf("sending setCentralControll failed three times\n\n");
					ERROR_PIN_LO;
					#endif
					return -3;
				}
				if (dgt3000SetCC()==0) break;
			}
		} else if (e==-2) {
			// timeout, line stay low -> reset i2c
			resetCount++;
			if (resetCount>1) {
				#ifdef debug
				ERROR_PIN_HI;
				printf("%.3f ",(float)*timer/1000000);
				printf("I2C error, remove jack plug\n\n");
				ERROR_PIN_LO;
				#endif
				return -2;
			}
			i2cReset();
			continue;
		} else if (e==-3) {
			// message not acked, probably collision
			continue;
		} else if (e==-7) {
			// message not acked, probably clock off -> wake
			// wake#++
			wakeCount++;

			// wake#>3? -> error
			if (wakeCount>3) {
				#ifdef debug
				ERROR_PIN_HI;
				printf("%.3f ",(float)*timer/1000000);
				printf("sending wake command failed three times\n");
				ERROR_PIN_LO;
				#endif
				return -3;
			}
			dgt3000Wake();
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



	// send wake
	*i2cMasterA=40;
	e=i2cSend(ping,0x00);
	*i2cMasterA=8;

	// succes? -> error. Wake messages should never get an Ack
	if (e==0) {
		#ifdef debug
		ERROR_PIN_HI;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending wake command failed, received Ack, this should never hapen\n");
		ERROR_PIN_LO;
		#endif
		return -3;
	}

	// Get Hello message (in max 10ms, usualy 5ms)
	t=*timer+10000;
	while (*timer<t) {
		if (dgtRx.hello==1)
			return 0;
		usleep(100);
	}

	#ifdef debug
	ERROR_PIN_HI;
	printf("%.3f ",(float)*timer/1000000);
	printf("sending wake command failed, no hello\n");
	ERROR_PIN_LO;
	#endif
	
	return -1;
}

// send set central controll command to dgt3000
int dgt3000SetCC() {
	int e;

	// send setCC, error? retry
	e=i2cSend(centralControll,0x10);

	// send succedfull?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.setCCSF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending SetCentralControll command failed, sending failed\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// listen to our own adress and get Reply

	e=dgt3000GetAck(0x10,0x0f,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.setCCAF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending SetCentralControll command failed, no ack\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// is positive ack?
	if ((dgtRx.ack[1]&8) == 8)
		return 0;
	
	#ifdef debug
	ERROR_PIN_HI;
	printf("%.3f ",(float)*timer/1000000);
	printf("sending SetCentralControll command failed, negative ack, clock running\n");
	ERROR_PIN_LO;
	#endif

	// nack clock running
	return -1;
}

// send set mode 25 to dgt3000
int dgt3000Mode25() {
	int e;

	mode25[4]=57;
	crc_calc(mode25);

	// send mode 25 message
	e=i2cSend(mode25, 0x10);

	// send succesful?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.changeStateSF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending mode25 command failed, sending failed\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// listen to our own adress an get Reply
	e=dgt3000GetAck(0x10,0x0b,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.changeStateAF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending mode25 command failed, no ack\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	if (dgtRx.ack[1]==8) return 0;

	#ifdef debug
	ERROR_PIN_HI;
	printf("%.3f ",(float)*timer/1000000);
	printf("sending mode25 command failed, negative ack, not in Central Controll\n");
	ERROR_PIN_LO;
	#endif
	
	// negetive ack not in CC
	return -1;
}

// send end display to dgt3000 to clear te display
int dgt3000EndDisplay() {
	int e;

	// send end Display
	e=i2cSend(endDisplay,0x10);

	// send succesful?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.endDisplaySF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending end display command failed, sending failed\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// get fast Reply = already empty
	e=dgt3000GetAck(0x10,0x07,1200);

	// display already empty
	if (e==0) {
		if ((dgtRx.ack[1]&0x07) == 0x05) {
			return 0;
		} else {
			#ifdef debug
			ERROR_PIN_HI;
			printf("%.3f ",(float)*timer/1000000);
			printf("sending end display command failed, negative specific ack:%02x\n",dgtRx.ack[1]);
			ERROR_PIN_LO;
			#endif
			return -1;
		}
	}

	//get slow broadcast Reply = display changed
	e=dgt3000GetAck(0x00,0x07,10000);

	// ack received?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.endDisplayAF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending end display command failed, no ack\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// display emptied
	if ((dgtRx.ack[1]&0x07) == 0x00)
		return 0;
	
	#ifdef debug
	ERROR_PIN_HI;
	printf("%.3f ",(float)*timer/1000000);
	printf("sending end display command failed, negative broadcast ack:%02x\n",dgtRx.ack[1]);
	ERROR_PIN_LO;
	#endif

	return -1;
}

// send display command to dgt3000
int dgt3000SetDisplay(char dm[]) {
	int e;

	// send the message
	e=i2cSend(dm,0x00);

	// send succesful?
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.displaySF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending display command failed, sending failed\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// get (broadcast) reply
	e=dgt3000GetAck(0x00,0x06,10000);

	// no reply
	if (e<0) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.displayAF++;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending display command failed, no ack\n");
		ERROR_PIN_LO;
		#endif
		return e;
	}

	// nack, already displaying message
	if ((dgtRx.ack[1]&0xf3)==0x23) {
		#ifdef debug
		ERROR_PIN_HI;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending display command failed, display already busy\n");
		ERROR_PIN_LO;
		#endif
		return -1;
	}

	return 0;
}

// try three times to end and set de display
int dgt3000Display(char text[], char beep, char ld, char rd) {
	int i,e;
	int sendCount = 0;


	for (i=0;i<11;i++) {
		if(text[i]==0) break;
		setDisplay[i+4]=text[i];
	}

	for (;i<11;i++) {
		setDisplay[i+4]=' ';
	}

	setDisplay[16]=beep;
	setDisplay[18]=ld;
	setDisplay[19]=rd;

	crc_calc(setDisplay);

	while (1) {
		sendCount++;
		if (sendCount>3) {
			#ifdef debug
			ERROR_PIN_HI;
			printf("%.3f ",(float)*timer/1000000);
			printf("sending clear display failed three times on error%d\n\n",e);
			ERROR_PIN_LO;
			#endif
			return e;
		}
		
		e=dgt3000EndDisplay();
		// succes?
		if (e==0)
			break;
	}

	sendCount=0;
	while (1) {
		sendCount++;
		if (sendCount>3) {
			#ifdef debug
			ERROR_PIN_HI;
			printf("%.3f ",(float)*timer/1000000);
			printf("sending display command failed three times on error%d\n\n",e);
			ERROR_PIN_LO;
			#endif
			return e;
		}
		// succes?
		e=dgt3000SetDisplay(setDisplay);
		if (e==0)
			break;
	}
	return 0;
}

// send set and run command to dgt3000
int dgt3000SetNRun(char lr, char lh, char lm, char ls,
					char rr, char rh, char rm, char rs) {
	int e;
	int sendCount = 0;
	
	setnrun[4]=lh;
	setnrun[5]=((lm/10)<<4) | (lm%10);
	setnrun[6]=((ls/10)<<4) | (ls%10);
	setnrun[7]=rh;
	setnrun[8]=((rm/10)<<4) | (rm%10);
	setnrun[9]=((rs/10)<<4) | (rs%10);
	setnrun[10]=lr | (rr<<2);

	crc_calc(setnrun);

	while (1) {
		sendCount++;
		if (sendCount>3) {
			#ifdef debug
			ERROR_PIN_HI;
			printf("%.3f ",(float)*timer/1000000);
			printf("sending SetNRun failed three times on error%d\n\n",e);
			ERROR_PIN_LO;
			#endif
			return e;
		}
		
		e=i2cSend(setnrun,0x10);

		// send succesful?
		if (e<0) {
			#ifdef debug
			ERROR_PIN_HI;
			bug.setNRunSF++;
			printf("%.3f ",(float)*timer/1000000);
			printf("sending SetNRun command failed, sending failed\n");
			ERROR_PIN_LO;
			#endif
			continue;
		}

		// listen to our own adress an get Reply
		e=dgt3000GetAck(0x10,0x0a,10000);

		// ack received?
		if (e<0) {
			#ifdef debug
			ERROR_PIN_HI;
			bug.setNRunAF++;
			printf("%.3f ",(float)*timer/1000000);
			printf("sending SetNRun command failed, no ack\n");
			ERROR_PIN_LO;
			#endif
			continue;
		}

		// Positive Ack?
		if (dgtRx.ack[1]==8)
			return 0;

		#ifdef debug
		ERROR_PIN_HI;
		printf("%.3f ",(float)*timer/1000000);
		printf("sending SetNRun command failed, not in mode 25\n");
		ERROR_PIN_LO;
		#endif
		
		dgt3000Configure();
	}
}

// check for messages from dgt3000
void *dgt3000Receive(void *a) {
	char rm[RECEIVE_BUFFER_LENGTH];
	int e;
	#ifdef debug2
	int i;
	#endif
	
	#ifdef debug
	RECEIVE_THREAD_RUNNING_PIN_HI;
	#endif

	while (dgtRx.on) {
		if ( (*i2cSlaveFR&0x20) != 0 || (*i2cSlaveFR&2) == 0 ) {
			pthread_mutex_lock(&receiveMutex);

			e=i2cReceive(rm);
			
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
								#ifdef debug
								printf("%.3f ",(float)*timer/1000000);
								printf("Button buffer full, on/off ignored\n");
								#endif
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
								#ifdef debug
								printf("%.3f ",(float)*timer/1000000);
								printf("Button buffer full, lever change ignored\n");
								#endif
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
								#ifdef debug
								printf("%.3f ",(float)*timer/1000000);
								printf("Button buffer full, buttons ignored\n");
								#endif
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
						#ifdef debug
					default:
						ERROR_PIN_HI;
						printf("%.3f ",(float)*timer/1000000);
						printf("Receive Error: Unknown message from clock\n");
						ERROR_PIN_LO;
						#endif
				}
			} else  if (e<0) {
				#ifdef debug2
				printf(" = Error: %d\n",e);
				#endif
			}
			pthread_mutex_unlock(&receiveMutex);
		} else {
			#ifdef debug
			RECEIVE_THREAD_RUNNING_PIN_LO;
			#endif
			usleep(400);
			#ifdef debug
			RECEIVE_THREAD_RUNNING_PIN_HI;
			#endif

		}
	}
	#ifdef debug
	RECEIVE_THREAD_RUNNING_PIN_LO;
	#endif

	return 0;
}

// wait for an Ack message
int dgt3000GetAck(char adr, char cmd, long long int timeOut) {
	struct timespec receiveTimeOut;


	pthread_mutex_lock(&receiveMutex);


	// listen to given adress
//	while ((*i2cSlaveFR&0x20) != 0 );
	*i2cSlaveSLV=adr;

	// check until timeout
	timeOut+=*timer;
	receiveTimeOut.tv_sec=timeOut/1000000;
	receiveTimeOut.tv_nsec=timeOut%1000000;

	while (*timer<timeOut) {
		if (dgtRx.ack[0]==cmd) {
			pthread_mutex_unlock(&receiveMutex);
			return 0;
		}
		pthread_cond_timedwait(&receiveCond, &receiveMutex, &receiveTimeOut);
	}

	// listen for broadcast again
	*i2cSlaveSLV=0x00;
	pthread_mutex_unlock(&receiveMutex);

	if (dgtRx.ack[0]==cmd)
		return 0;
	else
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
	e=i2cSend(mode25,0x10);

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
	e=i2cSend(mode25,0x00);

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
	
	// pinmode GPIO2,GPIO3=input
	*gpio &= 0xfffff03f;
	// pinmode GPIO18,GPIO19=input
	*(gpio+1) &= 0xc0ffffff;
}



// send message using I2CMaster
int i2cSend(char message[], char ackAdr) {
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
	WAIT_FOR_FREE_BUS_PIN_HI;
	#endif
	for(i=0;i<256;i++) {
		// lines low (data is being send, or plug half inserted, or PI I2C peripheral crashed or ...)
		if ((SCL1IN==0) || (SDA1IN==0)) {
			i=0;
		}
		if ( ((*i2cSlaveFR&0x20)!=0) || ((*i2cSlaveFR&2)==0) ) {
			i=0;
		}
		// timeout waiting for bus free, I2C Error (or someone pushes 500 buttons/seccond)
		if (*timer>timeOut) {
			#ifdef debug
			printf("%.3f ",(float)*timer/1000000);
			printf("    Send error: Bus free timeout, waited more then 10ms for bus to be free\n");
			if(SCL1IN==0)
				printf("                SCL low. Remove jack?\n");
			if(SDA1IN==0)
				printf("                SDA low. Remove jack?\n");
			if((*i2cSlaveFR&0x20) != 0)
				printf("                I2C Slave receive busy, is the receive thread running?\n");
			if((*i2cSlaveFR&2) == 0)
				printf("                I2C Slave receive fifo not emtpy, is the receive thread running?\n");
			#endif
			return -2;
		}
	}
	#ifdef debug
	WAIT_FOR_FREE_BUS_PIN_LO;
	#endif

	// clear ack and hello so we can receive a new ack or hello
	dgtRx.ack[0]=0;
	dgtRx.hello=0;

	// dont let the slave listen to 0 (wierd errors)?
	// listen to ack adress
	*i2cSlaveSLV = ackAdr;

	// start sending
	*i2cMasterS = 0x302;
	*i2cMaster = 0x8080;

	// write the rest of the message
	for (; n<message[2]; n++) {
		// wait for space in the buffer
		timeOut=*timer + 10000;   // should be done in 10ms
		while((*i2cMasterS&0x10)==0) {
			if (*i2cMasterS&2) {
				*i2cSlaveSLV = 0x00;
				#ifdef debug
				printf("%.3f ",(float)*timer/1000000);
				printf("    Send error: done before complete send\n");
				#endif
				break;
			}
			if (*timer>timeOut) {
				*i2cSlaveSLV = 0x00;
				#ifdef debug
				printf("%.3f ",(float)*timer/1000000);
				printf("    Send error: Buffer free timeout, waited more then 10ms for space in the buffer\n");
				#endif
				return -2;
			}
		}
		if (*i2cMasterS&2)
			break;
		#ifdef debug2
		printf("%02x ", message[n]);
		if(n == message[2]-1)
			printf("= %s\n",packetDescriptor[message[3]-1]);
		#endif
		*i2cMasterFIFO=message[n];
	}
		
	// wait for done
	timeOut=*timer + 10000;   // should be done in 10ms
	while ((*i2cMasterS&2)==0)
		if (*timer>timeOut) {
			*i2cSlaveSLV = 0x00;
			#ifdef debug
			printf("%.3f ",(float)*timer/1000000);
			printf("    Send error: done timeout, waited more then 10ms for message to be finished sending\n");
			#endif
			return -2;
		}

	// succes?
	if ((*i2cMasterS&0x300)==0) {
//		*i2cSlaveSLV = ackAdr;
		return 0;
	}

	*i2cSlaveSLV = 0x00;
	
	// collision or clock off
	if (*i2cMasterS&0x100) {
		// reset error flags
		*i2cMasterS=0x100;
		#ifdef debug
		printf("%.3f ",(float)*timer/1000000);
		printf("    Send error: byte not Acked\n");
		#endif
	}
	if (*i2cMasterS&0x200) {
		// reset error flags
		*i2cMasterS=0x200;
		#ifdef debug
		printf("%.3f ",(float)*timer/1000000);
		printf("    Send error: collision, clock stretch timeout\n");
		#endif
		
		// probably collision
		return -3;
	}

	// clear fifo
	*i2cMaster|=0x10;		
		

	if ((SCL1IN==0) || (SDA1IN==0) || ((*i2cSlaveFR&0x20)!=0) || ((*i2cSlaveFR&2)==0)) {
		#ifdef debug
		printf("%.3f ",(float)*timer/1000000);
		printf("    Send error: collision, lines busy after send.\n");
		#endif
		
		// probably collision
		return -3;
	}
	
	// probably clock off
	return -7;
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
			ERROR_PIN_HI;
			printf("%.3f ",(float)*timer/1000000);
			printf("    Receive error: Timeout, hardware stays in receive mode for more then 10ms\n");
			bug.rxTimeout++;
			hexPrint(m,i);
			ERROR_PIN_LO;
			#endif
			pthread_mutex_unlock(&receiveMutex);
			return -2;
		}

		// when a byte is availible, store it
		if((*i2cSlaveFR&2) == 0) {
			m[i]=*i2cSlave & 0xff;
			i++;
			// complete packet
			if (i>2 && i>=m[2])
				break;
			if (i >= RECEIVE_BUFFER_LENGTH) {
				#ifdef debug
				ERROR_PIN_HI;
				bug.rxWrongAdr++;
				printf("%.3f ",(float)*timer/1000000);
				printf("    Receive error: Buffer overrun, size to large for the supplied buffer %d bytes.\n",i);
				hexPrint(m,i);
				ERROR_PIN_LO;
				#endif
				return -4;
			}
		} else {
		// no byte availible receiving a new one will take 70us
			#ifdef debug
			RECEIVE_THREAD_RUNNING_PIN_LO;
			#endif
			usleep(10);
			#ifdef debug
			RECEIVE_THREAD_RUNNING_PIN_HI;
			#endif
		}
	}

	// listen for broadcast again
	*i2cSlaveSLV=0x00;

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
		ERROR_PIN_HI;
		bug.rxWrongAdr++;
		printf("%.3f ",(float)*timer/1000000);
		printf("    Receive error: Wrong adress, Received message not from clock (16) but from %d.\n",m[1]);
		hexPrint(m,i);
		ERROR_PIN_LO;
		#endif
		return -2;
	}

	// errors?
	if (*i2cSlaveRSR&1 || i<5 || i!=m[2] )  {
		#ifdef debug
		ERROR_PIN_HI;
		printf("%.3f ",(float)*timer/1000000);
		if(*i2cSlaveRSR&1) {
			printf("    Receive error: Hardware buffer full.\n");
			bug.rxBufferFull++;
		} else {
			if (i<5)
				printf("    Receive Error: Packet to small, %d bytes.\n",i);
			else
				printf("    Receive Error: Size mismatch, packet length is %d bytes but received %d bytes.\n",m[2],i);
			bug.rxSizeMismatch++;
		}
		hexPrint(m,i);
		ERROR_PIN_LO;
		#endif
		*i2cSlaveRSR=0;
		return -5;
	}

	if (crc_calc(m)) {
		#ifdef debug
		ERROR_PIN_HI;
		bug.rxCRCFault++;
		printf("%.3f ",(float)*timer/1000000);
		printf("    Receive error: CRC Error\n");
		hexPrint(m,i);
		ERROR_PIN_LO;
		#endif
		return -6;
	}

	return i;
}

// read register
static unsigned int dummyRead(volatile unsigned int *addr) {
	return *addr;
}

// configure IO pins and I2C Master and Slave
void i2cReset() {	
	*i2cSlaveCR = 0;
	*i2cMaster = 0x10;
	*i2cMaster = 0x8000;
	
	// pinmode GPIO2,GPIO3=input (togle via input to reset i2C master(sometimes hangs))
	*gpio &= 0xfffff03f;
	// pinmode GPIO18,GPIO19=input (togle via input to reset)
	*(gpio+1) &= 0x00ffffff;
	// send something in case master hangs
	//*i2cMasterFIFO = 0x69;
	*i2cMasterDLEN = 0;
	*i2cMaster = 0x8080;
	while((*i2cSlaveFR&2) == 0) {
		dummyRead(i2cSlave);
	}
	usleep(1000);	// not tested! some delay maybe needed
	*i2cSlaveCR = 0;
	*i2cMasterS = 0x302;
	*i2cMaster = 10;
	// pinmode GPIO2,GPIO3=ALT0
	*gpio |= 0x900;
	// pinmode GPIO18,GPIO19=ALT3
	*(gpio+1) |= 0x3f000000;

	usleep(1000);	// not tested! some delay maybe needed


	#ifdef debug
	if ((SDA1IN==0) || (SCL1IN==0)) {
		printf("I2C Master might be stuck in transfer?\n");
		printf("FIFO=%x\n",*i2cMasterFIFO);
		printf("C   =%x\n",*i2cMaster);
		printf("S   =%x\n",*i2cMasterS);
		printf("DLEN=%x\n",*i2cMasterDLEN);
		printf("A   =%x\n",*i2cMasterA);
		printf("FIFO=%x\n",*i2cMasterFIFO);
		printf("DIV =%x\n",*i2cMasterDiv);
		printf("SDA=%x\n",SDA1IN);
		printf("SCL=%x\n",SCL1IN);
	}
	// pinmode GPIO17,GPIO27,GPIO22=output for debugging
	*(gpio+1) = (*(gpio+1)&0xff1fffff) | 0x00200000;	// GIO17
	*(gpio+2) = (*(gpio+2)&0xff1fffff) | 0x00200000;	// GIO27
	*(gpio+2) = (*(gpio+2)&0xfffffe3f) | 0x00000040;	// GIO22
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


// print hex values
void hexPrint(char bytes[], int length) {
	int i;
	
	for (i=0;i<length;i++)
		printf("%02x ", bytes[i]);
	printf("\n");
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
		#ifdef debug
		printf("Unable to open /proc/cpuinfo") 
		#endif
		;

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
