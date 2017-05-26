/* functions to communicate to a DGT3000 using I2C
 * version 0.8
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

#include "dgtpicom.h"
#include "dgtpicom_dgt3000.h"

int main (int argc, char *argv[]) {
	return ERROR_OK;
}

// Get direct access to BCM2708/9 chip.
int dgtpicom_init() {
	return ERROR_OK;
}

// Configure the dgt3000.
int dgtpicom_configure() {
	return ERROR_OK;
}

// send set and run command to dgt3000
int dgtpicom_set_and_run(char lr, char lh, char lm, char ls,
						 char rr, char rh, char rm, char rs) {
	return ERROR_OK;
}

// Send set and run command to the dgt3000 with current clock values.
int dgtpicom_run(char lr, char rr) {
	return ERROR_OK;
}

// Set a text message on the DGT3000.
int dgtpicom_set_text(char text[], char beep, char ld, char rd) {
	return ERROR_OK;
}

// End a text message on the DGT3000 an return to clock mode.
int dgtpicom_end_text() {
	return ERROR_OK;
}

// Put the last received time message in time[].
void dgtpicom_get_time(char time[]) {
	time[0]=0;
	time[1]=0;
	time[2]=0;
	time[3]=0;
	time[4]=0;
	time[5]=0;
}

// Get a button message from the buffer returns number of messages in
// the buffer or recieve error if one occured.
int dgtpicom_get_button_message(char *buttons, char *time) {
	return ERROR_OK;
}

// Return the current button state.
int dgtpicom_get_button_state() {
	return dgtRx.lastButtonState;
}

// Turn off the dgt3000.
int dgtpicom_off(char returnMode) {
	return ERROR_OK;
}

// Disable the I2C hardware.
void dgtpicom_stop() {
}

