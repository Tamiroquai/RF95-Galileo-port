/*
* Author: Kiveisha Yevgeniy
* Copyright (c) 2015 Intel Corporation.
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
* LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
* OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <signal.h>
#include "RH_RF95.h"
#include "mraa.h"

bool amWorking = true;

void
sig_handler (int signo) {
	if (signo == SIGINT) {
		amWorking = false;
	}
}

int
main (int argc, char ** argv) {
    RH_RF95* rf95 = new RH_RF95 (10, 0);

    if (!rf95->init()) {
        std::cout << "RF22 init failed" << std::endl;
        return 0x1;
    }

    uint8_t data[] = "Hello World!";
    uint8_t buf[RH_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    signal (SIGINT, sig_handler);
    while (amWorking) {
        std::cout << "Sending to rf22_server" << std::endl;
        // Send a message to rf22_server
        rf95->send(data, sizeof(data));
        rf95->waitPacketSent();
        // Now wait for a reply
				
		rf95->waitAvailableTimeout(1000000);
		
		if (rf95->recv(buf, &len)) {
            std::cout << "got response: " << (char*)buf << std::endl;
		} else {
            std::cout << "!!! NO RESPONSE !!!" << std::endl;
        }
    }
	delete rf95;
	std::cout << "Exit 'rfm22-client'" << std::endl;
	return 0;
}

