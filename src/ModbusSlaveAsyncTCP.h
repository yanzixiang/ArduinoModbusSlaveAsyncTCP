/**
 * Copyright (c) 2015, Yaacov Zamir <kobi.zamir@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any 
 * purpose with or without fee is hereby granted, provided that the above 
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR 
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES 
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN 
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF 
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF  THIS SOFTWARE.
 */

#include <inttypes.h>
#include <AsyncTCP.h>
#include "ModbusSlaveDefs.h"

#define MAX_BUFFER 64

/**
 * @class Modbus 
 */
class ModbusAsyncTCP {
public:
    ModbusAsyncTCP(uint8_t unitID);
    void begin();
    uint16_t readRegisterFromBuffer(int offset);
    void writeCoilToBuffer(int offset, uint16_t state);
    void writeRegisterToBuffer(int offset, uint16_t value);
    void writeStringToBuffer(int offset, uint8_t *str, uint8_t length);
	
	void EnableSlaveIDCheck();
	void DisableSlaveIDCheck();
	bool CheckSlaveIDEnabled();
    
    CallBackFunc cbVector[4];
		
private:
    uint8_t unitID;
	bool checkSlaveID;
    uint8_t bufIn[MAX_BUFFER];
    uint8_t bufOut[MAX_BUFFER];
	int parse(char*, size_t, AsyncClient*);
};

