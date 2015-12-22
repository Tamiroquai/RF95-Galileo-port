//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF95.cpp,v 1.8 2015/08/12 23:18:51 mikem Exp $


#include <RH_RF95.h>
#include <mraa.h>
#include <cstring>
#include <cmath>
#include <sys/time.h>
#include <stdint.h>
#include <iostream>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF95, allowing you to have
// 2 or more LORAs per Arduino
RH_RF95* RH_RF95::_RF95ForInterrupt[2] = {0, 0};
uint8_t RH_RF95::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
static const RH_RF95::ModemConfig MODEM_CONFIG_TABLE[] =
{
    //  1d,     1e,      26
    { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
    { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
    { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
    { 0x78,   0xc4,    0x00}, // Bw125Cr48Sf4096
    
};

RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin)
    :
    _rxBufValid(0),
    _mode(RHModeInitialising),
    _thisAddress(RH_BROADCAST_ADDRESS),
    _txHeaderTo(RH_BROADCAST_ADDRESS),
    _txHeaderFrom(RH_BROADCAST_ADDRESS),
    _txHeaderId(0),
    _txHeaderFlags(0),
    _rxBad(0),
    _rxGood(0),
    _txGood(0)
{
    _slaveSelectPin = slaveSelectPin;
    _interruptPin = interruptPin;
    _myInterruptIndex = 0xff; // Not allocated yet
}

uint8_t RH_RF95::init()
{
    // initialize Slave select pin
    _cs = mraa_gpio_init(_slaveSelectPin);
    mraa_gpio_dir(_cs, MRAA_GPIO_OUT);
    mraa_gpio_write(_cs, 0x1);

    // start the SPI library:
    // Note the RF22 wants mode 0, MSB first and default to 1 Mbps
    _spi = mraa_spi_init(0);
    mraa_spi_mode (_spi, MRAA_SPI_MODE0);
    mraa_spi_lsbmode(_spi, 0);
    mraa_spi_frequency(_spi, 1000000); // 1Mhz
    usleep (10000);

     // No way to check the device type :-(
     
    // Set sleep mode, so we can also set LORA mode:
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
    usleep(10000); // Wait for sleep mode to take over from say, CAD
    // Check we are in sleep mode, with LORA set
    if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE))
    {
				return 0; // No device present?
    }

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
//    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knwledge of what Arduino board you are running on.
    gpio_edge_t edge = MRAA_GPIO_EDGE_RISING;
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	_irq = mraa_gpio_init(_interruptPin + 2);
        mraa_gpio_dir(_irq, MRAA_GPIO_IN);
  
	if (_interruptCount <= RH_RF95_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return 0; // Too many devices, not enough interrupt vectors
    }
    _RF95ForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
    {
		_RF95ForInterrupt[0] = this;
 		mraa_gpio_isr(_irq, edge, &RH_RF95::isr0,NULL);
    }
	  else if (_myInterruptIndex == 1)
    {
    		_RF95ForInterrupt[1] = this;
    		mraa_gpio_isr(_irq, edge, &RH_RF95::isr1,NULL);
    }	
    else
	return 0; // Too many devices, not enough interrupt vectors

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    setModeIdle();

    // Set up default configuration
    // No Sync Words in LORA mode.
    setModemConfig(Bw125Cr45Sf128); // Radio default
//    setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
    setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    setFrequency(434.0);
    // Lowish power
    setTxPower(13);

    return 1;
}

bool RH_RF95::waitAvailableTimeout(unsigned long timeout)
{
    unsigned long endtime = getTimestamp() + timeout;
	unsigned long currenttime = getTimestamp();
    while (currenttime < endtime) {
		currenttime = getTimestamp();
		if (available()) {
			return true;
		}
	}
		
    return false;
}

uint64_t 
RH_RF95::getTimestamp () {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(1000000 * tv.tv_sec + tv.tv_usec);
}

void RH_RF95::waitPacketSent()
{
    while (_mode == RHModeTx)
    return;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void RH_RF95::handleInterrupt()
{
//std::cout << ".";
    // Read the interrupt register
    uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
    {
	_rxBad++;
    }
    else if (_mode == RHModeRx  && irq_flags & RH_RF95_RX_DONE)
    {   
	// Have received a packet
	uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

	// Reset the fifo read ptr to the beginning of the packet
	spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
	spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
	_bufLen = len;
	spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

	// Remember the RSSI of this packet
	// this is according to the doc, but is it really correct?
	// weakest receiveable signals are reported RSSI at about -66
	_lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;

	// We have received a message.
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Got one 
    }
    else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
    {
	_txGood++;

	setModeIdle();
    }
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF95.
// 3 interrupts allows us to have 3 different devices
void RH_RF95::isr0(void* args)
{
    if (_RF95ForInterrupt[0])
		_RF95ForInterrupt[0]->handleInterrupt();
}
void RH_RF95::isr1(void* args)
{
    if (_RF95ForInterrupt[1])
		_RF95ForInterrupt[1]->handleInterrupt();
}

uint8_t RH_RF95::spiRead(uint8_t reg)
{
    uint8_t data;
    spiBurstRead (reg, &data, 1);
    return data;
}

void RH_RF95::spiWrite(uint8_t reg, uint8_t val)
{
    spiBurstWrite (reg, &val, 1);
}

void RH_RF95::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    uint8_t *request;
    uint8_t *response;
    
    request =  (uint8_t *) malloc(sizeof(uint8_t) * (len + 1));
    response = (uint8_t *) malloc(sizeof(uint8_t) * (len + 1));
    memset(request,  0x00, len + 1);
	memset(response, 0x00, len + 1);
    
    request[0] = reg & ~RH_SPI_WRITE_MASK;
    memcpy (&request[1], dest, len);
    
    mraa_gpio_write(_cs, 0x1);
	mraa_gpio_write(_cs, 0x0);
	usleep(100);
	mraa_spi_transfer_buf(_spi, request, response, len + 1);
	usleep(100);
	mraa_gpio_write(_cs, 0x1);
    
    memcpy (dest, &response[1], len);
    
    free (request);
    free (response);
}

void RH_RF95::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    uint8_t *request;
    uint8_t *response;
    
    request =  (uint8_t *) malloc(sizeof(uint8_t) * (len + 1));
    response = (uint8_t *) malloc(sizeof(uint8_t) * (len + 1));
    memset(request,  0x00, len + 1);
	memset(response, 0x00, len + 1);
    
    request[0] = reg | RH_SPI_WRITE_MASK;
    memcpy (&request[1], src, len);
    
    mraa_gpio_write(_cs, 0x1);
    mraa_gpio_write(_cs, 0x0);
    usleep(100);
    mraa_spi_transfer_buf(_spi, request, response, len + 1);
    usleep(100);
    mraa_gpio_write(_cs, 0x1);
    
    free (request);
    free (response);
}

// Check whether the latest received message is complete and uncorrupted
void RH_RF95::validateRxBuf()
{
    if (_bufLen < 4)
	return; // Too short to be a real message
    // Extract the 4 headers
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = 1;
    }
}

uint8_t RH_RF95::available()
{
    if (_mode == RHModeTx)
	return 0;
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void RH_RF95::clearRxBuf()
{
    
    _rxBufValid = 0;
    _bufLen = 0;
    
}

uint8_t RH_RF95::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return 0;
    if (buf && len)
    {
	
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_RF95_HEADER_LEN)
	    *len = _bufLen-RH_RF95_HEADER_LEN;
	memcpy(buf, _buf+RH_RF95_HEADER_LEN, *len);
	
    }
    clearRxBuf(); // This message accepted and cleared
    return 1;
}

uint8_t RH_RF95::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_RF95_MAX_MESSAGE_LEN)
	return 0;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    // Position at the beginning of the FIFO
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return 1;
}

uint8_t RH_RF95::printRegisters()
{
/*#ifdef RH_HAVE_SERIAL
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
    {
	Serial.print(registers[i], HEX);
	Serial.print(": ");
	Serial.println(spiRead(registers[i]), HEX);
    }
#endif*/
    return 1;
}

uint8_t RH_RF95::maxMessageLength()
{
    return RH_RF95_MAX_MESSAGE_LEN;
}

uint8_t RH_RF95::setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

    return true;
}

void RH_RF95::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
//	std::cout << "setModeIdle write" << std::to_string(RH_RF95_MODE_STDBY) << std::to_string(spiRead(RH_RF95_REG_40_DIO_MAPPING1)) << std::endl;
//	std::cout << "setModeIdle " << std::to_string(spiRead(RH_RF95_REG_01_OP_MODE)) << " " << std::to_string(spiRead(RH_RF95_REG_40_DIO_MAPPING1)) << std::endl;
	_mode = RHModeIdle;
    }
}

uint8_t RH_RF95::sleep()
{
    if (_mode != RHModeSleep)
    {
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
	_mode = RHModeSleep;
    }
    return 1;
}

void RH_RF95::setModeRx()
{
    if (_mode != RHModeRx)
    {
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
	spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
//	std::cout << "setModeRx write" << std::to_string(RH_RF95_MODE_RXCONTINUOUS) << " " << std::to_string(0x00) << std::endl;
//	std::cout << "setModeRx " << std::to_string(spiRead(RH_RF95_REG_01_OP_MODE)) << " " << std::to_string(spiRead(RH_RF95_REG_40_DIO_MAPPING1)) << std::endl;
	_mode = RHModeRx;
    }
}

void RH_RF95::setModeTx()
{
    if (_mode != RHModeTx)
    {
	spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
	spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
//	std::cout << "setModeRx write" << std::to_string(RH_RF95_MODE_TX) << " " << std::to_string(0x40) << std::endl;
//	std::cout << "setModeTx " << std::to_string(spiRead(RH_RF95_REG_01_OP_MODE)) << " " << std::to_string(spiRead(RH_RF95_REG_40_DIO_MAPPING1)) << std::endl;
	_mode = RHModeTx;
    }
}

void RH_RF95::setTxPower(int8_t power)
{
    if (power > 23)
	power = 23;
    if (power < 5)
	power = 5;

    // For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
    // RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
    // for 21, 22 and 23dBm
    if (power > 20)
    {
	spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
	power -= 3;
    }
    else
    {
	spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
    }

    // RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
    // pin is connected, so must use PA_BOOST
    // Pout = 2 + OutputPower.
    // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
    // but OutputPower claims it would be 17dBm.
    // My measurements show 20dBm is correct
    spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
}

// Sets registers from a canned modem configuration structure
void RH_RF95::setModemRegisters(const ModemConfig* config)
{
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    spiWrite(RH_RF95_REG_26_MODEM_CONFIG3,       config->reg_26);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
uint8_t RH_RF95::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return 0;

    ModemConfig cfg;
    memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF95::ModemConfig));
    setModemRegisters(&cfg);

    return 1;
}

void RH_RF95::setPreambleLength(uint16_t bytes)
{
    spiWrite(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

