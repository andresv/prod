/*
 * Copyright (c) 2008-2012, SOWNet Technologies B.V.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "Assert.h"
#include "ChipconRegisters.h"
#include "ChipconRegisterValues.h"
#include "ChipconAssert.h"
#include "HalChipconControl.h"

/**
 * Radio on/off control and configuration.
 * All HalChipconControl commands except init() are only available in between StdControl.start()/stop().
 */
module HalChipconControlP {
	provides {
		interface Init;
		interface HalChipconControl;
		interface Get<cc_hal_status_t*> as Status;
	}

	uses {
		interface Resource as SpiResource;
		interface HplChipconSpi;
		interface GeneralIO as SI;
		interface GeneralIO as SO;
		interface GeneralIO as Clock;
		interface GeneralIO as CSn;
		interface GeneralIO as G0;
		interface GeneralIO as G2;
		interface GpioInterrupt as G0Interrupt;
		interface BusyWait<TMicro, uint16_t>;
		interface ActiveMessageAddress;
		interface Timer<TMilli> as TxTimer;
		interface LocalTime<T32khz>;
	}
}

implementation {

	// Timing values in microseconds taken from the data sheet, Table 28: State Transition Timing,
	// Application Note 038: Wake-On-Radio, and Design Note 505: RSSI Interpretation and Timing
	enum Timings {
		CALIBRATION_TIME = 721,	// additional time taken when calibrating during a state transition
		RX_TO_TX_TIME = 10,			// switching from RX to TX

	};
	
	// limit the amount of time we'll send a preamble to prevent a broken program jamming everyone
	#define TX_PREAMBLE_TIMEOUT 60*1024UL
	
	// if we don't get a txDone within a reasonable time window, assume the interrupt was lost and recover
	#define TX_DATA_TIMEOUT (3 *  8 * MAX_PACKET_LENGTH * 1024UL)/BAUD_RATE 

	// Chip states, Table 17: Status Byte Summary
	enum States {
		STATE_IDLE,
		STATE_RX,
		STATE_TX,
		STATE_FSTXON,
		STATE_CALIBRATE,
		STATE_SETTLING,
		STATE_RXFIFO_OVERFLOW,
		STATE_TXFIFO_UNDERFLOW,
		STATE_OFF,	// extra state to represent the radio being turned off
	};
	
	enum {
		FIFO_SIZE = 64,
		AVAILABLE_BYTES_IN_TX_FIFO = 59 // check FIFOTHR register for clarification
	};

	// indicates if we are currently transmitting
	bool transmitting = FALSE;
	// indicates if we are currently receiving
	bool receiving = FALSE;

	// if packet is longer than TXFIFO size, it counts how many bytes is waiting to be sent
	uint8_t bytesToSend = 0;
	uint8_t* txBuffer = NULL;

	// number of end of packet events pending
	uint8_t pending = 0;
	
	// timestamps of the last end-of-packet interrupt
	uint32_t rxTimeStamp;
	uint32_t txTimeStamp;
	
	// number of packets sent, received and dropped
	cc_hal_status_t status;
	
	/**
	 * Send a command strobe.
	 * @return the chip status byte.
	 */
	uint8_t strobe(uint8_t cmd) {
		return call HplChipconSpi.strobe(cmd);
	}
	
	/**
	 * Extract the chip state from the chip status byte, obtained by sending a NOP strobe.
	 * If the radio is off (CSn is high), this returns STATE_OFF without querying the radio.
	 */
	uint8_t getChipState() {
		uint8_t statusByte;
		
		if (call CSn.get()) {
			// chip select is high, so the radio is off
			return STATE_OFF;
		}

		// chip state is in bits [6:4] of the chip status byte
		statusByte = strobe(SNOP);
		return (statusByte >> 4) & 0x07;
	}
	
	/**
	 * Wait for the voltage control and oscillator to stabilize after
	 * power up or coming out of sleep mode.
	 */
	void waitForChipReady() {
		// chip is ready when SO goes low
		call BusyWait.wait(1); // delay, sometimes SO is not yet raised to fall down
		while (call SO.get());
	}
	
	/**
	 * Read a register.
	 */
	uint8_t readRegister(uint8_t reg) {
		// for status registers (address 0x30 to 0x3E), set the burst bit to distinguish them from command strobes
		if (reg >= 0x30 && reg <= 0x3E) reg |= 0x40;
		
		// set the read flag
		reg |= 0x80;
		
		return call HplChipconSpi.readRegister(reg);
	}
	
	/**
	 * Write a register.
	 */
	void writeRegister(uint8_t reg, uint8_t value) {
		call HplChipconSpi.writeRegister(reg, value);
	}
	
	/**
	 * Send reset command.
	 * @pre not currently transmitting
	 */
	void reset() {
		atomic assertNot(transmitting, ASSERT_CC_HAL_NOT_IDLE);

		/*
		 * Wiggle the pins according to section 19.1 of the data sheet.
		 * We don't set the CLK/SI SIDLE pin control combination (section 11.3)
		 * because
		 * a) it wouldn't work if hardware SPI already has control of the pins,
		 * b) we never activate pin control mode, and
		 * c) even if we did trigger a pin control command (which would be SPWD),
		 *     the radio will be turned off after it is configured, anyway.
		 */
		
		call CSn.set();
		call CSn.clr();
		call CSn.set();
		call BusyWait.wait(45);
		call CSn.clr();
		waitForChipReady();

		strobe(SRES);
		waitForChipReady();
	}
	
	/**
	 * Verify the part number and version registers.
	 */
	void checkChipVersion() {
		uint8_t partnum = readRegister(PARTNUM);
		uint8_t version = readRegister(VERSION);
		assert(partnum == EXPECTED_PARTNUM, ASSERT_CC_HAL_CHIP_VERSION);
		assert(version == EXPECTED_VERSION_CC1101 || version == EXPECTED_VERSION_RF1A, ASSERT_CC_HAL_CHIP_VERSION);
	}
	
	/**
	 * Send calibration command and busy wait for it to finish.
	 */
	void calibrate() {
		strobe(SCAL);
		call BusyWait.wait(CALIBRATION_TIME);
	}

	/**
	 * Load default configuration into registers. 
	 */
	void configure() {
		uint8_t i;
		for (i = 0; i < NUM_REGISTERS * 2; i += 2) {
			uint8_t reg = chipconRegisterValues[i];
			uint8_t value = chipconRegisterValues[i+1];
			call HplChipconSpi.writeRegister(reg, value);
		}
	}
	
	/**
	 * Switch to RX mode.
	 * Depending on radio settings, this may perform calibration first,
	 * so the radio may not go to RX mode immediately.
	 */
	void listen() {
		// Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached.
		// De-asserts when the RX FIFO is empty.
		call HplChipconSpi.writeRegister(IOCFG0, 0x01);
		call G0Interrupt.enableRisingEdge();

		strobe(SRX);
	}
	
	/**
	 * Wait for either the CS (Carrier Senser) or CCA (Clear Channel Assessment) bits
	 * to be set (which indicates the radio has a valid RSSI measurement).
	 */
	bool waitForRssiValid() {
		uint8_t pktstatus;
		while (1) {
			pktstatus = readRegister(PKTSTATUS);

			// sync word received
			// if radio now tries to go to Tx mode (look .tx command), it thinks it is able to do that
			// but actually it fail, status shows "calibrating", however it is because we just received a packet
			// it is not because TX was just issued, so if sync word is received, try sending packet little bit later
			if (pktstatus & (1<<3)) {
				return FALSE;
			}
			// wait until either CS (bit 6) or CCA (bit 4) is asserted in PKTSTATUS
			else if (pktstatus & ((1<<6) | (1<<4))) {
				return TRUE;
			}
			// if CRC_OK is set it means we have just received a packet and should not go to Tx
			// unless it is read out, otherwise it is deadlock because 0xA2 is always read
			else if (pktstatus & (1<<7)) {
				return FALSE;
			}
		}
	}
	
	/**
	 * Set the ADDR register to the low byte of our current ActiveMessage address.
	 */
	void setAddress() {
		// Address can change at any time (for example, GuArtNet changes it during
		// PlatformInit by reading it from flash) and we may not have access to the SPI yet.
		// In that case, just skip it and HalChipconControl.init() will set the address later.
		if (call SpiResource.isOwner()) {
			writeRegister(ADDR, call ActiveMessageAddress.amAddress() & 0xFF);
		}
	}

	/**
	 * Update RX success or drop count
	 */
	inline void updateRxStatistics(bool success) {
		if (success) {
			status.rxCount++;
		}
		else {
			status.dropCount++;
		}
	}

	/**
	 * Get RX FIFO status.
	 * @return the RX FIFO status byte.
	 */
	inline uint8_t getRxFifoStatus() {
		// 0:6 show bytes in RX FIFO
		//   7 shows overflow

		// check errata 1 for implementation details: http://www.ti.com/lit/er/swrz020b/swrz020b.pdf
		uint8_t fifostatus, lastReading;

		fifostatus = readRegister(RXBYTES);
		do {
			lastReading = fifostatus;
			fifostatus = readRegister(RXBYTES);
		} while (fifostatus < 2 && fifostatus != lastReading);

		return fifostatus;
	}

	task void taskSetAddress() {
		setAddress();
	}
	
	async event void ActiveMessageAddress.changed() {
		// post a task so we don't have to worry about using the SPI bus asynchronously
		post taskSetAddress();
	}
	
	/**
	 * Packet sent, check for underflow or radio turned off, post task to signal the event and resume receiving.
	 */
	task void txDone() {
		uint8_t state;
		uint32_t timestamp;
		
		atomic {
			assertEquals(pending, 1, ASSERT_CC_HAL_TOO_MANY_PENDING);
			pending = 0;
			transmitting = FALSE;
			timestamp = txTimeStamp;
			call TxTimer.stop();
		}
		
		dbg("HALRadio", "%s N%u: %s: txDone\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);

		// verify we didn't interpret a received packet as a transmitted packet
		// if there are some bytes in RX FIFO flush them
		if (getRxFifoStatus() != 0) {
			strobe(SIDLE);
			strobe(SFRX);
			platform_printf("Hal: flush: RX_BYTES = %u\n", readRegister(RXBYTES));
		}
		
		state = getChipState();
		if (state == STATE_OFF) {
			signal HalChipconControl.txDone(timestamp, EOFF);
		} else if (state == STATE_TXFIFO_UNDERFLOW) {
			platform_printf("Hal: TX_FIFO_UNDERFLOW\n");
			
			// acknowledge underflow
			strobe(SFTX);

			// go back to receive mode and inform upper layer of failed transmission
			listen();
			signal HalChipconControl.txDone(timestamp, ERETRY);
		} else if (state == STATE_IDLE || state == STATE_CALIBRATE) {
			// return to receive mode and signal successful completion event
			status.txCount++;
			listen();
			signal HalChipconControl.txDone(timestamp, SUCCESS);
		} else {
			// unexpected state, report what state we're in
			// (by comparing to one we know it isn't)
			assertEquals(state, STATE_IDLE, ASSERT_CC_HAL_NOT_IDLE);
		}
	}
	
	/**
	 * Fills TX FIFO again to send next segment of the packet.
	 */
	task void txNextSegment() {
		uint8_t txbytes;
		atomic txbytes = bytesToSend;

		if (txbytes <= AVAILABLE_BYTES_IN_TX_FIFO) {
			call HplChipconSpi.write(TX_BURST_WRITE, txBuffer, txbytes);
			txbytes = 0;
			call CSn.set();
			call CSn.clr();

			// this must be here, right before changing interrupt behavior, because packet done interrupt comes immediately
			// if radio is in UNDERFLOW state
			atomic bytesToSend = txbytes;
			// De-asserts at the end of the packet
			call HplChipconSpi.writeRegister(IOCFG0, 0x06);
		}
		else {
			call HplChipconSpi.write(TX_BURST_WRITE, txBuffer, AVAILABLE_BYTES_IN_TX_FIFO);
			call CSn.set();
			call CSn.clr();

			txbytes -= AVAILABLE_BYTES_IN_TX_FIFO;
			txBuffer += AVAILABLE_BYTES_IN_TX_FIFO;
			atomic bytesToSend = txbytes;
		}
	}

	task void rxWaiting() {
		uint32_t timestamp;
		atomic timestamp = rxTimeStamp;
		signal HalChipconControl.rxWaiting(timestamp);
	}
	
	/**
	 * Set to trigger on the falling edge in TX mode,
	 * 		rising edge in RX mode to get FIFO almost full interrupt and falling edge to get packet received interrupt.
	 * G0 is configured as follows:
	 * TX: 0x02 De-asserts when the TX FIFO is below the same TX FIFO threshold - new bytes must be written soon to FIFO.
	 * TX: 0x06 De-asserts at the end of the packet, last segment was sent.
	 *
	 * RX: 0x00 Asserts when RX FIFO is filled at or above the RX FIFO threshold. Start reading data out or else overflow occurs.
	 * RX: 0x06 De-asserts at the end of the packet, last segment was received.
	 *
	 * So, this signals one of the following events:
	 * - TX FIFO almost empty
	 * - RX FIFO almost full
	 * - packet sent
	 * - packet received
	 * - radio turned off or reset
	 */
	async event void G0Interrupt.fired() {
		// if we don't check the transmitting flag here but in a task, there will be a race condition
		// between the interrupt, interrupt handler, tx() and rx/txDone()
		uint32_t time = call LocalTime.get();
		pending++;
		if (transmitting) {
			if (bytesToSend == 0) {
				txTimeStamp = time;
				post txDone();
			}
			else {
				post txNextSegment();
			}
		} else {
			receiving = TRUE;
			rxTimeStamp = time;
			post rxWaiting();
		}
	}
	
	/**
	 * Configure the radio with default settings, then power it down and release the SPI bus.
	 * @pre: SPI bus unclaimed and available
	 * @return SUCCESS
	 */
	command error_t Init.init() {
		call HalChipconControl.init();
		dbg("HALRadio", "%s N%u: %s: INFO radio HAL init SUCCESS\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);
		return SUCCESS;
	}

	/**
	 * Configure the radio with default settings, then power it down and release the SPI bus.
	 */
	command void HalChipconControl.init() {
		atomic transmitting = FALSE;

		// pin configuration
		// G0 starts out as a clock signal, so don't enable the interrupt until we reconfigure it
		call G0Interrupt.disable();
		call G0.makeInput();
		call G2.makeInput();
		call CSn.makeOutput();
		call CSn.set();

		call SO.makeInput();
		call SI.makeOutput();
		call Clock.makeOutput();
		dbg("HALRadio", "%s N%u: %s: INFO pins initialized\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);

		// enable and initialise the chip
		call HalChipconControl.on();
		dbg("HALRadio", "%s N%u: %s: INFO radio chip powered up\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);
		reset();
		dbg("HALRadio", "%s N%u: %s: INFO radio chip reset\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);
		checkChipVersion();
		//dbg("HALRadio", "%s N%u: %s: INFO radio chip version OK\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);
		configure();
		//dbg("HALRadio", "%s N%u: %s: INFO radio chip configured\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);
		setAddress();
		//dbg("HALRadio", "%s N%u: %s: INFO radio chip address set\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);
		calibrate();
		//dbg("HALRadio", "%s N%u: %s: INFO radio chip calibrated\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);

		// go into sleep mode (power down)
		call HalChipconControl.off();
		//dbg("HALRadio", "%s N%u: %s: INFO radio in sleep mode\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);

		// enable the end of packet interrupt (a spurious event will be ignored since the radio is off)
		//call G0Interrupt.enableFallingEdge();

		// enable the RXFIFO almost full interrupt (a spurious event will be ignored since the radio is off)
		call G0Interrupt.enableRisingEdge();
	}

	/**
	 * Claim the SPI bus, power up the radio chip and go to RX mode.
	 * @pre SPI bus not already owned and immediately available
	 */
	command void HalChipconControl.on() {
		// claim the SPI bus
		error_t error = call SpiResource.immediateRequest();
		assertSuccess(error, ASSERT_CC_HAL_SPI_REQUEST);
		
		// activate the chip select pin, wait for the chip to power up
		call CSn.clr();
		waitForChipReady();
		
		// go to receive mode
		listen();
	}
	
	/**
	 * Power down and release the SPI bus.
	 */
	command void HalChipconControl.off() {
		// go idle, then power down strobe and deactivate the chip enable pin
		strobe(SIDLE);
		strobe(SPWD);
		call CSn.set();

		// release the SPI bus
		call SpiResource.release();
	}

	/**
	 * @return TRUE iff the radio is currently busy transmitting or receiving a packet.
	 */
	command bool HalChipconControl.isBusy() {
		atomic return transmitting || call G0.get() || pending > 0;
	}
	
	/**
	 * Switch to TX mode and send the data in the TX FIFO. If the FIFO is empty,
	 * the radio will start sending a continuous preamble until data is written to the FIFO.
	 * It will only switch to TX if the channel is clear and the radio is not currently receiving
	 * a packet.
	 * 
	 * @pre not already transmitting
	 * @pre radio is in receive mode (to do CCA)
	 * @return:
	 *  	SUCCESS if the switch was made (txDone() will be signalled),
	 * 		EOFF if the radio is off,
	 * 		ERETRY if the radio is calibrating,
	 * 		EBUSY if the radio is busy receiving a packet,
	 * 		FAIL if we could not switch because of CCA
	 */
	command error_t HalChipconControl.tx() {
		uint8_t state = getChipState();
		if (state == STATE_OFF) return EOFF;
		
		atomic assertNot(transmitting, ASSERT_CC_HAL_TX_ALREADY);
		
		// we can only do CCA in RX mode, so check for that
		// if we're idle, calibrating or settling, return ERETRY so the sender will try again
		// else, anything but RX means something is wrong
		if (state == STATE_IDLE || state == STATE_CALIBRATE || state == STATE_SETTLING || state == STATE_RXFIFO_OVERFLOW) {
			// come back later
			return ERETRY;
		} else {
			assertEquals(state, STATE_RX, ASSERT_CC_HAL_CCA_NO_RX);
		}
		
		// Note: we must be very careful not to create a race condition on the
		// transmitting flag and the end-of-packet interrupt.
		// The current approach is as follows:
		// test G0 outside an atomic block:
		// if G0 is high, a packet is being received and we must wait for the interrupt to go off -> EBUSY
		// if G0 is low, we should be clear, but maybe it *just went low* and the interrupt is pending.
		// That is why the test is outside the atomic block: the interrupt handler will run first and increment pending.
		// Then, in the atomic block:
		// 		check if pending > 0; if so, a packet still needs to be handled -> EBUSY
		// 		try to switch to TX (protected by TX-if-CCA)
		// 		if successfully switched, set the transmitting flag

		if (call G0.get()) {
			return EBUSY;
		}
		
		// G0 is low; if this just happened (interrupt pending),
		// the interrupt handler will run before we enter the atomic block
		// and pending will be non-zero
		
		atomic {
			// no pending interrupts - see if any tasks are pending
			// or we get receive interrupt - it might take awhile to get the whole packet
			if (pending > 0 || receiving) {
				return EBUSY;
			}
			
			// If a TX follows quickly after entering RX mode, the RSSI may not be valid yet.
			// This may take about 100 us.
			// Also it ensures that we are not missed Rx interrupt, although (pending > 0) should catch it
			if (!waitForRssiValid()){
				return EBUSY;
			}
			
			// if G0 goes high now, TX-if-CCA will prevent us from switching
			// attempt to switch to TX mode
			strobe(STX);
			
			// wait and see if we actually made the transition
			call BusyWait.wait(RX_TO_TX_TIME);
			
			// check against STATE_RX instead of STATE_TX, just in case we're still in STATE_SETTLING or somesuch
			if (getChipState() == STATE_RX) {
				return FAIL;
			}

			// TX-if-CCA really changed state to TX
			// falling edge is for TX FIFO almost empty interrupt or for packet sent interrupt
			call G0Interrupt.enableFallingEdge();

			// set the flag - next interrupt indicates end of sent whole packet or sent packet fragment
			transmitting = TRUE;
			
			// set a timer to guard against losing the end-of-packet interrupt
			if (call TxTimer.isRunning()) {
				// write() has already been called, stick to the short timeout
			} else {
				// allow for a reasonable length preamble
				call TxTimer.startOneShot(TX_PREAMBLE_TIMEOUT);
			}
		}
		
		return SUCCESS;
	}
	
	/**
	 * Write packet payload into the TX FIFO. If the buffer underflows, this will be detected
	 * by the end-of-packet interrupt handler, which will signal txDone(ERETRY).
	 */
	command void HalChipconControl.write(uint8_t* buffer, uint8_t length) {
		uint8_t txBytes;
		atomic txBytes = bytesToSend;
		
		assert(length == buffer[0] + 1, ASSERT_CC_HAL_INVALID_LENGTH);

		if (length <= FIFO_SIZE) {
			// GDO0 de-asserts (falling) at the end of the packet.
			call HplChipconSpi.writeRegister(IOCFG0, 0x06);

			// write the first byte to start transmitting the sync word
			call HplChipconSpi.writeRegister(TXFIFO, buffer[0]);
		
			// timestamp the start of the packet
			signal HalChipconControl.txStart(call LocalTime.get());

			// data fits into TXFIFO, therefore it can be sent by just filling it once
			call HplChipconSpi.write(TX_BURST_WRITE, buffer+1, length-1);
			txBytes = 0;

		}
		else {
			// GDO0 asserts when the TX FIFO is filled at or above the TX FIFO threshold.
			// De-asserts when the TX FIFO is below the same threshold.
			call HplChipconSpi.writeRegister(IOCFG0, 0x02);

			// write the first byte to start transmitting the sync word
			call HplChipconSpi.writeRegister(TXFIFO, buffer[0]);
		
			// timestamp the start of the packet
			signal HalChipconControl.txStart(call LocalTime.get());

			// data does not fit, therefore TXFIFO must be filled couple of times
			call HplChipconSpi.write(TX_BURST_WRITE, buffer+1, FIFO_SIZE-1); // we already transmitted length byte
			txBytes = length - FIFO_SIZE;
			txBuffer = buffer + FIFO_SIZE;

			// TXFIFO is filled again in GDO0 interrupt
		}

		atomic bytesToSend = txBytes;

		// end burst
		call CSn.set();
		call CSn.clr();

		// If tx() has already been called, the written data will now be sent, or else, tx() will be called soon.
		// Either way, we can expect the end of the packet reasonably soon.
		dbg("HALRadio", "%s N%u: %s: TxTimer started with interval %u\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__, TX_DATA_TIMEOUT);
		call TxTimer.startOneShot(TX_DATA_TIMEOUT);
	}
	
	/**
	 * Put radio back to listen mode.
	 * @return SUCCESS
	 */
	command error_t HalChipconControl.rx() {
		// there are some bytes in FIFO and/or overflow
		atomic pending = 0;
		if (getRxFifoStatus() != 0) {
			strobe(SIDLE);
			strobe(SFRX);
		}

		// we are in IDLE state after receiving
		// so there can not be any receiving interrupt even while this atomic block is executed
		atomic receiving = FALSE;
		listen();
		return SUCCESS;
	}

	/**
	 * Read a segment from the RX FIFO into the buffer.
	 * @pre there is data in the RX FIFO.
	 * @return how many bytes were read from bufferm if there was error return 0xFF.
	 */
	command uint8_t HalChipconControl.read(uint8_t* buffer, uint8_t packet_len, uint8_t bytes_received) {
		uint8_t bytesInFifo = getRxFifoStatus();

		// NOTE: check errata 1 why 1 byte must be left in RX FIFO until packet is completely received:
		// http://www.ti.com/lit/er/swrz020b/swrz020b.pdf

		// wrong interrupt or spi error or RX FIFO overflow
		if ((bytesInFifo & 0x7F) == 0 || (bytesInFifo & 0x7F) > 64 || (bytesInFifo & (1<<7))) {
			return 0xFF;
		}
		else {
			// we have first segment in FIFO (ReceiveP has not received anything yet, so it does not know packet_len yet)
			// it knows about packet_len after this first segment is read out.
			if (bytes_received == 0) {
				uint8_t length = call HplChipconSpi.readRegister(RXFIFO);
				buffer[0] = length;

				if (length >= MAX_PACKET_LENGTH) {
					return 0xFF;
				}

				// +3 is because of prepended len + appended CRC.
				// 		It means this first length byte shows 3 bytes less that is actually received from radio altogether.
				// TX appended real CRC, however now RX changed it to CRC status, LQI and RSSI if it was packet received interrupt,
				// 		not FIFO almost full interrupt.
				if (length + 3 > bytesInFifo) {
					// there must be another interrupt coming, we do not have whole packet in FIFO
					call HplChipconSpi.read(RX_BURST_READ, buffer+1, bytesInFifo-2); // -2 instead of -1 because of errata 1
					call CSn.set();
					call CSn.clr();

					// packet end comes with the next interrupt
					if (length + 3 - bytesInFifo-1 <= FIFO_SIZE) {
						call G0Interrupt.enableFallingEdge();
						call HplChipconSpi.writeRegister(IOCFG0, 0x06);
					}
					// packet end does not come with the next interrupt (RX FIFO should be read out couple of times)
					else {
						call HplChipconSpi.writeRegister(IOCFG0, 0x00);
					}

					// we left 1 byte in FIFO because of errata, those -1 here are every time because of that errata
					// whole FIFO can be read out only if whole packet is received
					return bytesInFifo - 1;
				}
				else {
					// whole packet is in FIFO, read it out
					call HplChipconSpi.read(RX_BURST_READ, buffer+1, bytesInFifo-1); // -1 because we already read out length byte from FIFO
					call CSn.set();
					call CSn.clr();

					// check if CRC status shows CRC match
					updateRxStatistics(*(buffer+bytesInFifo) & 0x80);

					// ReceiveP changes radio into right state after receiving packet
					return bytesInFifo; // we read out length and all other bytes from FIFO
				}
			}

			// we have last segment in FIFO, after reading this out whole packet is received
			else if (packet_len + 3 - bytes_received <= FIFO_SIZE) {
				call HplChipconSpi.read(RX_BURST_READ, buffer, bytesInFifo);
				call CSn.set();
				call CSn.clr();
				
				// check if CRC status shows CRC match
				updateRxStatistics(*(buffer+bytesInFifo) & 0x80);

				return bytesInFifo;
			}

			else {
				call HplChipconSpi.read(RX_BURST_READ, buffer, bytesInFifo-1);
				call CSn.set();
				call CSn.clr();

				if (packet_len + 3 - bytes_received - bytesInFifo > FIFO_SIZE) {
					// more than FIFO_SIZE is coming after that
					// do assert if RXFIFO is almost full
					call HplChipconSpi.writeRegister(IOCFG0, 0x00);
				}
				else {
					// last segment is coming after that
					// de assert with end of packet
					call G0Interrupt.enableFallingEdge();
					call HplChipconSpi.writeRegister(IOCFG0, 0x06);
				}
				return bytesInFifo-1;
			}
		}
	}
	
	/**
	 * Reports whether the channel is clear. Returns TRUE if the radio is not in RX mode.
	 */
	command bool HalChipconControl.isChannelClear() {
		return call G2.get();
	}
	
	/**
	 * Returns the current RSSI if no packet is being received, or the RSSI of the sync word
	 * if a packet is being received.
	 * @pre radio is in RX mode
	 */
	command int16_t HalChipconControl.getRssi() {
		uint8_t state = getChipState();
		int16_t rssi;
		assertEquals(state, STATE_RX, ASSERT_CC_HAL_RSSI_NO_RX);
		rssi = readRegister(RSSI);
		if (rssi > 127) rssi -= 256;
		return rssi/2 - RSSI_OFFSET;
	}
		
	/**
	 * Enable or disable address checking (disabled by default).
	 */
	command void HalChipconControl.useAddressChecking(bool enable) {
		// address checking is set in bits 1:0 of PKTCTRL1
		// 00 is off, 11 is on with both 0x00 and 0xFF accepted as broadcast addresses
		uint8_t setting = readRegister(PKTCTRL1);
		
		if (enable) {
			setting |= 3;
		} else {
			setting &= ~3;
		}
		
		writeRegister(PKTCTRL1, setting);
	}
	
	/**
	 * Set the channel number.
	 * This sets the frequency to the base frequency + (channel number * channel width).
	 * @pre not currently transmitting
	 */
	command void HalChipconControl.setChannel(uint8_t channel) {
		atomic assertNot(transmitting, ASSERT_CC_HAL_NOT_IDLE);
		
		strobe(SIDLE);
		writeRegister(CHANNR, channel);
		listen();
	}

	/**
	 * Enable or disable automatic calibration when going from RX or TX to IDLE (on by default).
	 */
	command void HalChipconControl.autoCalibrate(bool enable) {
		// FS_AUTOCAL is bits 5:4 in MCSM0
		// 00 (0x00) is off (manual calibration only),
		// 01 (0x10) automatically calibrates when going from IDLE to RX or TX
		// 10 (0x20) automatically calibrates when going from RX or TX to IDLE
		uint8_t setting = readRegister(MCSM0);
		
		// clear bits 5:4
		setting &= ~0x30;
		if (enable) {
			setting |= 0x20;
		}
		
		writeRegister(MCSM0, setting);
	}
	
	/**
	 * If not transmitting, go idle, calibrate the radio and resume receiving.
	 * Otherwise, do nothing (keep transmitting).
	 */
	command void HalChipconControl.calibrate() {
		if (transmitting) return;
		strobe(SIDLE);
		calibrate();
		listen();
	}
	
	/**
	 * @return module status: transmitted, received and dropped packet count.
	 */
	command cc_hal_status_t* Status.get() {
		return &status;
	}
	
	/**
	 * Either we incurred a huge delay during sending, or we didn't get a txDone event.
	 * If we were delayed but did get an interrupt, leave it to the txDone task.
	 * Else, generate a txDone event and recover. If the radio is still transmitting, switch back to RX.
	 */
	event void TxTimer.fired() {
		dbg("HALRadio", "%s N%u: %s: INFO TxTimer fired\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__);

		atomic {
			dbg("HALRadio", "%s N%u: %s: ERROR TxTimer fired with pending %u\n", sim_time_string(), TOS_NODE_ID, __FUNCTION__, pending);

			platform_printf("*** HAL: tx timeout!\n");

			assert(transmitting, ASSERT_CC_HAL_NO_TX);
			status.errorCount++;
			transmitting = FALSE;
			pending = 0;
			if (getChipState() == STATE_TX) listen();
			signal HalChipconControl.txDone(call LocalTime.get(), FAIL);
		}
	}

	// we only use immediateRequest()
	event void SpiResource.granted() {}

	default event void HalChipconControl.rxWaiting(uint32_t timestamp) {}	
	default event void HalChipconControl.txStart(uint32_t timestamp) {}
	default event void HalChipconControl.txDone(uint32_t timestamp, error_t error) {}

}
