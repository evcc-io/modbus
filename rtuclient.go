// Copyright 2014 Quoc-Viet Nguyen. All rights reserved.
// This software may be modified and distributed under the terms
// of the BSD license. See the LICENSE file for details.

package modbus

import (
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"time"
)

const (
	rtuMinSize = 4
	rtuMaxSize = 256

	rtuExceptionSize = 5
)

const (
	stateSlaveID = 1 << iota
	stateFunctionCode
	stateReadLength
	stateReadPayload
	stateCRC
)

const (
	readCoilsFunctionCode           = 0x01
	readDiscreteInputsFunctionCode  = 0x02
	readHoldingRegisterFunctionCode = 0x03
	readInputRegisterFunctionCode   = 0x04

	writeSingleCoilFunctionCode       = 0x05
	writeSingleRegisterFunctionCode   = 0x06
	writeMultipleRegisterFunctionCode = 0x10
	writeMultipleCoilsFunctionCode    = 0x0F
	maskWriteRegisterFunctionCode     = 0x16

	readWriteMultipleRegisterFunctionCode = 0x17
	readFifoQueueFunctionCode             = 0x18
)

// RTUClientHandler implements Packager and Transporter interface.
type RTUClientHandler struct {
	rtuPackager
	*rtuSerialTransporter
}

// NewRTUClientHandler allocates and initializes a RTUClientHandler.
func NewRTUClientHandler(address string) *RTUClientHandler {
	return &RTUClientHandler{
		rtuSerialTransporter: &rtuSerialTransporter{
			serialPort: defaultSerialPort(address),
		},
	}
}

// RTUClient creates RTU client with default handler and given connect string.
func RTUClient(address string) Client {
	handler := NewRTUClientHandler(address)
	return NewClient(handler)
}

// Clone creates a new client handler with the same underlying shared transport.
func (mb *RTUClientHandler) Clone() *RTUClientHandler {
	return &RTUClientHandler{
		rtuSerialTransporter: mb.rtuSerialTransporter,
	}
}

// rtuPackager implements Packager interface.
type rtuPackager struct {
	SlaveID byte
}

// SetSlave sets modbus slave id for the next client operations
func (mb *rtuPackager) SetSlave(slaveID byte) {
	mb.SlaveID = slaveID
}

// Encode encodes PDU in an RTU frame:
//
//	Slave Address   : 1 byte
//	Function        : 1 byte
//	Data            : 0 up to 252 bytes
//	CRC             : 2 byte
func (mb *rtuPackager) Encode(pdu *ProtocolDataUnit) (adu []byte, err error) {
	length := len(pdu.Data) + 4
	if length > rtuMaxSize {
		err = fmt.Errorf("modbus: length of data '%v' must not be bigger than '%v'", length, rtuMaxSize)
		return
	}
	adu = make([]byte, length)

	adu[0] = mb.SlaveID
	adu[1] = pdu.FunctionCode
	copy(adu[2:], pdu.Data)

	// Append crc
	var crc crc
	crc.reset().pushBytes(adu[0 : length-2])
	checksum := crc.value()

	adu[length-1] = byte(checksum >> 8)
	adu[length-2] = byte(checksum)
	return
}

// Verify verifies response length and slave id.
func (mb *rtuPackager) Verify(aduRequest []byte, aduResponse []byte) (err error) {
	length := len(aduResponse)
	// Minimum size (including address, function and CRC)
	if length < rtuMinSize {
		err = fmt.Errorf("modbus: response length '%v' does not meet minimum '%v'", length, rtuMinSize)
		return
	}
	// Slave address must match
	if aduResponse[0] != aduRequest[0] {
		err = fmt.Errorf("modbus: response slave id '%v' does not match request '%v'", aduResponse[0], aduRequest[0])
		return
	}
	return
}

// Decode extracts PDU from RTU frame and verify CRC.
func (mb *rtuPackager) Decode(adu []byte) (pdu *ProtocolDataUnit, err error) {
	length := len(adu)
	// Calculate checksum
	var crc crc
	crc.reset().pushBytes(adu[0 : length-2])
	checksum := uint16(adu[length-1])<<8 | uint16(adu[length-2])
	if checksum != crc.value() {
		err = fmt.Errorf("modbus: response crc '%v' does not match expected '%v'", checksum, crc.value())
		return
	}
	// Function code & data
	pdu = &ProtocolDataUnit{}
	pdu.FunctionCode = adu[1]
	pdu.Data = adu[2 : length-2]
	return
}

// rtuSerialTransporter implements Transporter interface.
type rtuSerialTransporter struct {
	serialPort
}

// InvalidLengthError is returned by readIncrementally when the modbus response would overflow buffer
// implemented to simplify testing
type InvalidLengthError struct {
	length byte // length received which triggered the error
}

// Error implements the error interface
func (e *InvalidLengthError) Error() string {
	return fmt.Sprintf("invalid length received: %d", e.length)
}

// readIncrementally reads incrementally
func readIncrementally(slaveID, functionCode byte, r io.Reader, deadline time.Time) ([]byte, error) {
	if r == nil {
		return nil, fmt.Errorf("reader is nil")
	}

	buf := make([]byte, 1)
	data := make([]byte, 0, rtuMaxSize)

	state := stateSlaveID
	var length, toRead byte
	var crcCount int

	for {
		if time.Now().After(deadline) { // Possible that serialport may spew data
			return nil, errors.New("failed to read from serial port within deadline")
		}

		if _, err := io.ReadAtLeast(r, buf, 1); err != nil {
			return nil, err
		}

		switch state {
		// expecting slaveID
		case stateSlaveID:
			// read slaveID
			if buf[0] == slaveID {
				state = stateFunctionCode
				data = append(data, buf[0])
			}

		case stateFunctionCode:
			// read function code
			if buf[0] == functionCode {
				switch functionCode {
				case readDiscreteInputsFunctionCode,
					readCoilsFunctionCode,
					readHoldingRegisterFunctionCode,
					readInputRegisterFunctionCode,
					readWriteMultipleRegisterFunctionCode,
					readFifoQueueFunctionCode:

					state = stateReadLength
				case writeSingleCoilFunctionCode,
					writeSingleRegisterFunctionCode,
					writeMultipleRegisterFunctionCode,
					writeMultipleCoilsFunctionCode:

					state = stateReadPayload
					toRead = 4
				case maskWriteRegisterFunctionCode:
					state = stateReadPayload
					toRead = 6
				default:
					return nil, fmt.Errorf("functioncode not handled: %d", functionCode)
				}
				data = append(data, buf[0])
			} else if buf[0] == functionCode+0x80 {
				state = stateReadPayload
				toRead = 1 // only exception code left to read
				data = append(data, buf[0])
			}

		case stateReadLength:
			// read length byte
			length = buf[0]
			// max length = rtuMaxSize - SlaveID(1) - FunctionCode(1) - length(1) - CRC(2)
			if length > rtuMaxSize-5 || length == 0 {
				return nil, &InvalidLengthError{length: length}
			}

			state = stateReadPayload
			toRead = length
			data = append(data, length)

		case stateReadPayload:
			// read payload
			data = append(data, buf[0])
			toRead--
			if toRead == 0 {
				state = stateCRC
			}

		case stateCRC:
			// read crc
			data = append(data, buf[0])
			crcCount++
			if crcCount == 2 {
				return data, nil
			}
		}
	}
}

func (mb *rtuSerialTransporter) Send(aduRequest []byte) (aduResponse []byte, err error) {
	mb.mu.Lock()
	defer mb.mu.Unlock()

	// Make sure port is connected
	if err = mb.connect(); err != nil {
		return
	}

	// Wait for previous frame delay to elapse
	time.Sleep(time.Until(mb.lastActivity.Add(2 * mb.frameDelay())))
	defer func() { mb.lastActivity = time.Now() }()

	// Start the timer to close when idle
	mb.lastActivity = time.Now()
	mb.startCloseTimer()

	// Send the request
	mb.logf("modbus: send % x\n", aduRequest)
	if _, err = mb.port.Write(aduRequest); err != nil {
		return
	}

	bytesToRead := calculateResponseLength(aduRequest)
	time.Sleep(mb.calculateDelay(len(aduRequest) + bytesToRead))

	data, err := readIncrementally(aduRequest[0], aduRequest[1], mb.port, time.Now().Add(mb.Config.Timeout))
	mb.logf("modbus: recv % x\n", data[:])
	aduResponse = data
	return
}

// calculateDelay roughly calculates time needed for the next frame.
// See MODBUS over Serial Line - Specification and Implementation Guide (page 13).
func (mb *rtuSerialTransporter) calculateDelay(chars int) time.Duration {
	characterDuration := 11000000 / mb.BaudRate

	var characterDelay int // us
	if mb.BaudRate <= 0 || mb.BaudRate > 19200 {
		characterDelay = 750
	} else {
		characterDelay = 16500000 / mb.BaudRate
	}

	return time.Duration((characterDuration+characterDelay)*chars)*time.Microsecond + mb.frameDelay()
}

// frameDelay returns the required minimum delay at the start and and the end of each frame.
// See MODBUS over Serial Line - Specification and Implementation Guide (page 13).
func (mb *rtuSerialTransporter) frameDelay() time.Duration {
	var fd int // µs
	if mb.BaudRate <= 0 || mb.BaudRate > 19200 {
		fd = 1750
	} else {
		fd = 38500000 / mb.BaudRate
	}
	return time.Duration(fd) * time.Microsecond
}

func calculateResponseLength(adu []byte) int {
	length := rtuMinSize
	switch adu[1] {
	case FuncCodeReadDiscreteInputs,
		FuncCodeReadCoils:
		count := int(binary.BigEndian.Uint16(adu[4:]))
		length += 1 + count/8
		if count%8 != 0 {
			length++
		}
	case FuncCodeReadInputRegisters,
		FuncCodeReadHoldingRegisters,
		FuncCodeReadWriteMultipleRegisters:
		count := int(binary.BigEndian.Uint16(adu[4:]))
		length += 1 + count*2
	case FuncCodeWriteSingleCoil,
		FuncCodeWriteMultipleCoils,
		FuncCodeWriteSingleRegister,
		FuncCodeWriteMultipleRegisters:
		length += 4
	case FuncCodeMaskWriteRegister:
		length += 6
	case FuncCodeReadFIFOQueue:
		// undetermined
	default:
	}
	return length
}
