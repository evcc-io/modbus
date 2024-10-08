// Copyright 2014 Quoc-Viet Nguyen. All rights reserved.
// This software may be modified and distributed under the terms
// of the BSD license. See the LICENSE file for details.

package modbus

import (
	"bytes"
	"reflect"
	"testing"
)

func TestRTUEncoding(t *testing.T) {
	encoder := rtuPackager{}
	encoder.SlaveID = 0x01

	pdu := ProtocolDataUnit{}
	pdu.FunctionCode = 0x03
	pdu.Data = []byte{0x50, 0x00, 0x00, 0x18}

	adu, err := encoder.Encode(&pdu)
	if err != nil {
		t.Fatal(err)
	}
	expected := []byte{0x01, 0x03, 0x50, 0x00, 0x00, 0x18, 0x54, 0xC0}
	if !bytes.Equal(expected, adu) {
		t.Fatalf("adu: expected %v, actual %v", expected, adu)
	}
}

func TestRTUDecoding(t *testing.T) {
	decoder := rtuPackager{}
	adu := []byte{0x01, 0x10, 0x8A, 0x00, 0x00, 0x03, 0xAA, 0x10}

	pdu, err := decoder.Decode(adu)
	if err != nil {
		t.Fatal(err)
	}

	if pdu.FunctionCode != 16 {
		t.Fatalf("Function code: expected %v, actual %v", 16, pdu.FunctionCode)
	}
	expected := []byte{0x8A, 0x00, 0x00, 0x03}
	if !bytes.Equal(expected, pdu.Data) {
		t.Fatalf("Data: expected %v, actual %v", expected, pdu.Data)
	}
}

var responseLengthTests = []struct {
	adu    []byte
	length int
}{
	{[]byte{4, 1, 0, 0xA, 0, 0xD, 0xDD, 0x98}, 7},
	{[]byte{4, 2, 0, 0xA, 0, 0xD, 0x99, 0x98}, 7},
	{[]byte{1, 3, 0, 0, 0, 2, 0xC4, 0xB}, 9},
	{[]byte{0x11, 5, 0, 0xAC, 0xFF, 0, 0x4E, 0x8B}, 8},
	{[]byte{0x11, 6, 0, 1, 0, 3, 0x9A, 0x9B}, 8},
	{[]byte{0x11, 0xF, 0, 0x13, 0, 0xA, 2, 0xCD, 1, 0xBF, 0xB}, 8},
	{[]byte{0x11, 0x10, 0, 1, 0, 2, 4, 0, 0xA, 1, 2, 0xC6, 0xF0}, 8},
}

func TestCalculateResponseLength(t *testing.T) {
	for _, input := range responseLengthTests {
		output := calculateResponseLength(input.adu)
		if output != input.length {
			t.Errorf("Response length of %x: expected %v, actual: %v",
				input.adu, input.length, output)
		}
	}
}

func BenchmarkRTUEncoder(b *testing.B) {
	encoder := rtuPackager{
		SlaveID: 10,
	}
	pdu := ProtocolDataUnit{
		FunctionCode: 1,
		Data:         []byte{2, 3, 4, 5, 6, 7, 8, 9},
	}
	for i := 0; i < b.N; i++ {
		_, err := encoder.Encode(&pdu)
		if err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkRTUDecoder(b *testing.B) {
	decoder := rtuPackager{
		SlaveID: 10,
	}
	adu := []byte{0x01, 0x10, 0x8A, 0x00, 0x00, 0x03, 0xAA, 0x10}
	for i := 0; i < b.N; i++ {
		_, err := decoder.Decode(adu)
		if err != nil {
			b.Fatal(err)
		}
	}
}

func Test_readIncrementally(t *testing.T) {
	testcases := []struct {
		description           string
		slaveID, functionCode byte
		data                  []byte
		want                  []byte
		wantErr               bool
		expectedErr           interface{}
	}{
		{
			description:  "valid response",
			slaveID:      0x05,
			functionCode: 0x03,
			data: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
			want: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
		},
		{
			description:  "2 bytes too much at the end",
			slaveID:      0x05,
			functionCode: 0x03,
			data: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7, 0xFF, 0x98,
			},
			want: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
		},
		{
			description:  "2 bytes too much in front",
			slaveID:      0x05,
			functionCode: 0x03,
			data: []byte{
				0xff, 0x89,
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
			want: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
		},
		{
			description:  "2 bytes too much in front and end",
			slaveID:      0x05,
			functionCode: 0x03,
			data: []byte{
				0xff, 0x89,
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
				0xff, 0x89,
			},
			want: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
		},
		{
			description:  "three times the slaveID in front",
			slaveID:      0x05,
			functionCode: 0x03,
			data: []byte{
				0x05, 0x05,
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
				0xff, 0x89,
			},
			want: []byte{
				0x05, 0x03, 0x2c, 0x80, 0x00, 0x16, 0xc4, 0x68,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0,
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0xd7,
			},
		},
		{
			description:  "not enough data",
			slaveID:      0x0F,
			functionCode: 0x03,
			data:         []byte{0x05, 0x03, 0x01},
			wantErr:      true,
		},
		{
			description:  "max data length", // rtuMaxSize(256) - SlaveID(1) - FunctionCode(1) - length(1) - CRC(2) = 251 (FB)
			slaveID:      0x05,
			functionCode: 0x03,
			data: []byte{
				0x05, 0x03, 0xFB,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 10
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 20
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 30
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 40
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 50
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 60
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 70
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 80
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 90
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // a0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // b0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // c0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // d0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // e0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // f0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x7e, 0xf9, 0xea,
				0xd7, 0xd7, // crc (not valid but not tested)
			},
			want: []byte{
				0x05, 0x03, 0xFB,
				0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0xec, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 10
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 20
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 30
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 40
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 50
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 60
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 70
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 80
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // 90
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // a0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // b0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // c0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // d0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // e0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x9b, 0x37, 0x6c, 0x42, 0xe7, 0xbc, 0x0e, 0xd0, // f0
				0x7e, 0xf9, 0xea, 0xff, 0x00, 0x00, 0x04, 0x67,
				0x7e, 0xf9, 0xea,
				0xd7, 0xd7, // crc (not valid but not tested)
			},
		},
		{
			description:  "data over buffer size",
			slaveID:      0x0F,
			functionCode: 0x03,
			data:         []byte{0x0F, 0x03, 0xFC, 0x01},
			wantErr:      true,
			expectedErr:  &InvalidLengthError{length: 0xFC},
		},
		{
			description:  "slave returns length of 0",
			slaveID:      0x0F,
			functionCode: 0x03,
			data:         []byte{0x0F, 0x03, 0x00, 0x01},
			wantErr:      true,
			expectedErr:  &InvalidLengthError{length: 0x00},
		},
	}
	for _, tc := range testcases {
		t.Run(tc.description, func(t *testing.T) {
			got, err := readIncrementally(tc.slaveID, tc.functionCode, bytes.NewBuffer(tc.data), 0)
			if tc.wantErr {
				if err == nil {
					t.Fatalf("expected error but did not get one")
				} else if tc.expectedErr != nil && !reflect.DeepEqual(err, tc.expectedErr) {
					t.Fatalf("expected error %#v but got %#v", tc.expectedErr, err)
				}
			} else {
				if err != nil {
					t.Fatalf("got unexpected error: %+v", err)
				}

				if !bytes.Equal(tc.want, got) {
					t.Errorf("unexpected response: want [% x], but got [% x]", tc.want, got)
				}
			}
		})
	}
}
