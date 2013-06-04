/*
  MeetAndroid.cpp - Arduino Library for Amarino
  Copyright (c) 2009 Bonifaz Kaufmann.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Includes
#include "meet_android.h"

#ifndef F_CPU
#error F_CPU must be defined!
#endif

extern "C" {
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
}

// Private methods
void MeetAndroid::processCommand(){
	if(buffer[0]-FunctionBufferOffset < FunctionBufferLength){
		void (*H_FuncPtr)(uint8_t, uint8_t) = intFunc[buffer[0]-FunctionBufferOffset];
		if (H_FuncPtr != 0) {
			H_FuncPtr(buffer[0], getArrayLength());
		}
		else {
			send("Flag not registered: ");
			send(buffer[0]);
		}
	}
	else {
		if (customErrorFunc)
			errorFunc(buffer[0], getArrayLength());
		else {
			send("Flag out of bounds: ");
			send(buffer[0]);
		}
	}
}


void MeetAndroid::init()
{
	waitTime = 30;
	startFlag = 18;
	ack = 19;
	abort = 27;
	delimiter = 59; //';'

	numberOfValues = 0;
	
	for(int a = 0;a < FunctionBufferLength;a++){
		intFunc[a] = errorFunc;
	}
}


// public methods
MeetAndroid::MeetAndroid(Serial * const ser) 
  : serial(ser), stream(ser->stream())
{
  // it is hard to use member function pointer together with normal function pointers.
  customErrorFunc = false;
  errorFunc = 0;
  init();
}

void MeetAndroid::registerFunction(void(*userfunction)(uint8_t, uint8_t),uint8_t command){
	intFunc[command-FunctionBufferOffset] = userfunction;
}
void MeetAndroid::unregisterFunction(uint8_t command){
	intFunc[command-FunctionBufferOffset] = errorFunc;
}

bool MeetAndroid::receive(){
	uint8_t lastByte;
	bool timeout = false;
	while(!timeout)
	{
		while(serial->available() > 0)
		{
			lastByte = fgetc(stream);
			
			if(lastByte == abort){
				flush();
			}
			else if(lastByte == ack){
				processCommand();
				flush();
			}
			else if(bufferCount < ByteBufferLength){
				buffer[bufferCount] = lastByte;
				bufferCount++;
			}
			else return false;
		}
		
		if(serial->available() <= 0 && !timeout){
		  if(waitTime > 0) _delay_us(30); // TODO: This is hardcoded because IT NEVER CHANGES
		  if(serial->available() <= 0) timeout = true;
		}
	}
	return timeout;
}




void MeetAndroid::getBuffer(uint8_t buf[]){

	for(int a = 0;a < bufferCount;a++){
		buf[a] = buffer[a];
	}
}

void MeetAndroid::getString(char string[]){

	for(int a = 1;a < bufferCount;a++){
		string[a-1] = buffer[a];
	}
	string[bufferCount-1] = '\0';
}

int MeetAndroid::getInt()
{
	uint8_t b[bufferCount];
	for(int a = 1;a < bufferCount;a++){
		b[a-1] = buffer[a];
	}

	b[bufferCount-1] = '\0';
	return atoi((char*)b);
}

long MeetAndroid::getLong()
{
	uint8_t b[bufferCount];
	for(int a = 1;a < bufferCount;a++){
		b[a-1] = buffer[a];
	}

	b[bufferCount-1] = '\0';
	return atol((char*)b);
}

float MeetAndroid::getFloat()
{
	return (float)getDouble();
}

int MeetAndroid::getArrayLength()
{
	if (bufferCount == 1) return 0; // only a flag and ack was sent, not data attached
	numberOfValues = 1;
	// find the amount of values we got
	for (int a=1; a<bufferCount;a++){
		if (buffer[a]==delimiter) numberOfValues++;
	}
	return numberOfValues;
}

void MeetAndroid::getFloatValues(float values[])
{
	int t = 0; // counter for each char based array
	int pos = 0;

	int start = 1; // start of first value
	for (int end=1; end<bufferCount;end++){
		// find end of value
		if (buffer[end]==delimiter) {
			// now we know start and end of a value
			char b[(end-start)+1]; // create container for one value plus '\0'
			t = 0;
			for(int i = start;i < end;i++){
				b[t++] = (char)buffer[i];
			}
			b[t] = '\0';
			values[pos++] = atof(b);
			start = end+1;
		}
	}
	// get the last value
	char b[(bufferCount-start)+1]; // create container for one value plus '\0'
	t = 0;
	for(int i = start;i < bufferCount;i++){
		b[t++] = (char)buffer[i];
	}
	b[t] = '\0';
	values[pos] = atof(b);
}

// not tested yet
void MeetAndroid::getDoubleValues(float values[])
{
	getFloatValues(values);
}

// not tested yet
void MeetAndroid::getIntValues(int values[])
{
	int t = 0; // counter for each char based array
	int pos = 0;

	int start = 1; // start of first value
	for (int end=1; end<bufferCount;end++){
		// find end of value
		if (buffer[end]==delimiter) {
			// now we know start and end of a value
			char b[(end-start)+1]; // create container for one value plus '\0'
			t = 0;
			for(int i = start;i < end;i++){
				b[t++] = (char)buffer[i];
			}
			b[t] = '\0';
			values[pos++] = atoi(b);
			start = end+1;
		}
	}
	// get the last value
	char b[(bufferCount-start)+1]; // create container for one value plus '\0'
	t = 0;
	for(int i = start;i < bufferCount;i++){
		b[t++] = (char)buffer[i];
	}
	b[t] = '\0';
	values[pos] = atoi(b);
}


double MeetAndroid::getDouble()
{
	char b[bufferCount];
	for(int a = 1;a < bufferCount;a++){
		b[a-1] = (char)buffer[a];
	}

	b[bufferCount-1] = '\0';
	return atof(b);	
}

void MeetAndroid::write(uint8_t b){
  fprintf(stream, "%u", b); // Is this the same as the sends?
}

void MeetAndroid::send(char c){
  fputc(startFlag, stream);
  fputc(c, stream);
  fputc(ack, stream);
}

void MeetAndroid::send(const char str[]){
  fputc(startFlag, stream);
  fputs(str, stream);
  fputc(ack, stream);
}
void MeetAndroid::send(uint8_t n){
  fputc(startFlag, stream);
  fprintf(stream, "%u", n);
  fputc(ack, stream);
}
void MeetAndroid::send(int n){
  fputc(startFlag, stream);
  fprintf(stream, "%d", n);
  fputc(ack, stream);
}
void MeetAndroid::send(unsigned int n){
  fputc(startFlag, stream);
  fprintf(stream, "%u", n);
  fputc(ack, stream);
}
void MeetAndroid::send(long n){
  fputc(startFlag, stream);
  fprintf(stream, "%ld", n);
  fputc(ack, stream);
}
void MeetAndroid::send(unsigned long n){
  fputc(startFlag, stream);
  fprintf(stream, "%lu", n);
  fputc(ack, stream);
}
void MeetAndroid::send(long n, int base){
  fputc(startFlag, stream);
  fprintf(stream, "%ld %d", n, base); // TODO: This is almost certainly wrong
  fputc(ack, stream);
}

void MeetAndroid::send(float n){
  fputc(startFlag, stream);
  fprintf(stream, "%f", n);
  fputc(ack, stream);
}
void MeetAndroid::sendln(void){
  fputc(startFlag, stream);
  fputc('\n', stream);
  fputc(ack, stream);
}

void MeetAndroid::flush(){
  for(uint8_t a=0; a < ByteBufferLength; a++){
    buffer[a] = 0;
  }
  bufferCount = 0;
  numberOfValues = 0;
}
