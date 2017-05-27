#include "Arduino.h"

#include "BSerial.h"
#include <SoftwareSerial.h>
#include <Math.h>

BSerial::BSerial(unsigned char rxPort, unsigned char txPort) {
	this->rxPort = rxPort;
	this->txPort = txPort;

	pinMode(this->rxPort, INPUT);
	pinMode(this->txPort, OUTPUT);
	this->serialSw = new SoftwareSerial(rxPort, rxPort);
	this->baudeRate = 0;

	this->bufferPointer = this->bufferSize = this->parsingMessageLength = this->tempChuckBuffer = this->parsingReverseCounter = 0;

	this->cStatus = BSerial::parsingStep::waiting;

	this->timeoutTimer.setInterval(BSERIAL_TIMEOUT_TIME);
	this->timeoutTimer.setCallback(this);

	this->resetStatus();
}

void BSerial::begin(long baude) {
	this->baudeRate = baude;
	this->serialSw->begin(this->baudeRate);
}

void BSerial::check() {
	char chunck;
	while (this->serialSw->available() > 0) {
		chunck = this->serialSw->read();
		this->manageChunck(chunck);

	}

}

void BSerial::manageChunck(char chunck) {
	// check if new message is starting
	if (chunck == BSERIAL_MSG_START
			&& this->cStatus == BSerial::parsingStep::waiting) {
		this->cStatus = BSerial::parsingStep::osurrend;
		return;
	}
	// if message is started
	if (this->cStatus == BSerial::parsingStep::osurrend|| this->cStatus == BSerial::parsingStep::csurrend) {
		if (chunck == BSERIAL_MSG_DELIMITER && this->cStatus == BSerial::parsingStep::osurrend) {
			this->cStatus = BSerial::parsingStep::lenght;
			this->parsingReverseCounter = BSERIAL_LEGTH_BYTE;
			this->parsingMessageLength = 0;
		} else if (chunck == BSERIAL_MSG_DELIMITER && this->cStatus == BSerial::parsingStep::csurrend) {
			this->cStatus = BSerial::parsingStep::close;
		}
		else{
			// invalid char, nothing to do
			this->cStatus = BSerial::parsingStep::waiting;
		}
		return;
	}



	if (this->cStatus == BSerial::parsingStep::lenght) {
		if (this->parsingReverseCounter > 0) {
			this->parsingMessageLength += chunck * (unsigned int) (pow(256, this->parsingReverseCounter - 1) + 0.5);
			this->parsingReverseCounter--;
		}
		if (this->parsingReverseCounter <= 0) {
			this->cStatus = BSerial::parsingStep::id;
			this->parsingReverseCounter = BSERIAL_ID_BYTE;
		}
		return;
	}

	if (this->cStatus == BSerial::parsingStep::id) {
		if (this->parsingReverseCounter > 0) {
			this->cCommand.id += chunck * (unsigned int) (pow(256, this->parsingReverseCounter - 1) + 0.5);
			this->parsingReverseCounter--;
		}
		if (this->parsingReverseCounter <= 0) {
			this->cStatus = BSerial::parsingStep::target;
			this->parsingReverseCounter = BSERIAL_TARGET_BYTE;
			this->tempChuckBuffer = 0;
		}
		return;
	}

	if (this->cStatus == BSerial::parsingStep::target) {
		if (this->parsingReverseCounter > 0) {
			this->cCommand.target += chunck * (pow(256, this->parsingReverseCounter - 1) + 0.5);
			this->parsingReverseCounter--;
		}
		if (this->parsingReverseCounter <= 0) {
			this->cStatus = BSerial::parsingStep::mode;
			this->parsingReverseCounter = BSERIAL_MODALITY_BYTE;
			this->tempChuckBuffer = 0;
		}
		return;
	}

	if (this->cStatus == BSerial::parsingStep::mode) {
		if (this->parsingReverseCounter > 0) {
			this->tempChuckBuffer = 0.5 + chunck * pow(256, this->parsingReverseCounter - 1);
			this->parsingReverseCounter--;
		}
		if (this->parsingReverseCounter <= 0) {
			this->cCommand.action = this->tempChuckBuffer == 0 ? RequestModality::read : RequestModality::write;
			this->cStatus = BSerial::parsingStep::code;
			this->parsingReverseCounter = BSERIAL_CODE_BYTE;
			this->cCommand.code = 0;
		}
		return;
	}

	if (this->cStatus == BSerial::parsingStep::code) {
		if (this->parsingReverseCounter > 0) {
			this->cCommand.code += 0.5
					+ chunck * pow(256, this->parsingReverseCounter - 1);
			this->parsingReverseCounter--;
		}
		if (this->parsingReverseCounter <= 0) {
			this->cStatus = BSerial::parsingStep::message;
			if (this->parsingMessageLength > 10) {
				this->parsingReverseCounter = this->parsingMessageLength - (BSERIAL_ID_BYTE + BSERIAL_CODE_BYTE + BSERIAL_TARGET_BYTE + BSERIAL_MODALITY_BYTE);
				this->cCommand.msgSize = this->parsingReverseCounter;
				this->cCommand.msg = new char[this->cCommand.msgSize + 1];
			} else {
				// no message to get
				this->cStatus = BSerial::parsingStep::csurrend;
			}
		}
		return;

	}

	if (this->cStatus == BSerial::parsingStep::message) {
		if (this->parsingReverseCounter > 0) {
			this->cCommand.msg[this->cCommand.msgSize - this->parsingReverseCounter] = chunck;
			this->parsingReverseCounter--;
		}
		if (this->parsingReverseCounter <= 0) {
			this->cCommand.msg[this->cCommand.msgSize] = '\0';
			this->cStatus = BSerial::parsingStep::csurrend;

		}
		return;

	}

	if (this->cStatus == BSerial::parsingStep::close) {
		// done!
		this->cStatus = BSerial::parsingStep::waiting;

		if(BSERIAL_MSG_BUFFER_SIZE > this->bufferSize){
			this->msgBuffer[this->bufferPointer] = (BSerialComand*) malloc(sizeof(BSerialComand));
			*(this->msgBuffer[this->bufferPointer]) = this->cCommand;
			this->bufferSize ++;
			this->bufferPointer = (this->bufferPointer + 1) % BSERIAL_MSG_BUFFER_SIZE;
		}

		this->cCommand.msg = 0;

		this->resetStatus();
	}
}


BSerial::BSerialComand* BSerial::get(){
	if(this->bufferSize > 0){
		unsigned char indexOffset = this->bufferPointer - this->bufferSize >= 0 ? 0 : BSERIAL_MSG_BUFFER_SIZE;
		return this->msgBuffer[this->bufferPointer - this->bufferSize + indexOffset];
	}
	return 0;

}
BSerial::BSerialComand* BSerial::pop(){
	if(this->bufferSize > 0){
		BSerialComand* temp = this->get();
		this->bufferSize --;
		return temp;
	}
	return 0;
}


unsigned char BSerial::available() {
	return this->bufferSize;
}

void BSerial::resetStatus(){
	if(this->cCommand.msg != 0){
			delete [] this->cCommand.msg;
		}


		this->cCommand.action = RequestModality::read;
		this->cCommand.code = 0;
		this->cCommand.target = 0;
		this->cCommand.msg = 0;
		this->cCommand.msgSize = 0;

	this->cStatus = BSerial::parsingStep::waiting;
}

void BSerial::timerCallback(BTimer*){
		this->resetStatus();
}


