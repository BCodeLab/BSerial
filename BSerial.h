#ifndef BSerial_h
#define BSerial_h

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <BTimer.h>

// size of the buffer
#define BSERIAL_MSG_BUFFER_SIZE 2
// the timeout (in milliseconds)
#define BSERIAL_TIMEOUT_TIME 1000

/**
 * Message structure:
 *  ^  [1]  start message symbol
 *  *  [1]  message delimiter 1
 *     [2]  length of message
 *     [2]  unique id of the message
 *     [2]  id of the receiver
 *     [4]  modality code [R/W]
 *     [4]  operation code
 *  *  [1]  message delimiter 2
 *  $  [1]  end message symbol
 */
#define BSERIAL_MSG_START 0x5E //^
#define BSERIAL_MSG_DELIMITER 0x2A //*
#define BSERIAL_MSG_END 0x24 //$

#define BSERIAL_LEGTH_BYTE 2
#define BSERIAL_ID_BYTE 2
#define BSERIAL_CODE_BYTE 4
#define BSERIAL_TARGET_BYTE 2
#define BSERIAL_MODALITY_BYTE 4



class BSerial :  public BTimerHandler{
	public:
	// the supported modalities
	enum RequestModality{
		read, write
	};
	//the struct for a parse message
	struct BSerialComand {
		RequestModality action;
		unsigned short int id;
		unsigned short int target;
		unsigned int code;
		unsigned short int msgSize;
		char* msg;
	};

	/*
	 * Constructor, set the rx pin and tx port for the serial communication
	 */
	BSerial(unsigned char rxPort, unsigned char txPort);
	
	/*
	 * Begin the serial port checker, specify yhe baudeRate (9600?)
	 */
	void begin(long baudeRate);

	/**
	 * Function to monitor the serial port.
	 * Put in you loop function
	 */
	void check();

	/**
	 * Return the number of message read not yet poped
	 */
	unsigned char available();

	/**
	 * Reset internal status, partial message will be lost
	 */
	void resetStatus();

	/**
	 * Function required by BTimerHandler
	 */
	void timerCallback(BTimer*);

	/**
	 * Get the first message of the queue
	 */
	BSerialComand* get();
	/**
	 * Get and remove the first message of the queue
	 */
	BSerialComand* pop();
private:

	/**
	 * Function to parse the message
	 */
	void manageChunck(char chunck);
	/**
 	 * Internal status used by manageChunck
 	 */
	enum parsingStep {waiting, osurrend, lenght, id, target, mode, code, message, csurrend, close};

	// the number of the rx pin
	// the number of the tx pin
	unsigned char rxPort, txPort;
	// the selected baude rate
	long baudeRate;
	// internal instance of serial
	SoftwareSerial* serialSw;
	// the container of the parsed messages
	BSerialComand* msgBuffer[BSERIAL_MSG_BUFFER_SIZE];
	// the number of messages in the buffer
	// the index of the last message stored into the buffer
	// the reverse counter for every part of the message, if 0 -> go to next part
	// the length of the current message (taken from the massage
	unsigned char bufferSize, bufferPointer, parsingReverseCounter, parsingMessageLength;
	// aux buffer for the message read from the por
	unsigned long tempChuckBuffer;
	// current parsed message
	BSerialComand cCommand;
	// current parsing status
	parsingStep cStatus;
	// timer to detect timeouts
	BTimer timeoutTimer;
};

#endif
