/*
 * sonoff_pipe.h
 *
 *  Created on: 29 Aug 2019
 *      Author: jcera
 */

#ifndef SRC_SONOFF_PIPE_H_
#define SRC_SONOFF_PIPE_H_
#include <stdint.h>

class SonoffPipe
{
	enum eState
	{
		UNKNOWN,
		IDLE,
		CHECK_STATE,
		WAIT_KO,
		WAIT_OK,
		EXIT_PY,
		WAIT_TERMINAL,
		PUBLISH
	}mState;
	uint8_t mBuffer[128];
	uint8_t line[128];
	int idx = 0;
	int mHead;
	int mTail;
	char *mPublishMessage;
	int (*transmitCB)(uint8_t *buffer, int len);
	uint32_t mKeepAliveTick;
	eState mSonoffReply;
	int  mPromptCount;

	void serviceBuffer();
	void handleLine(const char* line);
	void (*mReceiveCB)(const char* line);

public:
	SonoffPipe(int (*transmit_cb)(uint8_t *buffer, int len));
	virtual ~SonoffPipe();

	void handleByte(uint8_t byte);
	void run();
	void resetSonoff();

	bool publish(const char *message);
	void setReceivedCB(void (*receive_cb)(const char* line));
};

#endif /* SRC_SONOFF_PIPE_H_ */
