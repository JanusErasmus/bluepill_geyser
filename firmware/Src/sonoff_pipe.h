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
		IDLE,
		CHECK_STATE,
		WAIT_OK,
		EXIT_PY,
		WAIT_TERMINAL,
		RESET
	}mState;
	uint8_t mBuffer[128];
	uint8_t line[128];
	int idx = 0;
	int mHead;
	int mTail;
	int (*transmitCB)(uint8_t *buffer, int len);
	uint32_t mKeepAliveTick;
	bool mSonoffOK;
	void checkSonoff();
	void handleLine(const char* line);

public:
	SonoffPipe(int (*transmit_cb)(uint8_t *buffer, int len));
	virtual ~SonoffPipe();

	void handleByte(uint8_t byte);
	void run();
};

#endif /* SRC_SONOFF_PIPE_H_ */
