/*
 * sonoff_pipe.cpp
 *
 *  Created on: 29 Aug 2019
 *      Author: jcera
 */
#include <stdio.h>
#include <string.h>

#include "sonoff_pipe.h"
#include "stm32f1xx_hal.h"

SonoffPipe::SonoffPipe(int (*transmit_cb)(uint8_t *buffer, int len)) : transmitCB(transmit_cb)
{
	mHead = 0;
	mTail = 0;
	mKeepAliveTick = 0;
	mSonoffOK = false;
	mState = IDLE;
}

SonoffPipe::~SonoffPipe()
{
}

void SonoffPipe::handleByte(uint8_t byte)
{
	mBuffer[mHead] = byte;
	mHead = (mHead + 1) % 128;
}

void SonoffPipe::run()
{
	while(mHead != mTail)
	{
		line[idx] = mBuffer[mTail];
		mTail = (mTail + 1) % 128;

		if((line[idx] == '\n') || (line[idx] == '\r'))
		{
			if(idx > 1)
			{
				line[idx] = 0;
				handleLine((const char*)line);
			}
			idx = 0;
		}
		else
		{
			idx++;
		}

		if(idx >= 128)
			idx = 0;
	}

	if(mKeepAliveTick < HAL_GetTick())
	{
		mKeepAliveTick = HAL_GetTick() + 30000;
		if(mState == WAIT_OK)
		{
			printf("Sonoff timed out\n");
			mState = EXIT_PY;
		}
		mState = CHECK_STATE;
	}

	switch(mState)
	{
		case IDLE:
			break;
		case CHECK_STATE:
		{
			printf("Check Sonoff state\n");
			mSonoffOK = false;
			uint8_t buff[] = {"hello\n"};
			if(transmitCB(buff, 6))
			{
				mState = WAIT_OK;
			}
		}
		break;
		case WAIT_OK:
		{
			if(mSonoffOK)
			{
				printf("OK replied\n");
				mState = IDLE;
			}
		}
		break;
		case EXIT_PY:
		{
			mSonoffOK = false;
			uint8_t buff[] = {"exit\n"};
			if(transmitCB(buff, 6))
			{
				printf("Waiting for Sonoff terminal\n");
				mState = WAIT_TERMINAL;
			}
		}
			break;
		case WAIT_TERMINAL:
		{

		}
		break;
		case RESET:
		{

		}
		break;
	}
}

void SonoffPipe::handleLine(const char *line)
{
	printf("sonoff_RX %s\n", line);
	if(!strncmp(line, "KO", 2))
	{
		mSonoffOK = true;
	}
}

void SonoffPipe::checkSonoff()
{
	printf("Check Sonoff connection\n");


}
