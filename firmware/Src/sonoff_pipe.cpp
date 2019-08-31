/*
 * sonoff_pipe.cpp
 *
 *  Created on: 29 Aug 2019
 *      Author: jcera
 */
#include <stdio.h>
#include <string.h>
#include <string.h>
#include <stdlib.h>

#include "sonoff_pipe.h"
#include "stm32f1xx_hal.h"

SonoffPipe::SonoffPipe(int (*transmit_cb)(uint8_t *buffer, int len)) : transmitCB(transmit_cb)
{
	mHead = 0;
	mTail = 0;
	mKeepAliveTick = 0;
	mSonoffReply = UNKNOWN;
	mState = IDLE;
	mPublishMessage = 0;
	mPromptCount = 0;
	mReceiveCB = 0;
}

SonoffPipe::~SonoffPipe()
{
}

void SonoffPipe::handleByte(uint8_t byte)
{
	mBuffer[mHead] = byte;
	mHead = (mHead + 1) % 128;
}

void SonoffPipe::serviceBuffer()
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
			continue;
		}

		//check if this was a message for this node
		if((line[idx] == '~'))
		{
			if(idx > 1)
			{
				line[idx] = 0;
				if(mReceiveCB)
					mReceiveCB((const char*)line);
			}
			idx = 0;
			continue;
		}

		//check if this was a terminal prompt >>>
		if (idx > 2)
		{
			if(!strncmp((char*)line, ">>>", 3))
			{

				printf("Terminal prompt on sonoff\n");
				mSonoffReply = WAIT_TERMINAL;

				if(mPromptCount > 5)
					mState = EXIT_PY;
			}
		}

		if(idx++ >= 128)
			idx = 0;
	}
}

void SonoffPipe::run()
{
	serviceBuffer();

	if(mKeepAliveTick < HAL_GetTick())
	{
		mKeepAliveTick = HAL_GetTick() + 30000;
		switch(mState)
		{
		case WAIT_OK:
		case WAIT_KO:
		{
			printf("Sonoff hello timed out\n");
			mState = EXIT_PY;
			break;
		}
		case WAIT_TERMINAL:
		{
			printf("Sonoff terminal timed out\n");
			mState = EXIT_PY;
		}
		break;
		case IDLE:
			mState = CHECK_STATE;
			break;
		default:
			break;
		}
	}

	switch(mState)
	{
		case UNKNOWN:
		case IDLE:
			break;
		case CHECK_STATE:
		{
			//printf("Check Sonoff state\n");
			uint8_t buff[] = {"hello\n"};
			if(transmitCB(buff, 6))
			{
				mState = WAIT_KO;
			}
		}
		break;
		case WAIT_KO:
		{
			if(mSonoffReply == WAIT_KO)
			{
				mSonoffReply = UNKNOWN;
				//printf("KO replied\n");
				mState = IDLE;
			}
		}
		break;
		case WAIT_OK:
		{
			if(mSonoffReply == WAIT_OK)
			{
				mSonoffReply = UNKNOWN;
				//printf("OK replied\n");
				mState = IDLE;
			}
		}
		break;
		case EXIT_PY:
		{
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
			if(mSonoffReply == WAIT_TERMINAL)
			{
				printf("Sonoff terminal available\n");
				uint8_t reset_seq[] = {0x0A, 0x0D, 0x04};
				if(transmitCB(reset_seq, 3))
				{
					mPromptCount = 0;
					printf("Software Reset signal sent\n");
					mState = IDLE;
				}
			}
		}
		break;
		case PUBLISH:
		{
			if(mPublishMessage)
			{
				//printf("Sonoff Publish %s\n", mPublishMessage);

				if(transmitCB((uint8_t*)mPublishMessage, strlen(mPublishMessage)))
				{
					free(mPublishMessage);
					mPublishMessage = 0;
					mState = WAIT_OK;
				}
				else
				{
					printf("Sent publish request timed out\n");
				}
			}
			else
			{
				mState = IDLE;
			}

		}
		break;
	}
}

void SonoffPipe::handleLine(const char *line)
{
	printf("sonoff_RX %s\n", line);
	if(!strncmp(line, "KO", 2))
	{
		mSonoffReply = WAIT_KO;
	}
	if(!strncmp(line, "OK", 2))
	{
		mSonoffReply = WAIT_OK;
	}
}


void SonoffPipe::resetSonoff()
{
	printf("Reset Sonoff\n");

	mState = EXIT_PY;

}

bool SonoffPipe::publish(const char *message)
{
	if(mState != IDLE)
		return false;

	if(mPublishMessage)
		free(mPublishMessage);

	int str_len = strlen(message);
	mPublishMessage = (char*)malloc(str_len + 2);
	strcpy(mPublishMessage, message);
	mPublishMessage[str_len] = '~';
	mPublishMessage[str_len + 1] = 0;

	mKeepAliveTick = HAL_GetTick() + 30000;

	mState = PUBLISH;

	return true;
}

void SonoffPipe::setReceivedCB(void (*receive_cb)(const char* line))
{
	mReceiveCB = receive_cb;
}
