/*
 ******************************************************************************
  * @file           : Buffer.c
  * @brief          : Main program body
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "buffer.h"


/* Private variables ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
BUF_HandleTypeDef Buffer_Init(size_t size)
{
	BUF_HandleTypeDef circular_buffer = malloc(sizeof(circular_buf));
	// If size is 1024, capacity will be 1025, and indices' range is [0..1024]
	circular_buffer->buffer = (uint8_t*)malloc(size + 1);
	circular_buffer->capacity = size + 1;
	Buffer_Clear(circular_buffer);
	return circular_buffer;
}

size_t Buffer_Size(BUF_HandleTypeDef buf)
{
	size_t size = (buf->capacity + buf->write_index - buf->read_index) % buf->capacity;
	return size;
}

int Buffer_IsEmpty(BUF_HandleTypeDef buf)
{
	return Buffer_Size(buf) == 0;
}

void Buffer_Clear(BUF_HandleTypeDef buf)
{
	buf->write_index = 0;
	buf->read_index = 0;
}

int Buffer_IsFull(BUF_HandleTypeDef buf)
{
	return Buffer_Size(buf) == buf->capacity - 1;
}

uint8_t Buffer_Read(BUF_HandleTypeDef buf)
 {
	uint8_t data = 0;
	if(!Buffer_IsEmpty(buf))
	{
		data = buf->buffer[buf->read_index];
		buf->read_index++;
		buf->read_index %= buf->capacity;
	}
	return data;
}

void Buffer_Write(BUF_HandleTypeDef buf, uint8_t data)
{
	if(Buffer_IsFull(buf))
	{
	    Buffer_Read(buf);
	}
	buf->buffer[buf->write_index] = data;
	buf->write_index++;
	buf->write_index %= buf->capacity;
}

