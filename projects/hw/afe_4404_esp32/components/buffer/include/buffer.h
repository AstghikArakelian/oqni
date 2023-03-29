/*
 * Buffer.h
 *
 *  Created on: Feb 11, 2023
 *      Author: astgh
 */

#ifndef BUFFER_H_
#define BUFFER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>

/* Exported types ------------------------------------------------------------*/
struct format_uint32
{
	uint32_t data;
	uint32_t addr;
};

struct format_float
{
	float data;
	uint32_t addr;
};

typedef union
{
	uint8_t command[8];
	struct format_uint32 separated_uint32;
	struct format_float separated_float;
} cmd_separate;

struct circular_buf {
	uint8_t* buffer;
	size_t write_index;
	size_t read_index;
	size_t capacity;
};

typedef struct circular_buf circular_buf;
typedef circular_buf* BUF_HandleTypeDef;

/* Exported functions prototypes ---------------------------------------------*/

BUF_HandleTypeDef Buffer_Init(size_t size);
void Buffer_Clear(BUF_HandleTypeDef buf);
size_t Buffer_Size(BUF_HandleTypeDef buf);
int Buffer_IsEmpty(BUF_HandleTypeDef buf);
int Buffer_IsFull(BUF_HandleTypeDef buf);
void Buffer_Write(BUF_HandleTypeDef buf, uint8_t data);
uint8_t Buffer_Read(BUF_HandleTypeDef buf);


#endif /* BUFFER_H_ */
