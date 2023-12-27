/**
 * @file MovingAverageFilter.c
 * @author Talha Sarı (talha.sari@outlook.com.tr)
 * @brief An N-Point Moving Average Filter implementation in C
 * @version v1.0
 * @date 2021-04-20
 * 
 * @copyright Copyright (c) 2021 Talha Sarı
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include "MovingAverageFilter.h"


void MovingAverageFilter_Init(MovingAverageFilter *filter, uint8_t nPoint)
{
    /* Set N-Point Size */
    filter->bufferSize = nPoint;

    /* Clearing Filter Buffers */
    if (filter->buffer != 0) {
        filter->buffer = (float*)realloc(filter->buffer, filter->bufferSize * sizeof(float));
    } else {
        filter->buffer = (float*)calloc(filter->bufferSize, sizeof(float));
    }

    if (filter->impulseResponse != 0) {
        filter->impulseResponse = (float*)realloc(filter->impulseResponse, filter->bufferSize * sizeof(float));
    } else {
        filter->impulseResponse = (float*)calloc(filter->bufferSize, sizeof(float));
    }

    /* Init Filter Buffer */
    for (int8_t i = 0; i < filter->bufferSize; i++) {
        *((filter->buffer) + i) = 0.0f;
    }

    /* Init Filter Impulse Response */
    for (int8_t i = 0; i < filter->bufferSize; i++) {
        *((filter->impulseResponse) + i) = (float)(1.0f / filter->bufferSize);
    }

    /* Clear Filter Output */
    filter->output = 0.0f;
}

float MovingAverageFilter_Update(MovingAverageFilter *filter, float input)
{
    filter->output = 0.0f;

    /* Data Shift */
    for (int8_t i = (filter->bufferSize - 1); i >= 0; i--) {
        *(filter->buffer + i + 1) = *(filter->buffer + i);
    }
    *(filter->buffer) = input;

    /* Filter Output Calculation */
    for (int8_t i = 0; i < filter->bufferSize; i++) {
        filter->output = filter->output + (*(filter->buffer + i) * (*(filter->impulseResponse + i)));
    }

    return filter->output;
}