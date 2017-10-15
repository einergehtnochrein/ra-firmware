/* Copyright (c) 2017, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "linregression.h"


/* Read a new RSSI value and filter it. Return filtered RSSI in dBm. */
float LINREG_computeRegression(void* context, const LINREG_Input* in, LINREG_Output* out)
{
    int i;
    float f;
    float x, y;
    float maxdev = FLT_MAX;
    float posWeight = 1.0f;
    float negWeight = 1.0f;


    /* Overrides from input */
    if (in->maxdev != 0) {
        maxdev = in->maxdev;
    }
    if (in->posWeight != 0) {
        posWeight = fabs(in->posWeight);
    }
    if (in->negWeight != 0) {
        negWeight = fabs(in->negWeight);
    }

    /* Mean of all samples */
    float meana = 0;
    for (i = 0; i < in->N; i++) {
        in->get_data(context, i, &x, &y);
        meana += y;
    }
    meana /= in->N;

    /* Improve mean estimate by discarding all samples "too far away" */
    int n = 0;
    float mean = 0;
    float meanx = 0;
    for (i = 0; i < in->N; i++) {
        in->get_data(context, i, &x, &y);
        if (fabs(y - meana) < maxdev) {
            mean += y;
            meanx += x;
            ++n;
        }
    }
    if (n < 2) {
        in->get_data(context, in->N - 1, &x, &y);
        return y;
    }
    mean /= n;
    meanx /= n;

    float sum1 = 0;
    float sum2 = 0;
    for (i = 0; i < in->N; i++) {
        in->get_data(context, i, &x, &y);
        if (fabs(y - meana) < maxdev) {
            f = x - meanx;
            if (y - mean < 0) {
                sum1 += negWeight * f * (y - mean);
            }
            else {
                sum1 += posWeight * f * (y - mean);
            }
            sum2 += f * f;
        }
    }

    float slope = sum1 / sum2;
    float intercept = mean - out->slope * meanx;
    if (out != NULL) {
        out->slope = slope;
        out->intercept = intercept;
        out->variance = sum2 / (n - 1);
    }

    return intercept + slope * (in->N - 1);
}


