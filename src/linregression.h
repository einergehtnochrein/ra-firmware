
#ifndef __LINREGRESSION_H
#define __LINREGRESSION_H


typedef struct {
    int N;
    void (*get_data)(void *context, int index, float *x, float *y);
    float maxdev;                           /* Maximum allowed deviation of samples from mean (0 = no check) */
    float posWeight;                        /* Weight of samples above mean (1.0f = normal, 0 = off) */
    float negWeight;                        /* Weight of samples below mean (1.0f = normal, 0 = off) */
} LINREG_Input;

typedef struct {
    float slope;
    float intercept;
    float variance;
} LINREG_Output;

float LINREG_computeRegression (void *context, const LINREG_Input *in, LINREG_Output *out);

#endif
