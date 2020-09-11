/* Copyright (c) 2015, DF9DQ
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

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#include "observer.h"
#include "rinex.h"



/* Compute the range rate of an SV relative to a user at rest.
 */
bool GPS_computeUserToSVRangeRate (ECEF_Coordinate *observer, ECEF_Coordinate *sv, float *rangeRate)
{
    /* Vector from user to sv */
    float dx = observer->x - sv->x;
    float dy = observer->y - sv->y;
    float dz = observer->z - sv->z;

    /* Scalar product of this vector and the SV velocity vector, divided by the range,
     * gives us the velocity along the user-SV vector.
     */
    *rangeRate = -(dx * sv->vx + dy * sv->vy + dz * sv->vz) / sqrtf(dx * dx + dy * dy + dz * dz);

    return true;
}



/* Determine suitability of satellite selection for a position solution. */
float GPS_checkSatelliteConstellation (GPS_4SatInfo *sats)
{
    float A[3][3];
    float det;

    A[0][0] = sats->sat1.pos.x - sats->sat2.pos.x;
    A[1][0] = sats->sat1.pos.x - sats->sat3.pos.x;
    A[2][0] = sats->sat1.pos.x - sats->sat4.pos.x;
    A[0][1] = sats->sat1.pos.y - sats->sat2.pos.y;
    A[1][1] = sats->sat1.pos.y - sats->sat3.pos.y;
    A[2][1] = sats->sat1.pos.y - sats->sat4.pos.y;
    A[0][2] = sats->sat1.pos.z - sats->sat2.pos.z;
    A[1][2] = sats->sat1.pos.z - sats->sat3.pos.z;
    A[2][2] = sats->sat1.pos.z - sats->sat4.pos.z;

    det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2])
        - A[0][1] * (A[1][0] * A[2][2] - A[2][0] * A[1][2])
        + A[0][2] * (A[1][0] * A[2][1] - A[2][0] * A[1][1]);

    return det;
}




/* Non-recursive matrix inversion.
 * http://www.c4learn.com/c-programs/c-program-to-find-inverse-of-3-x-3.html
 * https://www.algebra.com/algebra/homework/Matrices-and-determiminant/Matrices-and-determiminant.faq.question.255959.html
 */
static void _GPS_invert3x3Matrix_f (double m[3][3])
{
    int i, j;
    int n;
    double factor;
    static double _mat3x3[3][3];             /* Declared static so it won't consume stack! */


    memset(_mat3x3, 0, sizeof(_mat3x3));
    for (n = 0; n < 3; n++) {
        _mat3x3[n][n] = 1;
    }

    for (n = 0; n < 3; n++) {
        factor = m[n][n];   // Select pivot element from main diagonal
        if (factor != 0) {
            for (i = 0; i < 3; i++) {
                m[n][i] /= factor;
                _mat3x3[n][i] /= factor;
            }

            for (i = 0; i < 3; i++) {
                if (i != n) {
                    factor = m[i][n];
                    for (j = 0; j < 3; j++) {
                        m[i][j] -= m[n][j] * factor;
                        _mat3x3[i][j] -= _mat3x3[n][j] * factor;
                    }
                }
            }
        }
    }

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            m[i][j] = _mat3x3[i][j];
        }
    }
}



/* Non-recursive matrix inversion.
 * http://www.c4learn.com/c-programs/c-program-to-find-inverse-of-3-x-3.html
 * https://www.algebra.com/algebra/homework/Matrices-and-determiminant/Matrices-and-determiminant.faq.question.255959.html
 */
static void _GPS_invert4x4Matrix (double m[4][4])
{
    int i, j;
    int n;
    double factor;
    static double _mat4x4[4][4];            /* Declared static so it won't consume stack! */


    memset(_mat4x4, 0, sizeof(_mat4x4));
    for (n = 0; n < 4; n++) {
        _mat4x4[n][n] = 1;
    }

    for (n = 0; n < 4; n++) {
        factor = m[n][n];   // Select pivot element from main diagonal
        if (factor != 0) {
            for (i = 0; i < 4; i++) {
                m[n][i] /= factor;
                _mat4x4[n][i] /= factor;
            }

            for (i = 0; i < 4; i++) {
                if (i != n) {
                    factor = m[i][n];
                    for (j = 0; j < 4; j++) {
                        m[i][j] -= m[n][j] * factor;
                        _mat4x4[i][j] -= _mat4x4[n][j] * factor;
                    }
                }
            }
        }
    }

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            m[i][j] = _mat4x4[i][j];
        }
    }
}



/* Compute the three roots of the cubic equation
 * x^3 + ax^2 + bx + c = 0
 * Roots are stored in P.
 * Function returns the number of real roots (1 or 3).
 */
static int _GPS_solveCubic (double a, double b, double c, double complex P[3])
{
    double p = b - a*a/3.0;
    double q = 2.0*a*a*a/27.0 - a*b/3.0 + c;

    double D = q*q/4.0 + p*p*p/27.0;
    if (D > 0) {
        /* One real and two conjugate complex roots */
        double argu = -0.5*q + sqrt(D);
        double argv = -0.5*q - sqrt(D);
        double complex uc = cpow(argu, 1.0/3.0);
        double complex vc = cpow(argv, 1.0/3.0);
        double u = copysign(cabs(uc), argu);
        double v = copysign(cabs(vc), argv);

        double complex e1 = -0.5 + 0.5*sqrt(3.0)*I;
        double complex e2 = conjf(e1);

        P[0] = (u + v);
        P[1] = (u*e1 + v*e2);
        P[2] = (u*e2 + v*e1);
    }
    else {
        /* Three real roots */
        double phi = acos(-0.5*q / sqrt(-p*p*p/27.0));
        P[0] = sqrt(-p/3.0) * 2.0 * cos(phi/3.0);
        P[1] = sqrt(-p/3.0) * 2.0 * cos((phi + 2.0*M_PI)/3.0);
        P[2] = sqrt(-p/3.0) * 2.0 * cos((phi + 4.0*M_PI)/3.0);
    }

    P[0] -= a/3.0;
    P[1] -= a/3.0;
    P[2] -= a/3.0;

    return (D < 0) ? 3 : 1;
}



/* Compute the four roots of the quartic equation
 * x^4 + ax^3 + bx^2 + cx + d = 0
 * Roots are stored in P.
 */
static void _GPS_solveQuartic (double a, double b, double c, double d, double complex P[4])
{
    double f = -0.375*a*a + b;
    double g = 0.125*a*a*a - 0.5*a*b + c;
    double h = -3.0*a*a*a*a/256.0 + 0.0625*a*a*b - 0.25*a*c + d;

    double complex Pcubic[3];
    memset(Pcubic, 0, sizeof(Pcubic));
    int nReal = _GPS_solveCubic(f/2.0, (f*f-4.0*h)/16.0, -g*g/64.0, Pcubic);

    /* Select two non-zero roots. Prefer the conjugate complex roots if they exist (numerical stability). */
    double complex p, q;
    if (nReal == 1) {
        p = csqrt(Pcubic[1]);
        q = csqrt(Pcubic[2]);
    }
    else {
        p = csqrt(Pcubic[0]);
        if (p == 0) {
            p = csqrt(Pcubic[1]);
            q = csqrt(Pcubic[2]);
        }
        else {
            q = csqrt(Pcubic[1]);
            if (q == 0) {
                q = csqrt(Pcubic[2]);
            }
        }
    }
    double complex r = -g / (8.0*p*q);
    double s = a / 4.0;

    P[0] =  p + q + r - s;
    P[1] =  p - q - r - s;
    P[2] = -p + q - r - s;
    P[3] = -p - q + r - s;
}



/* Find position solution using all suitable satellites.
 * Uses least square algorithm.
 * Requires a useful initial guess.
 */
    int si[12];
void GPS_findPositionSolutionAllSats (
    GPS_SatInfo *sats,
    int numSats,
    double receiverEpoch,
    ECEF_Coordinate *startPos,
    double *rxClockOffset,
    float *hdop,
    float pressureAltitude,
    uint8_t *pUsedSats)
{
    static struct {
        double correctedPR;
        double Rho;
        double deltaRho;
    } extra[12];                            /* Declared static so it won't consume stack! */
    int i, j, k;
    int N;


    /* Make a selection from the available satellites */
    N = 0;
    for (i = 0; i < numSats; i++) {
        si[i] = 0;
        if ((sats[i].PRN != 0) && (sats[i].prange > 0) && (sats[i].prange < 50e6)
                               && (sats[i].elevation > 10.0*(M_PI/180.0f))
                               && !sats[i].ignore
           )
        {
            si[N] = i;
            ++N;
        }
    }
    if (pUsedSats != NULL) {
        *pUsedSats = N;
    }

    for (i = 0; i < N; i++) {
        double toc = EPHEMERIS_getTOC(sats[si[i]].PRN);
        double clockError = // ohne relativistische Effekte, ohne group delay Korrektur
            ephemeris->sats[sats[si[i]].PRN].SV_clock_bias
          + ephemeris->sats[sats[si[i]].PRN].SV_clock_drift * (receiverEpoch - toc)
          + ephemeris->sats[sats[si[i]].PRN].SV_clock_drift_rate * (receiverEpoch - toc) * (receiverEpoch - toc);
        double transmitTime = receiverEpoch - sats[si[i]].prange / GPS_SPEED_OF_LIGHT - clockError;

        float estimatedRange = 0.07f * 3e8f;
#if 0
        EPHEMERIS_calculateSatellitePosition(
                &ephemeris->sats[sats[si[i]].PRN],
                receiverEpoch ,//+ clockError,       //TODO ?????????????
                &sats[si[i]].pos,
                estimatedRange);
#endif
#if 1
        estimatedRange = sqrtf(
                            (startPos->x - sats[si[i]].pos.x) * (startPos->x - sats[si[i]].pos.x)
                            + (startPos->y - sats[si[i]].pos.y) * (startPos->y - sats[si[i]].pos.y)
                            + (startPos->z - sats[si[i]].pos.z) * (startPos->z - sats[si[i]].pos.z)
                            );
        EPHEMERIS_calculateSatellitePosition(
                            &ephemeris->sats[sats[si[i]].PRN],
                            receiverEpoch ,//+ clockError,
                            &sats[si[i]].pos,
                            estimatedRange
                            );
#endif

        GPS_computeSatelliteClockCorrectionAndDrift(
            &sats[si[i]],
            transmitTime,
            &ephemeris->sats[sats[si[i]].PRN],
            EPHEMERIS_getTOC(sats[si[i]].PRN));

        extra[i].correctedPR = sats[si[i]].prange + sats[si[i]].clockcorr; // includes T_GD!
extra[i].correctedPR -= sats[si[i]].ionoDelay;
    }

    if (N == 3) {
        /* Special case: N = 3. Use altitude from pressure sensor to find position solution */

        double A[3][3];
        double K[3];
        double Huser = 6.371e6 + pressureAltitude;
        double f[3];
        double q, r, s;
        double a[4];

        double x1, y1, z1;
        double x12, y12, z12;
        double x13, y13, z13;
        double p21, p31;

#if SEMIHOSTING_RS92
        printf("\r\npalt = %f\r\n", pressureAltitude);
        printf("sats3 = [\r\n");
        for (i = 0; i < 3; i++) {
            printf("[%lf,%lf,%lf,%lf],\r\n",
                    sats[si[i]].pos.x,
                    sats[si[i]].pos.y,
                    sats[si[i]].pos.z,
                    extra[i].correctedPR
            );
        }
        printf("]\r\n");
#endif

        startPos->x = 0;
        startPos->y = 0;
        startPos->z = 0;

        x1 = sats[si[0]].pos.x;
        y1 = sats[si[0]].pos.y;
        z1 = sats[si[0]].pos.z;
        x12 = sats[si[0]].pos.x - sats[si[1]].pos.x;
        y12 = sats[si[0]].pos.y - sats[si[1]].pos.y;
        z12 = sats[si[0]].pos.z - sats[si[1]].pos.z;
        x13 = sats[si[0]].pos.x - sats[si[2]].pos.x;
        y13 = sats[si[0]].pos.y - sats[si[2]].pos.y;
        z13 = sats[si[0]].pos.z - sats[si[2]].pos.z;
        p21 = extra[1].correctedPR - extra[0].correctedPR;
        p31 = extra[2].correctedPR - extra[0].correctedPR;

        A[0][0] = 2e-7 * x12;
        A[0][1] = 2e-7 * y12;
        A[0][2] = 2e-7 * z12;
        A[1][0] = 2e-7 * x13;
        A[1][1] = 2e-7 * y13;
        A[1][2] = 2e-7 * z13;
        A[2][0] = 2e-7 * sats[si[0]].pos.x;
        A[2][1] = 2e-7 * sats[si[0]].pos.y;
        A[2][2] = 2e-7 * sats[si[0]].pos.z;
        _GPS_invert3x3Matrix_f(A);

        int iteration;
        for (iteration = 0; iteration < 2; iteration++) {
            K[0] = (x12*x12 + y12*y12 + z12*z12 - p21*p21) * 1e-14;
            K[1] = (x13*x13 + y13*y13 + z13*z13 - p31*p31) * 1e-14;
            K[2] = (x1*x1 + y1*y1 + z1*z1 - Huser*Huser) * 1e-14;

            for (i = 0; i < 3; i++) {
                f[i] = 0;
                for (j = 0; j < 3; j++) {
                    f[i] += A[i][j] * K[j];
                }
            }

            q = 2e-7 * (A[0][0] * p21 + A[0][1] * p31);
            r = 2e-7 * (A[1][0] * p21 + A[1][1] * p31);
            s = 2e-7 * (A[2][0] * p21 + A[2][1] * p31);

            a[3] = -2.0 * (A[0][2]*q + A[1][2]*r + A[2][2]*s);
            a[2] = q*q + r*r + s*s + 2.0 * (A[0][2]*f[0] + A[1][2]*f[1] + A[2][2]*f[2]) - 1.0;
            a[1] = -2.0 * (q*f[0] + r*f[1] + s*f[2]);
            a[0] = f[0]*f[0] + f[1]*f[1] + f[2]*f[2];
            for (i = 0; i < 4; i++) {
                a[i] /= A[0][2]*A[0][2] + A[1][2]*A[1][2] + A[2][2]*A[2][2];
            }

            double complex P[4];
            _GPS_solveQuartic(a[3], a[2], a[1], a[0], P);
#if SEMIHOSTING_RS92
            printf("Quartic roots:\r\n");
            for (i = 0; i < 4; i++) {
                printf("  %f %+fi\r\n", creal(P[i]), cimag(P[i]));
            }
#endif

            for (i = 0; i < 4; i++) {
                if (cimag(P[i]) == 0) {
                    double d = creal(P[i]);
                    ECEF_Coordinate ecef;
                    ecef.x = 1e7 * (-A[0][2]*d*d + q*d - f[0]) + x1;
                    ecef.y = 1e7 * (-A[1][2]*d*d + r*d - f[1]) + y1;
                    ecef.z = 1e7 * (-A[2][2]*d*d + s*d - f[2]) + z1;
    //                if ((ecef.x > 0) && (ecef.y > 0) && (ecef.z > 0)) {
                    if ((ecef.z > 4.5e6) && (ecef.z < 5.77e6)) {    //TODO TODO (this check: 45° < lat < 65°)
                        LLA_Coordinate lla;
                        GPS_convertECEF2LLA(&ecef, &lla);
                        *hdop = 19.99f;
                        startPos->x = ecef.x;
                        startPos->y = ecef.y;
                        startPos->z = ecef.z;
#if SEMIHOSTING_RS92
                        printf("ECEF pos = %f | %f | %f (@ d = %f)\r\n", startPos->x, startPos->y, startPos->z, d);
#endif

                        Huser = GPS_getEllipsoidRadius(&lla) + pressureAltitude;
                    }
                }
            }
        }
    }
    else {
        double H[12][4];
        double HTH[4][4];
        double G[4][12];
        int iter;

        for (iter = 0; iter < 2; iter++) {
            /* Fill H, calculate Rho and deltaRho */
            memset(H, 0, sizeof(H));
            for (i = 0; i < N; i++) {
                extra[i].Rho = sqrt(
                    (startPos->x - sats[si[i]].pos.x) * (startPos->x - sats[si[i]].pos.x)
                + (startPos->y - sats[si[i]].pos.y) * (startPos->y - sats[si[i]].pos.y)
                + (startPos->z - sats[si[i]].pos.z) * (startPos->z - sats[si[i]].pos.z));
                extra[i].deltaRho = extra[i].correctedPR - extra[i].Rho;
                H[i][0] = (sats[si[i]].pos.x - startPos->x) / extra[i].Rho;
                H[i][1] = (sats[si[i]].pos.y - startPos->y) / extra[i].Rho;
                H[i][2] = (sats[si[i]].pos.z - startPos->z) / extra[i].Rho;
                H[i][3] = -1;
            }

            /* HT * H */
            memset(HTH, 0, sizeof(HTH));
            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    for (k = 0; k < N; k++) {
                        HTH[i][j] += H[k][i] * H[k][j];
                    }
                }
            }
            /* (HT * H)^-1 */
            _GPS_invert4x4Matrix(HTH);
            /* G = (HT * H)^-1 * HT */
            memset(G, 0, sizeof(G));
            for (i = 0; i < 4; i++) {
                for (j = 0; j < N; j++) {
                    for (k = 0; k < 4; k++) {
                        G[i][j] += HTH[k][i] * H[j][k];
                    }
                }
            }

            /* Calculate new estimate */
            double r[4];
            for (i = 0; i < 4; i++) {
                r[i] = 0;
                for (j = 0; j < N; j++) {
                    r[i] += G[i][j] * extra[j].deltaRho;
                }
            }

            startPos->x -= r[0];
            startPos->y -= r[1];
            startPos->z -= r[2];
            *rxClockOffset += r[3];
        }

        /* Calculate position precision */
        LLA_Coordinate posLLA;
        GPS_convertECEF2LLA(startPos, &posLLA);

        /* G */
        float G2[3][3];
        G2[0][0] = -sinf(posLLA.lat) * cos(posLLA.lon);
        G2[0][1] = -sinf(posLLA.lat) * sin(posLLA.lon);
        G2[0][2] = cosf(posLLA.lat);
        G2[1][0] = -sinf(posLLA.lon);
        G2[1][1] = cosf(posLLA.lon);
        G2[1][2] = 0;
        G2[2][0] = cosf(posLLA.lat) * cos(posLLA.lon);
        G2[2][1] = cosf(posLLA.lat) * sin(posLLA.lon);
        G2[2][2] = sinf(posLLA.lat);

        /* G2 * Cx (NOTE Cx is a sub-matrix of HTH) */
        float Cl1[3][3];
        memset(Cl1, 0, sizeof(Cl1));
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                for (k = 0; k < 3; k++) {
                    Cl1[i][j] += G2[i][k] * HTH[k][j];
                }
            }
        }
        /* Cl = G2 * Cx * G2T */
        float Cl[3][3];
        memset(Cl, 0, sizeof(Cl));
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                for (k = 0; k < 3; k++) {
                    Cl[i][j] += Cl1[i][k] * G2[j][k];
                }
            }
        }

        if (hdop) {
            *hdop = sqrtf(Cl[0][0] * Cl[0][0] + Cl[1][1] * Cl[1][1]);
        }
    }
}


