
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "psb3.h"
#include "psb3private.h"


/* NOTE: For the time being we only have an algorithm to check the validity of a codeword.
 *       According to available documentation, the frames use Reed-Solomon coding, but no
 *       code parameters have been identified yet. Therefore, no error correction is possible yet.
 */


/* Check parity bits */
_Bool _PSB3_checkParity (uint8_t *buffer, int length)
{
    bool result = false;

    return result;
}

