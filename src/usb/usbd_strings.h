/*
 * @brief Macro definitions for easy declaration of string descriptors for USBD.
 *
 * Copyright(C) 2015 DF9DQ
 * All rights reserved.
 * Code marked with "Copied from BOOST.ORG" below:
 *     (C) Copyright Edward Diener 2011.                                    *
 *     (C) Copyright Paul Mensonides 2011.                                  *
 *     Distributed under the Boost Software License, Version 1.0. (See      *
 *     accompanying file LICENSE_1_0.txt or copy at                         *
 *     http://www.boost.org/LICENSE_1_0.txt)                                *
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

#ifndef __USBD_STRINGS_H
#define __USBD_STRINGS_H


#define USBSTR_DECLx(i,s)       uint8_t length##i; uint8_t type##i; wchar_t text##i[sizeof(s)/2-1]
#define USBSTR_DECL1(s)         USBSTR_DECLx(1,s);
#define USBSTR_DECL2(s, ...)    USBSTR_DECLx(2,s);  USBSTR_DECL1(__VA_ARGS__)
#define USBSTR_DECL3(s, ...)    USBSTR_DECLx(3,s);  USBSTR_DECL2(__VA_ARGS__)
#define USBSTR_DECL4(s, ...)    USBSTR_DECLx(4,s);  USBSTR_DECL3(__VA_ARGS__)
#define USBSTR_DECL5(s, ...)    USBSTR_DECLx(5,s);  USBSTR_DECL4(__VA_ARGS__)
#define USBSTR_DECL6(s, ...)    USBSTR_DECLx(6,s);  USBSTR_DECL5(__VA_ARGS__)
#define USBSTR_DECL7(s, ...)    USBSTR_DECLx(7,s);  USBSTR_DECL6(__VA_ARGS__)
#define USBSTR_DECL8(s, ...)    USBSTR_DECLx(8,s);  USBSTR_DECL7(__VA_ARGS__)
#define USBSTR_DECL9(s, ...)    USBSTR_DECLx(9,s);  USBSTR_DECL8(__VA_ARGS__)
#define USBSTR_DECL10(s, ...)   USBSTR_DECLx(10,s); USBSTR_DECL9(__VA_ARGS__)
#define USBSTR_DECL11(s, ...)   USBSTR_DECLx(11,s); USBSTR_DECL10(__VA_ARGS__)
#define USBSTR_DECL12(s, ...)   USBSTR_DECLx(12,s); USBSTR_DECL11(__VA_ARGS__)
#define USBSTR_DECL13(s, ...)   USBSTR_DECLx(13,s); USBSTR_DECL12(__VA_ARGS__)
#define USBSTR_DECL14(s, ...)   USBSTR_DECLx(14,s); USBSTR_DECL13(__VA_ARGS__)
#define USBSTR_DECL15(s, ...)   USBSTR_DECLx(15,s); USBSTR_DECL14(__VA_ARGS__)
#define USBSTR_DECL16(s, ...)   USBSTR_DECLx(16,s); USBSTR_DECL15(__VA_ARGS__)
#define USBSTR_DECL17(s, ...)   USBSTR_DECLx(17,s); USBSTR_DECL16(__VA_ARGS__)
#define USBSTR_DECL18(s, ...)   USBSTR_DECLx(18,s); USBSTR_DECL17(__VA_ARGS__)
#define USBSTR_DECL19(s, ...)   USBSTR_DECLx(19,s); USBSTR_DECL18(__VA_ARGS__)
#define USBSTR_DECL20(s, ...)   USBSTR_DECLx(20,s); USBSTR_DECL19(__VA_ARGS__)
#define USBSTR_DECL21(s, ...)   USBSTR_DECLx(21,s); USBSTR_DECL20(__VA_ARGS__)
#define USBSTR_DECL22(s, ...)   USBSTR_DECLx(22,s); USBSTR_DECL21(__VA_ARGS__)
#define USBSTR_DECL23(s, ...)   USBSTR_DECLx(23,s); USBSTR_DECL22(__VA_ARGS__)
#define USBSTR_DECL24(s, ...)   USBSTR_DECLx(24,s); USBSTR_DECL23(__VA_ARGS__)
#define USBSTR_DECL25(s, ...)   USBSTR_DECLx(25,s); USBSTR_DECL24(__VA_ARGS__)
#define USBSTR_DECL26(s, ...)   USBSTR_DECLx(26,s); USBSTR_DECL25(__VA_ARGS__)
#define USBSTR_DECL27(s, ...)   USBSTR_DECLx(27,s); USBSTR_DECL26(__VA_ARGS__)
#define USBSTR_DECL28(s, ...)   USBSTR_DECLx(28,s); USBSTR_DECL27(__VA_ARGS__)
#define USBSTR_DECL29(s, ...)   USBSTR_DECLx(29,s); USBSTR_DECL28(__VA_ARGS__)
#define USBSTR_DECL30(s, ...)   USBSTR_DECLx(30,s); USBSTR_DECL29(__VA_ARGS__)

#define USBSTR_DEFx(i,s)        .length##i = sizeof(s), .type##i = 3, .text##i = s
    /* NOTE: sizeof(s) includes the terminating 0 of the string (2 characters because of wchar16).
     *       This conveniently covers the two bytes of the length and type fields!
     */
#define USBSTR_DEF1(s)          USBSTR_DEFx(1,s)
#define USBSTR_DEF2(s, ...)     USBSTR_DEFx(2,s),  USBSTR_DEF1(__VA_ARGS__)
#define USBSTR_DEF3(s, ...)     USBSTR_DEFx(3,s),  USBSTR_DEF2(__VA_ARGS__)
#define USBSTR_DEF4(s, ...)     USBSTR_DEFx(4,s),  USBSTR_DEF3(__VA_ARGS__)
#define USBSTR_DEF5(s, ...)     USBSTR_DEFx(5,s),  USBSTR_DEF4(__VA_ARGS__)
#define USBSTR_DEF6(s, ...)     USBSTR_DEFx(6,s),  USBSTR_DEF5(__VA_ARGS__)
#define USBSTR_DEF7(s, ...)     USBSTR_DEFx(7,s),  USBSTR_DEF6(__VA_ARGS__)
#define USBSTR_DEF8(s, ...)     USBSTR_DEFx(8,s),  USBSTR_DEF7(__VA_ARGS__)
#define USBSTR_DEF9(s, ...)     USBSTR_DEFx(9,s),  USBSTR_DEF8(__VA_ARGS__)
#define USBSTR_DEF10(s, ...)    USBSTR_DEFx(10,s), USBSTR_DEF9(__VA_ARGS__)
#define USBSTR_DEF11(s, ...)    USBSTR_DEFx(11,s), USBSTR_DEF10(__VA_ARGS__)
#define USBSTR_DEF12(s, ...)    USBSTR_DEFx(12,s), USBSTR_DEF11(__VA_ARGS__)
#define USBSTR_DEF13(s, ...)    USBSTR_DEFx(13,s), USBSTR_DEF12(__VA_ARGS__)
#define USBSTR_DEF14(s, ...)    USBSTR_DEFx(14,s), USBSTR_DEF13(__VA_ARGS__)
#define USBSTR_DEF15(s, ...)    USBSTR_DEFx(15,s), USBSTR_DEF14(__VA_ARGS__)
#define USBSTR_DEF16(s, ...)    USBSTR_DEFx(16,s), USBSTR_DEF15(__VA_ARGS__)
#define USBSTR_DEF17(s, ...)    USBSTR_DEFx(17,s), USBSTR_DEF16(__VA_ARGS__)
#define USBSTR_DEF18(s, ...)    USBSTR_DEFx(18,s), USBSTR_DEF17(__VA_ARGS__)
#define USBSTR_DEF19(s, ...)    USBSTR_DEFx(19,s), USBSTR_DEF18(__VA_ARGS__)
#define USBSTR_DEF20(s, ...)    USBSTR_DEFx(20,s), USBSTR_DEF19(__VA_ARGS__)
#define USBSTR_DEF21(s, ...)    USBSTR_DEFx(21,s), USBSTR_DEF20(__VA_ARGS__)
#define USBSTR_DEF22(s, ...)    USBSTR_DEFx(22,s), USBSTR_DEF21(__VA_ARGS__)
#define USBSTR_DEF23(s, ...)    USBSTR_DEFx(23,s), USBSTR_DEF22(__VA_ARGS__)
#define USBSTR_DEF24(s, ...)    USBSTR_DEFx(24,s), USBSTR_DEF23(__VA_ARGS__)
#define USBSTR_DEF25(s, ...)    USBSTR_DEFx(25,s), USBSTR_DEF24(__VA_ARGS__)
#define USBSTR_DEF26(s, ...)    USBSTR_DEFx(26,s), USBSTR_DEF25(__VA_ARGS__)
#define USBSTR_DEF27(s, ...)    USBSTR_DEFx(27,s), USBSTR_DEF26(__VA_ARGS__)
#define USBSTR_DEF28(s, ...)    USBSTR_DEFx(28,s), USBSTR_DEF27(__VA_ARGS__)
#define USBSTR_DEF29(s, ...)    USBSTR_DEFx(29,s), USBSTR_DEF28(__VA_ARGS__)
#define USBSTR_DEF30(s, ...)    USBSTR_DEFx(30,s), USBSTR_DEF29(__VA_ARGS__)


/**** Copied from BOOST.ORG (modified) ***********************************************************/

#define COUNT_ARGS(...) __COUNT_ARGS_I(__VA_ARGS__ ,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)        
#define __COUNT_ARGS_I(e30,e29,e28,e27,e26,e25,e24,e23,e22,e21,e20,e19,e18,e17,e16,e15,e14,e13,e12,e11,e10,e9,e8,e7,e6,e5,e4,e3,e2,e1, size, ...) size

/*************************************************************************************************/

#define __DECLARE_USBD_STRING_FINAL(name, language, size,...)   \
    struct {                                                    \
        uint8_t length0; uint8_t type0; uint16_t lang;          \
        USBSTR_DECL##size(__VA_ARGS__)                          \
        uint8_t terminator;                                     \
    } name = {                                                  \
        .length0 = 4, .type0 = 3, .lang = language,             \
        USBSTR_DEF##size(__VA_ARGS__),                          \
        .terminator = 0                                         \
    }
#define __DECLARE_USBD_STRING_I(name, language, size, ...)  __DECLARE_USBD_STRING_FINAL(name, language, size, __VA_ARGS__)
#define DECLARE_USBD_STRINGS(name, language, ...)           __DECLARE_USBD_STRING_I(name, language, COUNT_ARGS(__VA_ARGS__), __VA_ARGS__)

#endif /* __USBD_STRINGS_H */
