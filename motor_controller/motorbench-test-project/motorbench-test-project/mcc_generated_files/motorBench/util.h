/**
 * util.h
 * 
 * Utility routines and types for computation
 * 
 * Component: miscellaneous
 */

/* *********************************************************************
 *
 * Motor Control Application Framework
 * R7/RC37 (commit 116330, build on 2023 Feb 09)
 *
 * (c) 2017 - 2023 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(R) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
 * *****************************************************************************/

#ifndef __UTIL_H
#define __UTIL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Limit the slew rate of an output signal to within positive and negative limits.
 * This is intended to be called at a constant rate delta_t, in which case
 * the slew rate limits are equal to limit_pos/delta_t and limit_neg/delta_t.
 * 
 * @param input raw incoming signal
 * @param previousOutput previous output signal
 * @param limitPos maximum positive slew rate
 * @param limitNeg maximum negative slew rate
 * @return updated output signal
 */
inline static int16_t UTIL_LimitSlewRate(int16_t input, int16_t previousOutput, 
        int16_t limitPos, int16_t limitNeg)
{
    /* Note: use 32-bit difference. The difference of a pair of int16_t values
     * is technically a 17-bit integer; contrast the following two cases:
     *   (-10000) - (30000) = -40000
     *   ( 28536) - ( 3000) =  25536
     * If cast to an int16_t, both evaluate to 25536, but this is 
     * clearly an overflow error in the first case.
     */
    const int32_t delta_in = (int32_t)input - previousOutput;
    int16_t output = previousOutput;
    if (delta_in > limitPos)
    {
        output += limitPos;
    }
    else if (delta_in < -limitNeg)
    {
        output -= limitNeg;
    }
    else
    {
        output += (int16_t)delta_in;
    }
    return output;
}

/**
 * Limit the slew rate of an output signal to within positive and negative limits.
 * This is intended to be called at a constant rate delta_t, in which case
 * the slew rate limits are equal to limit/delta_t and limit/delta_t.
 * 
 * @param input raw incoming signal
 * @param previousOutput previous output signal
 * @param limit maximum slew rate
 * @return updated output signal
 */
inline static int16_t UTIL_LimitSlewRateSymmetrical(int16_t input, int16_t previousOutput, 
        int16_t limit)
{
    return UTIL_LimitSlewRate(input, previousOutput, limit, limit);
}

/**
 * Shifts right a 32-bit value by 15, returning the lower 16 bits of the result.
 * (We can gain some speed by doing it in a way that the compiler handles better.)
 * 
 * @param x input
 * @return x >> 15
 */
inline static int16_t UTIL_Shr15(int32_t x)
{
    return (int16_t)((x << 1) >> 16);
    /* use << 1 >> 16, rather than >> 15, because it helps the XC16 compiler
     * produce more optimal code; right now there are no specializations
     * for >> 15
     */
}

/**
 * Helper function to multiply two quantities and shift right by 14.
 * @param a first input
 * @param b second input
 * @return (a*b)>>14
 */
inline static int16_t UTIL_MulQ14(int16_t a, int16_t b)
{
    return (int16_t)(__builtin_mulss(a,b) >> 14);
}

/**
 * Helper function to multiply two Q15 quantities and return a Q15 result.
 * Note that this does NOT correctly handle the overflow situation
 * where -1.0*-1.0 -> -1.0, which needs to be special-cased.
 * 
 * @param a first input in Q15 format
 * @param b second input in Q15 format
 * @return a*b in Q15 format
 */
inline static int16_t UTIL_MulQ15(int16_t a, int16_t b)
{
    return UTIL_Shr15(__builtin_mulss(a,b));
}

/**
 * Helper function to multiply two quantities and shift right by 16.
 * (It could be two Q16 numbers with a Q16 result, 
 * could be a Q16 and a Q15 number with a Q15 result.)
 * 
 * @param a first input
 * @param b second input
 * @return (a*b)>>16
 */
inline static int16_t UTIL_MulQ16(int16_t a, int16_t b)
{
    return (int16_t)(__builtin_mulss(a,b) >> 16);
}

/**
 * Helper function to multiply two quantities and shift right by 16.
 * (It could be two Q16 numbers with a Q16 result, 
 * could be a Q16 and a Q15 number with a Q15 result.)
 * 
 * @param a first input (unsigned)
 * @param b second input
 * @return (a*b)>>16
 */
inline static int16_t UTIL_MulUSQ16(uint16_t a, int16_t b)
{
    return (int16_t)(__builtin_mulus(a,b) >> 16);
}

/**
 * Helper function to multiply two unsigned 16-bit quantities and 
 * shift right by 16.
 * (It could be two Q16 numbers with a Q16 result, 
 * could be a Q16 and a Q15 number with a Q15 result.)
 * 
 * @param a first input (unsigned 16-bit)
 * @param b second input (unsigned 16-bit)
 * @return (a*b)>>16 (unsigned 16-bit)
 */
inline static uint16_t UTIL_MulUUQ16(uint16_t a, uint16_t b)
{
    return (uint16_t)(__builtin_muluu(a,b) >> 16);
}

/**
 * Helper function to multiply two signed 16-bit quantities
 * and return a signed 32-bit result.
 * 
 * @param a first input (signed)
 * @param b second input (signed)
 * @return product a*b (signed)
 */
inline static int32_t UTIL_mulss(int16_t a, int16_t b)
{
    return __builtin_mulss(a,b);
}

/**
 * Helper function to multiply an unsigned 16-bit quantity
 * and a signed 16-bit quantity
 * and return a signed 32-bit result.
 * 
 * @param a first input (unsigned)
 * @param b second input (signed)
 * @return product a*b (signed)
 */
inline static int32_t UTIL_mulus(uint16_t a, int16_t b)
{
    return __builtin_mulus(a,b);
}

/**
 * Helper function to multiply an signed 16-bit quantity
 * and an unsigned 16-bit quantity
 * and return a signed 32-bit result.
 * 
 * @param a first input (signed)
 * @param b second input (unsigned)
 * @return product a*b (signed)
 */
inline static int32_t UTIL_mulsu(int16_t a, uint16_t b)
{
    return __builtin_mulsu(a,b);
}

/**
 * Helper function to multiply two unsigned 16-bit quantities
 * and return an unsigned 32-bit result.
 * 
 * @param a first input (unsigned)
 * @param b second input (unsigned)
 * @return product a*b (unsigned)
 */
inline static uint32_t UTIL_muluu(uint16_t a, uint16_t b)
{
    return __builtin_muluu(a,b);
}

/**
 * Function to calculate square of signed number.
 * Please note that use of this function assumes there will be no overflow.
 * The input value -32768 is invalid and will result
 * in an overflow, with the output value -32768. All other inputs are valid.
 *
 * @param x input in Q15 format
 * @return x*x in Q15 format
 */
inline static int16_t UTIL_SignedSqrNoOverflow(int16_t x) 
{
    return UTIL_MulQ15(x, x);
}

/**
 * Limit an unsigned 16-bit value to a specified minimum
 * 
 * @param x input value
 * @param xmin minimum output value
 * @return the input, limited at a minimum to xmin
 */
inline static uint16_t UTIL_LimitMinimumU16(uint16_t x, uint16_t xmin)
{
    return (x < xmin) ? xmin : x;
}

/**
 * Limit a signed 16-bit value to a specified minimum
 * 
 * @param x input value
 * @param xmin minimum output value
 * @return the input, limited at a minimum to xmin
 */
inline static int16_t UTIL_LimitMinimumS16(int16_t x, int16_t xmin)
{
    return (x < xmin) ? xmin : x;
}

/**
 * Limit an unsigned 16-bit value to a specified maximum
 * 
 * @param x input value
 * @param xmax maximum output value
 * @return the input, limited at a maximum to xmax
 */
inline static uint16_t UTIL_LimitMaximumU16(uint16_t x, uint16_t xmax)
{
    return (x > xmax) ? xmax : x;
}

/**
 * Limit a signed 16-bit value to a specified maximum
 * 
 * @param x input value
 * @param xmax maximum output value
 * @return the input, limited at a maximum to xmax
 */
inline static int16_t UTIL_LimitMaximumS16(int16_t x, int16_t xmax)
{
    return (x > xmax) ? xmax : x;
}

/** * Limits the input between a minimum and a maximum */
inline static int16_t UTIL_LimitS16(int16_t x, int16_t min, int16_t max)
{
    return (x > max ) ? max : ((x < min) ? min : x);
}

/**
 * Saturates a signed 32-bit value to a 16-bit positive/negative bound.
 * 
 * @param x signed 32-bit input to saturate
 * @param xlim saturation limit
 * @return input x saturated to xlim
 */
inline static int16_t UTIL_LimitS32ToS16(int32_t x, int16_t xlim)
{
    if (x >= xlim)
    {
        return xlim;
    }
    if (x <= -xlim)
    {
        return -xlim;
    }
    return (int16_t)x;
}

/**
 * Clears bits in uint16_t that are defined by mask
 * 
 * @param oldFlags input bits
 * @param mask bits to clear
 * @return input with mask cleared
 */
inline static uint16_t UTIL_ClearBits(uint16_t oldFlags, uint16_t mask)
{
  return oldFlags & ~mask;
}

/**
 * Sets bits in uint16_t that are defined by mask
 * 
 * @param oldFlags input bits
 * @param mask bits to set
 * @return input with mask set
 */
inline static uint16_t UTIL_SetBits(uint16_t oldFlags, uint16_t mask)
{
  return oldFlags | mask;
}

/**
 * Copies bits in uint16_t that are defined by mask:
 * clears if "on" is false, sets if "on" is true
 * 
 * @param oldFlags input bits
 * @param mask bits to set or clear
 * @param on true if setting, false if clearing
 * @return input with mask set or clear
 */
inline static uint16_t UTIL_CopyBits(uint16_t oldFlags, uint16_t mask, bool on)    
{
    return (on) ? UTIL_SetBits(oldFlags, mask)
                : UTIL_ClearBits(oldFlags, mask);
}

/* Unions for aliasing 32-bit and pairs of 16-bit variables */

/** Unsigned 16/32 bit alias union */
typedef union tagUX1632_t
{
    struct
    {
        uint16_t lo;    /** lower 16 bits */
        uint16_t hi;    /** upper 16 bits */
    } x16;              /** access as 16-bit values */
    uint32_t x32;       /** access as 32-bit values */
} ux1632_t;

/** Signed 16/32 bit alias union */
typedef union tagSX1632_t
{
    struct
    {
        uint16_t lo;    /** lower 16 bits */
        int16_t  hi;    /** upper 16 bits */ 
    } x16;              /** access as 16-bit values */
    int32_t x32;        /** access as 32-bit values */ 
} sx1632_t;

typedef struct
{
    int16_t min;
    int16_t max;
} minmax16_t;

typedef struct
{
    int16_t min;
    int16_t med;
    int16_t max;
} minmedmax16_t;

/* generalized fixed-point converter macro */
#define FIXEDPT(x, q, result_t) \
    ((result_t)(x * (1L << q) + 0.5))

#define Q15(x) FIXEDPT(x, 15, int16_t)
#define Q14(x) FIXEDPT(x, 14, int16_t)
#define Q13(x) FIXEDPT(x, 13, int16_t)
#define Q12(x) FIXEDPT(x, 12, int16_t)
#define Q11(x) FIXEDPT(x, 11, int16_t)
    
#define Q17(x) FIXEDPT(x, 17, int32_t)
#define Q16(x) FIXEDPT(x, 16, int32_t)

/**
 * Construct an unsigned 32-bit integer from two 16-bit integers.
 * @param xlo low word (unsigned 16-bit)
 * @param xhi high word (unsigned 16-bit)
 * @return xhi << 16 | xlo
 */
inline static uint32_t UTIL_PairU16(uint16_t xlo, uint16_t xhi)
{
    ux1632_t result;
    result.x16.lo = xlo;
    result.x16.hi = xhi;
    return result.x32;
}

/**
 * Construct a signed 32-bit integer from two 16-bit integers.
 * @param xlo low word (unsigned 16-bit)
 * @param xhi high word (signed 16-bit)
 * @return xhi << 16 | xlo
 */
inline static int32_t UTIL_PairS16(uint16_t xlo, int16_t xhi)
{
    sx1632_t result;
    result.x16.lo = xlo;
    result.x16.hi = xhi;
    return result.x32;
}

/**
 * Right shift an unsigned 32-bit value by some runtime value N with N <= 16,
 * returning an unsigned 32-bit result. 
 * 
 * @param x input
 * @param N number of right shifts (N <= 16)
 * @return x >> N
 */
inline static uint32_t UTIL_ShrU32N16(uint32_t x, uint16_t N)
{
    const uint16_t Ncomp = 16 - N;
    uint16_t xlo = x;
    uint16_t xhi = x >> 16;
    
    xlo >>= N;
    xlo |= xhi << Ncomp;
    xhi >>= N;
    
    return UTIL_PairU16(xlo, xhi);
}

/**
 * Right shift an unsigned 32-bit value by some runtime value N, returning an
 * unsigned 32-bit result.
 * 
 * @param x input
 * @param N number of right shifts
 * @return x >> N
 */
inline static uint32_t UTIL_ShrU32N(uint32_t x, uint16_t N)
{
    const uint16_t Ncomp = 16 - N;
    uint16_t xlo = x;
    uint16_t xhi = x >> 16;
    
    if ((int16_t)Ncomp < 0)
    {
        xlo = xhi >> (-Ncomp);
        xhi = 0;
    }
    else
    {
        xlo >>= N;
        xlo |= xhi << Ncomp;
        xhi >>= N;
    }
    
    return UTIL_PairU16(xlo, xhi);
}

/**
 * Right shift a signed 32-bit value by some runtime value N with N <= 16,
 * returning a signed 32-bit result.
 * 
 * @param x input
 * @param N number of right shifts (0 <= N <= 16)
 * @return x >> N
 */
inline static int32_t UTIL_ShrS32N16(int32_t x, uint16_t N)
{
    const uint16_t Ncomp = 16 - N;
    uint16_t xlo = x;
    int16_t xhi = x >> 16;
    
    xlo >>= N;
    xlo |= xhi << Ncomp;
    xhi >>= N;
    
    return UTIL_PairS16(xlo, xhi);
}

/**
 * Right shift a signed 32-bit value by some runtime value N, returning a
 * signed 32-bit result.
 * 
 * @param x input
 * @param N number of right shifts (N >= 0)
 * @return x >> N
 */
inline static int32_t UTIL_ShrS32N(int32_t x, uint16_t N)
{
    const uint16_t Ncomp = 16 - N;
    uint16_t xlo = x;
    int16_t xhi = x >> 16;
    
    if ((int16_t)Ncomp < 0)
    {
        xlo = xhi >> (-Ncomp);
        xhi >>= 15;
    }
    else
    {
        xlo >>= N;
        xlo |= xhi << Ncomp;
        xhi >>= N;
    }
    
    return UTIL_PairS16(xlo, xhi);
}

/**
 * Returns a directed version of (a <= b). If dir is negative, we reverse the
 * sign and compute a >= b instead.
 * 
 * @param a first operand
 * @param b second operand
 * @param dir direction
 * @return a <= b if dir is nonnegative else a >= b
 */
inline static bool UTIL_DirectedLessThanEqual(int16_t a, int16_t b, int16_t dir)
{
    return (dir >= 0)
         ? (a <= b)
         : (a >= b);
}

/**
 * Computes the absolute value of an int16_t number.
 * An input of -32768 will produce an output of +32767;
 * clipping is preferable to overflow. (The dsPIC libq implementation
 * of _Q15abs() uses these same instructions.)
 * 
 * @param x input value
 * @return the absolute value of x
 */
inline static int16_t UTIL_Abs16(int16_t x)
{
    asm volatile (
        "   ;UTIL_Abs16\n"
        "   btsc %[x], #15\n"
        "   neg  %[x], %[x]\n"
        "   btsc %[x], #15\n"
        "   com  %[x], %[x]\n"
        : [x]"+r"(x)
    );
    return x;
}

/**
 * Computes the sign of a nonzero value:
 * negative numbers return -1, zero and positive numbers return +1
 */
inline static int16_t UTIL_SignFromHighBit(int16_t x)
{
    return (x >> 15) | 1;
}

/**
 * Computes the saturated signed addition x+y limited to the -32768,
 * +32767 range.
 * 
 * @param x input value
 * @param y input value
 * @return the saturated signed addition x+y
 */
inline static int16_t UTIL_SatAddS16(int16_t x, int16_t y)
{
    /* Saturation can only occur if x and y have the same sign.
     * If x is nonnegative and y is nonnegative and we get overflow,
     *    x+y is positive and should be limited to 32767.
     * If x is negative and y is negative and we get overflow,
     *    x+y is negative and should be limited to -32768.
     * In either case, if overflow occurs, 
     *    we can use either x or y's most significant bit to decide the result
     */
    int16_t saturated_sum;
    asm volatile (
        "   ;UTIL_SatAddS16\n"
        "   asr %[y], #15, %[s]\n"  // s = 0xFFFF if y is negative, 0 otherwise
        "   btg %[s], #15\n"        // s = 0x7FFF if y is negative, 0x8000 otherwise
        "   add %[x], %[y], %[x]\n" // x = x+y
        "   btsc SR, #2\n"          // overflow in OV = SR<2>
        "   com %[s], %[x]\n"       // x = ~s if overflow bit was set, skip otherwise
        : [x]"+r"(x),
          [s]"=&r"(saturated_sum)
        : [y]"r"(y)
    );
    return x;
}

/**
 * Computes the saturated signed difference x-y, which is equal to
 * (x-y) limited to the -32768, +32767 range.
 * 
 * @param x input value
 * @param y input value
 * @return the saturated signed difference x-y
 */
inline static int16_t UTIL_SatSubS16(int16_t x, int16_t y)
{
    /* Saturation can only occur if x and y have opposite signs.
     *   (we can lump zero with positive values for arithmetic efficiency
     *    so we can just look at the high bit)
     * If x is nonnegative and y is negative and we get overflow,
     *    x-y is positive and should be limited to 32767
     * If x is negative and y is nonnegative and we get overflow
     *    x-y is negative and should be limited to -32768
     * In either case, if overflow occurs, 
     *    we can use either x or y's most significant bit to decide the result
     */
    int16_t saturated_difference;
    asm volatile (
        "   ;UTIL_SatSubS16\n"
        "   asr %[y], #15, %[s]\n"  // s = 0xFFFF if y is negative, 0 otherwise
        "   btg %[s], #15\n"        // s = 0x7FFF if y is negative, 0x8000 otherwise
        "   sub %[x], %[y], %[x]\n" // x = x-y
        "   btsc SR, #2\n"          // overflow in OV = SR<2>
        "   mov %[s], %[x]\n"       // x = s if overflow was set
        : [x]"+r"(x),
          [s]"=&r"(saturated_difference)
        : [y]"r"(y)
    );
    return x;
}

/**
 * Computes the approximate absolute value of an int16_t number.
 * Nonnegative inputs produce an exact output;
 * negative inputs produce an output that is off by 1
 * (e.g. abs16approx(-37) = 36, abs16approx(-32768) = 32767)
 * in order to decrease execution time while preventing overflow.
 * 
 * This function should *not* be used by algorithms which are sensitive
 * to off-by-1 errors: integrators being the main example.
 * 
 * @param x input
 * @return the approximate absolute value of x, equal to (x < 0 ? ~x : x)
 */
inline static int16_t UTIL_Abs16Approx(int16_t x)
{
    asm volatile (
        "   ;UTIL_Abs16Approx\n"
        "   btsc %[x], #15\n"
        "   com  %[x], %[x]\n"
        : [x]"+r"(x)
    );
    return x;
}

/**
 * Function to calculate the square of signed Q15 values.
 * We use UTIL_Abs16Approx() to fixup the -32768 value efficiently
 * without branching: this is a 2-instruction hit that yields +32767.
 *
 * @param input in Q15 format
 * @return square of the input, in Q15 format
 */
inline static int16_t UTIL_SignedSqr(int16_t x)
{
    return UTIL_Abs16Approx(UTIL_SignedSqrNoOverflow(x));
}

/**
 * Computes the Q15 quotient of num/den.
 * Does NOT check for overflow or divide-by-zero. 
 *
 * More specifically, it returns the integer calculation (32768 * num)/den,
 * if that is representable as an int16_t. 
 *
 * This is used mainly with num and den that have the same binary point,
 * in which case the result is a Q15 value.
 *
 * UTIL_DivQ15 can also act on inputs with unequal binary points:
 * if num and den are fixed-point values with Qn and Qd binary points,
 * then the result is a fixed-point value with binary point of Q(n-d+15).
 *
 * @param num dividend with Qn binary point
 * @param den divisor with Qd binary point
 * @return quotient = num/den with Q(n-d+15) binary point
 */
inline static int16_t UTIL_DivQ15(int16_t num, int16_t den)
{
    return __builtin_divf(num, den);
}

/**
 * Computes the Q15 quotient of num/den, 
 * saturating the result to +32767 on overflow.
 * (NOTE: This assumes num/den is a positive value if it can overflow.
 * Negative quotients that overflow are NOT handled properly by this function.)
 * 
 * Behavior is identical to UTIL_DivQ15(),
 * except that on overflow (if the results are not representable in a signed
 * 16-bit integer) the result is overwritten with 32767, providing a saturated
 * positive value for positive overflow.
 * 
 * See UTIL_DivQ15 for guidance on using arbitrary binary points;
 * the same guidance applies to this function.
 * 
 * @param num dividend with Qn binary point
 * @param den divisor with Qd binary point
 * @return quotient = num/den with Q(n-d+15) binary point
 */
inline static int16_t UTIL_DivQ15SatPos(int16_t num, int16_t den)
{
    int16_t quotient;
    
    asm (
        "    ;UTIL_DivQ15SatPos\n"
        "    repeat  #__TARGET_DIVIDE_CYCLES\n"
        "    divf    %[num],%[den]\n"
        "    btsc    SR,#2\n"            // OV = bit 2
        "    mov     #0x7fff, %[quotient]"
        : [quotient]"=a"(quotient)
        : [num]"r"(num), [den]"e"(den)   // den restricted to R2-R14 for DIVF
    );
    return quotient;
}

/* ----------------------------------------------------------------------------
 * The following functions are comparable to or slightly faster than
 * computing (abs16(x) < limit)
 * ----------------------------------------------------------------------------
 */

/**
 * Returns whether a and b are both negative or both nonnegative
 * 
 * @param a
 * @param b
 * @return true if a and b are both negative, or both nonnegative.
 */
inline static bool UTIL_BothNegativeOrNonnegative(int16_t a, int16_t b)
{
    return (a^b) >= 0;
}

/**
 * Computes whether abs(x) < limit
 * @param x input
 * @param limit limit (must be nonnegative)
 * @return whether abs(x) < limit
 */
inline static bool UTIL_AbsLessThan(int16_t x, int16_t limit)
{
    return (x > -limit) && (x < limit);
}

/**
 * Computes whether abs(x) <= limit
 * @param x input
 * @param limit limit (must be nonnegative)
 * @return whether abs(x) <= limit
 */
inline static bool UTIL_AbsLessThanEqual(int16_t x, int16_t limit)
{
    return (x >= -limit) && (x <= limit);
}

/**
 * Computes whether abs(x) > limit
 * @param x input
 * @param limit limit (must be nonnegative)
 * @return whether abs(x) > limit
 */
inline static bool UTIL_AbsGreaterThan(int16_t x, int16_t limit)
{
    return (x < -limit) || (x > limit);
}

/**
 * Computes whether abs(x) >= limit
 * @param x input
 * @param limit limit (must be nonnegative)
 * @return whether abs(x) >= limit
 */
inline static bool UTIL_AbsGreaterThanEqual(int16_t x, int16_t limit)
{
    return (x <= -limit) || (x >= limit);
}

/**
 * Repeat NOP (n+1) times
 * @param n argument to the REPEAT instruction
 */
inline static void UTIL_RepeatNop(uint16_t n)
{
    asm volatile (
        " ;UTIL_RepeatNop\n"
        "   repeat %[n]\n"
        "   nop"
        :: [n]"r"(n) : "memory"
    );
}

/**
 * Toggles the sign bit (bit 15)
 * @param x
 * @return x ^ 0x8000
 */
inline static uint16_t UTIL_ToggleBit15(uint16_t x)
{
    asm (
        "    ;UTIL_ToggleBit15\n"
        "    btg %[x], #15\n"
        : [x]"+r"(x)
    );
    return x;    
}

/**
 * Computes saturated shift-right with an U16 result.
 * If (x >> q) lies within the range of uint16_t values, return it,
 * otherwise return 0xFFFF.
 * 
 * @param x input to shift right
 * @param q number of bits to shift right
 * @return saturated right-shifted result
 */
inline static uint16_t UTIL_SatShrU16(uint32_t x, uint16_t q)
{
    uint16_t hi_word = x >> 16;
    if ((q < 16) && (hi_word >= (uint16_t)(1 << q)))
    {
        return 0xFFFF;
    }
    else
    {
        return x >> q;
    }
}

/**
 * Computes saturated shift-right with an S16 result.
 * If (x >> q) lies within the range of int16_t values, return it,
 * otherwise return the appropriately saturated result (0x8000 if x is negative,
 * else 0x7fff).
 */
inline static int16_t UTIL_SatShrS16(int32_t x, uint16_t q)
{
    const int32_t y = x >> q;    
    const int16_t ylo = y;
    if (q < 16)       // the only chance of overflow is for shift counts < 16
    {        
        const int16_t yhi = y >> 16;
        /* unused bits that will be thrown away
         * These must match the sign of y: either all zero or all one
         */
        const int16_t sign_ylo = ylo >> 15; // -1 if ylo is negative, otherwise 0
        if (yhi != sign_ylo)
        {
            // Uh oh, we had an overflow and need to saturate!
            const int16_t xhi = x >> 16;
            const int16_t sign_x = xhi >> 15;
            return sign_x ^ 0x7fff;
        }
    }
    return ylo;
}

/**
 * Compute the average of two uint16_t values
 * @param a first value
 * @param b second value
 * @return (a+b)/2
 */
inline static uint16_t UTIL_AverageU16(uint16_t a, uint16_t b)
{
    uint16_t c;
    
    asm (
        "    ;UTIL_AverageU16\n"
        "    add %[a],%[b],%[c]\n"
        "    rrc %[c],%[c]"
        : [c]"=r"(c)
        : [a]"r"(a), [b]"r"(b)
    );
    return c;
}

/**
 * Compute the average of two int16_t values
 * @param a first value
 * @param b second value
 * @return (a+b)/2
 */
inline static int16_t UTIL_AverageS16(int16_t a, int16_t b)
{
    return (int16_t)((((int32_t)a) + b) >> 1);
}

/**
 * Compute the average of two int16_t values
 * @param a first value
 * @param b second value
 * @return (a+b)/2
 */
inline static int16_t UTIL_AverageS16_asm(int16_t a, int16_t b)
{
    return UTIL_ToggleBit15(
             UTIL_AverageU16(
               UTIL_ToggleBit15(a),
               UTIL_ToggleBit15(b)
             )
           );
}

/**
 * Compute the minimum and maximum of a set of three int16_t values
 * @param a first value
 * @param b second value
 * @param c third value
 * @return struct containing minimum and maximum value --
 *   this is fairly unusual but it permits the compiler to
 *   optimize by placing in an appropriate pair
 *   of adjacent working registers.
 */
inline static minmax16_t UTIL_MinMax3_S16(int16_t a, int16_t b, int16_t c)
{
    /* Sort a,b,c */
    asm (
        "    ;UTIL_MinMax3_S16\n"
        "    cpslt   %[a], %[b]\n"
        "    exch    %[a], %[b]\n"
        "    cpslt   %[a], %[c]\n"
        "    exch    %[a], %[c]\n"
        "    cpslt   %[b], %[c]\n"
        "    exch    %[b], %[c]\n"
        : [a]"+r"(a),
          [b]"+r"(b),
          [c]"+r"(c)
    );
    /* Now a <= b <= c */

    minmax16_t result;
    result.min = a;
    result.max = c;
    return result;
}

/**
 *
 * Computes x*k, limits the result to the [-32768, 32767 range]
 *   This implementation not valid if both x=-32768 and k=-32768
 * 
 * @param x input
 * @param k gain
 * @return x*k, limited to [-32768, 32767]
 */
inline static int16_t UTIL_ScaleAndClip(int16_t x, int16_t k)
{
    int32_t result = (int32_t)x*k;
    const int16_t intmax = INT16_MAX;

    int16_t m; // most significant bits (30:15) of product
    int16_t s; // sign of m: -1 for negative m, 0 for nonnegative m

    // we will compute:
    // sat = intmax - s:  -32768 for negative m, +32767 for positive m
    // if m is either 0 or -1, then m == s

    asm volatile (
        ";UTIL_ScaleAndClip\n"
        "   rlc     %[result], %[m]\n"
        "   rlc     %d[result], %[m]\n"
        "   asr     %[m], #15, %[s]\n"
        "   cpseq   %[m], %[s]\n"
        "   sub     %[intmax], %[s], %[result]\n"  // set result = sat
        : [result]"+r"(result), [m]"=&r"(m), [s]"=&r"(s)
        : [intmax]"r"(intmax)
    );
    return result;
}

/**
 * Computes  shift-left with an S16 result.
 * If the shift count(q) is positive, the input x << q,
 * else if the shift count(q) is negative, the input x >> (-q)
 * This should only be done if the shift count is a compile-time constant
 * otherwise it takes significant run time
 */
inline static int16_t UTIL_BidirectionalShiftLeft(int16_t x, int16_t q)
{
    if (q < 0)
    {
       return x >> (-q);
    }
    else
    {
       return x << q;
    }   
}

/**
 * Computes (state & 1) ? x : -x;
 * @param state input state
 * @param x amplitude
 * @return x if bit 0 of state is set, -x if it is clear
 */
inline static int16_t UTIL_ApplySign(uint16_t state, int16_t x)
{
    asm (
        "; UTIL_ApplySign\n"
        "   btss  %[state], #0\n"   // skip if bit 0 set
        "   neg   %[x], %[x]\n"
        : [x]"+r"(x)
        : [state]"r"(state)
    );
    return x;   
}

/**
 * Copy sign from a source value: (state & 0x8000) ? -x : x;
 * @param sign_source source value
 * @param x amplitude
 * @return x if bit 15 of sign_source is clear, -x if it is set
 */
inline static int16_t UTIL_CopySign(int16_t sign_source, int16_t x)
{
    asm (
        "; UTIL_CopySign\n"
        "   btsc  %[src], #15\n"   // skip if bit 15 is clear
        "   neg   %[x], %[x]\n"
        : [x]"+r"(x)
        : [src]"r"(sign_source)
    );
    return x;   
}

/**
 * Sort the minimum, median, and maximum of a set of three int16_t values
 * @param a first value
 * @param b second value
 * @param c third value
 * @return struct containing minimum, median, and maximum value
 */
inline static minmedmax16_t UTIL_Sort3_S16(int16_t a, int16_t b, int16_t c)
{
    /* Sort a,b,c */
    asm (
        "    ;UTIL_Sort3_S16\n"
        "    cpslt   %[a], %[b]\n"
        "    exch    %[a], %[b]\n"
        "    cpslt   %[a], %[c]\n"
        "    exch    %[a], %[c]\n"
        "    cpslt   %[b], %[c]\n"
        "    exch    %[b], %[c]\n"
        : [a]"+r"(a),
          [b]"+r"(b),
          [c]"+r"(c)
    );
    /* Now a <= b <= c */

    minmedmax16_t result;
    result.min = a;
    result.med = b;
    result.max = c;
    return result;
}

#ifdef __cplusplus
}
#endif

#endif /* __UTIL_H */
