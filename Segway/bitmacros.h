#ifndef BITMACROS_H
#define BITMACROS_H

//
/* set bit */
#define SET_BIT(var, bit) ((var) |= (1u << (bit)))

/* set bits */
#define SET_BITS(var, bits) ((var) |= (bits))

/* clear bit */
#define CLEAR_BIT(var, bit) ((var) &= (unsigned)~(1u << (bit)))

/* clear bits */
#define CLEAR_BITS(var, bits) ((var) &= (unsigned)~(bits))

/* bit is set/clear */
#define BIT_IS_SET(var, bit) ((var) & (1u << (bit)))
#define BIT_IS_CLEAR(var, bit) !BIT_IS_SET(var, bit)


#endif /* BITMACROS_H */