#ifndef _WIRECONVERSIONS_H_
#define _WIRECONVERSIONS_H_
#include <stdint.h>

void uint32_to_wire(uint32_t v, uint8_t w[]);
void float_to_wire(float f, uint8_t w[]);
float float_from_wire(const uint8_t w[]);



#endif /* _WIRECONVERSIONS_H_ */
