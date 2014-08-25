#ifndef ENDIAN_H
#define ENDIAN_H
 
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN 0
#endif
 
#ifndef BIG_ENDIAN
#define BIG_ENDIAN 1
#endif
 
#ifndef BYTE_ORDER
#define BYTE_ORDER (*(char *) &(int) {1} == 1 ? LITTLE_ENDIAN : BIG_ENDIAN)
#endif
 
#endif