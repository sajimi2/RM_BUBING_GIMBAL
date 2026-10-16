#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


/* 使用标准定宽类型，避免 GCC/newlib 的 int32_t 定义发生冲突。 */
#include <stdint.h>
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif



