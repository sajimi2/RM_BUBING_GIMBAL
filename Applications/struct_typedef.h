#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


/* 使用编译器标准定宽类型，避免 GCC 的 uint32_t 与手写 typedef 冲突。 */
#include <stdint.h>
#include "cmsis_compiler.h"
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif



