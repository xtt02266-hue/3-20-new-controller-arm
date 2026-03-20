#ifndef ARM_TOOL_H
#define ARM_TOOL_H

#include "matrix.h"
#include "robotics.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void arm_tool_init(void);
    void arm_tool_fuc_fkine(float* q, float *arm);
    void arm_tool_fuc_ikine(float *q, float *arm, float *q_ikine);
#ifdef __cplusplus
}
#endif

#endif  // ARM_TOOL_H
