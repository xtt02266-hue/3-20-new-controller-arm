#include "arm_tool.h"
#include "main.h"
float m[6] = {0.2645, 0.17, 0.1705, 0, 0, 0};


Matrixf<4, 4> T;

Matrixf<3, 6>
    rc((float[18]){0, -8.5e-2, 0, 0, 0, 0,
                   13.225e-2, 0, 0, 0, 0, 0,
                   0, 3.7e-2, 8.525e-2, 0, 0, 0}); // 算力矩要用
// float q[6] = {0.5, -0.5, -0.3, -0.6, 0.5, 0.2};
// float qv[6] = {1, 0.5, -1, 0.3, 0, -1};
// float qa[6] = {0.2, -0.3, 0.1, 0, -1, 0};
// float he[6];
robotics::Link links[6];
Matrixf<3, 1> rpy;
Matrixf<3, 1> pos;
float M_PI_2 = 1.570796; // π/2
void arm_tool_init(void)
{
#ifdef __cplusplus // 只有 C++ 编译器会定义这个宏
    Matrixf<3, 3>
        I[6];
    float m[6] = {0.2645, 0.17, 0.1705, 0, 0, 0};
    float offset[6] = {-0.1428, 1.624, -0.6742, -0.0223, -0.1085 + 3.14, -0.3694}; //{-0.0677, -0.0307, 2.0727, 00.13523, 0.01659, -0.4221};
    Matrixf<3, 6> rc((float[18]){0, -8.5e-2, 0, 0, 0, 0,
                                 13.225e-2, 0, 0, 0, 0, 0,  
                                 0, 3.7e-2, 8.525e-2, 0, 0, 0}); // 算力矩要用
    I[0] = matrixf::diag<3, 3>((float[3]){1.542e-3, 0, 1.542e-3});
    I[1] = matrixf::diag<3, 3>((float[3]){0, 0.409e-3, 0.409e-3});
    I[2] = matrixf::diag<3, 3>((float[3]){0.413e-3, 0.413e-3, 0});
    I[3] = matrixf::eye<3, 3>() * 3.0f;
    I[4] = matrixf::eye<3, 3>() * 2.0f;
    I[5] = matrixf::eye<3, 3>() * 1.0f;

    links[0] = robotics::Link(0, 95, 0, PI / 2, robotics::R, offset[0], 0, 0, m[0], rc.col(0), I[0]); // theta,d,a,alpha
    links[1] = robotics::Link(0, 0, 68, 0, robotics::R, offset[1], 0, 0, m[1], rc.col(1), I[1]);
    links[2] = robotics::Link(0, 0, 82, PI / 2, robotics::R, offset[2], 0, 0, m[2], rc.col(2), I[2]);
    links[3] = robotics::Link(0, 96, 0, PI / 2, robotics::R, offset[3], 0, 0, m[3], rc.col(3), I[3]);
    links[4] = robotics::Link(0, 0, 0, PI / 2, robotics::R, offset[4], 0, 0, m[4], rc.col(4), I[4]);
    links[5] = robotics::Link(0, 107, 0, 0, robotics::R, offset[5], 0, 0, m[5], rc.col(5), I[5]);

    // float q[6] = {0.5, -0.5, -0.3, -0.6, 0.5, 0.2};
    // float qv[6] = {1, 0.5, -1, 0.3, 0, -1};
    // float qa[6] = {0.2, -0.3, 0.1, 0, -1, 0};
    // float he[6] = {1, 2, -3, -0.5, -2, 1};
#endif //__cplusplus
}

void arm_tool_fuc_fkine(float *q, float *arm) // arm(xyz,pitch,yaw,roll) -> q
{
#ifdef __cplusplus // 只有 C++ 编译器会定义这个宏
    robotics::Serial_Link <6> p560(links);
    T = p560.fkine(q); // 正运动学解算
    arm[0] = T[0][3];
    arm[1] = T[1][3];
    arm[2] = T[2][3];
    Matrixf<3, 3> R = T.block<3, 3>(0, 0);
    rpy = robotics::r2rpy(R);
    arm[3] = rpy[1][0];
    arm[4] = rpy[2][0];
    arm[5] = rpy[0][0];
#endif //__cplusplus
}

void arm_tool_fuc_ikine(float *q, float *arm, float *q_ikine)
{
#ifdef __cplusplus // 只有 C++ 编译器会定义这个宏
    robotics::Serial_Link<6> p560(links);
    rpy[1][0] = arm[3];
    rpy[2][0] = arm[4];
    rpy[0][0] = arm[5];
    Matrixf<3, 3> R = robotics::rpy2r(rpy);
    pos[0][0] = arm[0];
    pos[1][0] = arm[1];
    pos[2][0] = arm[2];
    T = robotics::rp2t(R, pos);
    Matrixf<6, 1>q_pos = p560.ikine(T, q); // q是上一次动作位置，T是目标，q_ikine是这一次的
    for (int i = 0; i < 6; i++)
        q_ikine[i] = q_pos[i][0];
#endif //__cplusplus
}
