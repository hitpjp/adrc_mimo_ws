#ifndef ADRC_H
#define ADRC_H

#include <cmath>
#include <algorithm>

class ADRC {
public:
    ADRC();
    void init(float h, float r, float b0);
    void setGains(float beta1, float beta2, float beta3, float k1, float k2, float c);

    // --- 新的拆分 API ---
    
    // 第一步：观测 (输入：当前测量值 y, 上一时刻控制量 u)
    // 功能：更新 ESO，计算出 z1, z2, z3
    void observe(float y, float u_prev);

    // 第二步：获取扰动 (输出：z3)
    // 功能：给外部提供 z3 用于向量运算
    float getZ3() const;
    
    // 获取观测值 z1
    float getZ1() const;

    // 第三步：计算初级控制量 u0 (输入：期望值 v0)
    // 功能：运行 TD 和 NLSEF，算出未补偿的 u0
    float calculateU0(float v0);

private:
    // 参数
    float h_, r_, b0_;
    float beta1_, beta2_, beta3_;
    float k1_, k2_;
    float c_;  // 用于 NLSEF 的 fhan
    
    // 状态
    float v1_, v2_;       // TD states
    float z1_, z2_, z3_;  // ESO states
    
    // 辅助函数
    float fal(float e, float alpha, float delta);
    float fhan(float x1, float x2, float r, float h);
    float sgn(float x);
};

#endif