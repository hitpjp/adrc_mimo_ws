#include "mimo_adrc/ADRC.h"
#include <cmath>
#include <iostream>

ADRC::ADRC() {
    // 初始化所有成员变量为0
    h_ = 0.0f;
    r_ = 0.0f;
    b0_ = 0.0f;
    beta1_ = 0.0f;
    beta2_ = 0.0f;
    beta3_ = 0.0f;
    k1_ = 0.0f;
    k2_ = 0.0f;
    c_ = 0.0f;
    v1_ = 0.0f;
    v2_ = 0.0f;
    z1_ = 0.0f;
    z2_ = 0.0f;
    z3_ = 0.0f;
}

void ADRC::init(float h, float r, float b0) {
    h_ = h;
    r_ = r;
    b0_ = b0;
    
    // 初始化状态变量
    v1_ = 0.0f;
    v2_ = 0.0f;
    z1_ = 0.0f;
    z2_ = 0.0f;
    z3_ = 0.0f;
}

void ADRC::setGains(float beta1, float beta2, float beta3, float k1, float k2, float c) {
    beta1_ = beta1; beta2_ = beta2; beta3_ = beta3;
    k1_ = k1; k2_ = k2;
    c_ = c; // 用于 NLSEF 的 fhan
}

float ADRC::sgn(float x) {
    if (x > 1e-6f) return 1.0f;
    if (x < -1e-6f) return -1.0f;
    return 0.0f;
}   

float ADRC::fal(float e, float alpha, float delta) {
    if (std::abs(e) <= delta) {
        return e / std::pow(delta, 1.0f - alpha);
    } else {
        return std::pow(std::abs(e), alpha) * sgn(e);
    }
}

// === 这一段完全翻译自你的 MATLAB 代码 ===
float ADRC::fhan(float x1, float x2, float r, float h) {
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = std::sqrt(d * (d + 8.0f * std::abs(y)));
    float a2 = a0 + sgn(y) * (a1 - d) / 2.0f;
    
    float sy = (sgn(y + d) - sgn(y - d)) / 2.0f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sgn(a + d) - sgn(a - d)) / 2.0f;
    
    return -r * (a / d - sgn(a)) * sa - r * sgn(a);
}

// ... observe 函数保持不变 (ESO 通常还是用 fal 或线性) ...

// === 重点修改：calculateU0 ===
float ADRC::calculateU0(float v0) {
    // 1. TD (跟踪微分器) - 对应 image_09cddc.png
    // MATLAB: fhan(v1-v0, v2, r, h)
    float fh = fhan(v1_ - v0, v2_, r_, h_);
    
    v1_ += h_ * v2_;
    v2_ += h_ * fh;

    // 2. NLSEF (误差反馈) - 对应 image_09cdf9.png
    // MATLAB: u0 = -fhan(e1, c*e2, r, h1)
    
    float e1 = v1_ - z1_; // 误差
    float e2 = v2_ - z2_; // 误差导数
    
    // 注意：这里用 r_ 作为 fhan 的参数，表示控制器的最大输出能力
    // h_ 作为步长参数
    // c_ 来自 setGains，用于调整阻尼
    float u0 = -fhan(e1, c_ * e2, r_, h_); 
    
    // 如果想要更激进的控制，NLSEF 里的 r 可以单独设一个比 TD 里的 r 更大的参数
    
    return u0;
}

float ADRC::getZ1() const {
    return z1_;
}

float ADRC::getZ3() const {
    return z3_;
}

void ADRC::observe(float y, float u_prev) {
    // 1. 计算误差 e (图右侧: z1 - y)
    // 注意：Simulink 图里经常把 y 接在负端，z1 接正端，所以 e = z1 - y
    float e = z1_ - y;

    // 2. 准备 fal 函数输出
    // 图中上方分支: fal(e, 0.25, h)
    float fe_025 = fal(e, 0.25f, h_);
    
    // 图中中间分支: fal(e, 0.5, h)
    float fe_050 = fal(e, 0.50f, h_);

    // 3. 状态更新 (积分器 1/s 离散化)
    
    // Z1 更新 (图最右侧积分器)
    // 输入: z2 (正) 和 beta_01 * e (负，因为是反馈)
    // 公式: z1_new = z1 + h * (z2 - beta1 * e)
    // 这里如果按照线性ESO习惯是 beta1*e，但图中是把 e 乘增益后减掉
    // 假设 beta1, beta2, beta3 都是正数参数
    z1_ += h_ * (z2_ - beta1_ * e);

    // Z2 更新 (图中间积分器)
    // 输入: z3 (正), b0*u (正), beta_02 * fal(e,0.5,h) (负)
    z2_ += h_ * (z3_ + b0_ * u_prev - beta2_ * fe_050);

    // Z3 更新 (图左侧积分器)
    // 输入: -1 * beta_03 * fal(e,0.25,h)
    z3_ += h_ * (-beta3_ * fe_025);
}