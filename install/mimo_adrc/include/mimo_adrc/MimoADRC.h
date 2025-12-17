#ifndef MIMO_ADRC_H
#define MIMO_ADRC_H

#include <cmath>
#include <immintrin.h> // <--- 核心！这是 AVX 指令集的头文件

// 定义我们一次处理多少个电机
// AVX2 是 256位寄存器，float 是 32位
// 256 / 32 = 8。所以我们一次能算 8 个轴。
#define SIMD_WIDTH 8 

class MimoADRC {
public:
    MimoADRC();
    
    // 初始化：注意 h, r, b0 这里我还是用 float
    // 意思是我们假设 8 个电机拥有相同的控制参数（这在多旋翼或足式机器人里很常见）
    void init(float h, float r, float b0);
    
    // 设置增益 (也是假设大家参数一样)
    void setGains(float beta1, float beta2, float beta3, float k1, float k2, float c);

    // --- 向量化 API ---
    
    // 这里的参数是指针 float*
    // 意思是你给我传一个数组（比如 float y[8]），我一次性读进来
    void observe(const float* y_batch, const float* u_prev_batch);
    
    // 获取 8 个轴的控制量
    void calculateU0(const float* v0_batch, float* u0_out);
    
    // 获取 8 个轴的 z3
    void getZ3(float* z3_out);

private:
    // --- 向量化辅助函数 ---
    // 以后我们会把 fhan 和 fal 改造成处理 __m256 类型的函数
    // __m256 就是那个“装了8个float的盒子”
    __m256 fal_vec(__m256 e, __m256 alpha, __m256 delta);
    __m256 fhan_vec(__m256 x1, __m256 x2, __m256 r, __m256 h);
    __m256 sign_vec(__m256 x);

    // --- 标量参数 (大家共用) ---
    float h_, r_, b0_;
    float beta1_, beta2_, beta3_;
    float k1_, k2_, c_;

    // --- 向量状态 (SoA 结构) ---
    // alignas(32) 是告诉编译器：
    // "请把这些数组在内存里对齐到 32字节边界"
    // 因为 AVX 的叉车如果铲到不整齐的内存，速度会变慢甚至报错。
    
    alignas(32) float v1_[SIMD_WIDTH]; // 存放 8 个轴的 v1
    alignas(32) float v2_[SIMD_WIDTH]; // 存放 8 个轴的 v2
    
    alignas(32) float z1_[SIMD_WIDTH]; // 存放 8 个轴的 z1
    alignas(32) float z2_[SIMD_WIDTH]; // 存放 8 个轴的 z2
    alignas(32) float z3_[SIMD_WIDTH]; // 存放 8 个轴的 z3
};

#endif