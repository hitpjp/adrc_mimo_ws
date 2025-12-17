#include "mimo_adrc/MimoADRC.h"
#include <cstring> // for memset
#include <cmath>   // for std::abs, std::pow

// ==========================================
//          构造与初始化
// ==========================================

MimoADRC::MimoADRC() {
    std::memset(v1_, 0, sizeof(v1_));
    std::memset(v2_, 0, sizeof(v2_));
    std::memset(z1_, 0, sizeof(z1_));
    std::memset(z2_, 0, sizeof(z2_));
    std::memset(z3_, 0, sizeof(z3_));
}

void MimoADRC::init(float h, float r, float b0) {
    h_ = h;
    r_ = r;
    b0_ = b0;
    
    std::memset(v1_, 0, sizeof(v1_));
    std::memset(v2_, 0, sizeof(v2_));
    std::memset(z1_, 0, sizeof(z1_));
    std::memset(z2_, 0, sizeof(z2_));
    std::memset(z3_, 0, sizeof(z3_));
}

void MimoADRC::setGains(float beta1, float beta2, float beta3, float k1, float k2, float c) {
    beta1_ = beta1; 
    beta2_ = beta2; 
    beta3_ = beta3;
    k1_ = k1; 
    k2_ = k2;
    c_ = c;
}

// ==========================================
//          数学核心函数 (AVX2)
// ==========================================

__m256 MimoADRC::sign_vec(__m256 x) {
    __m256 zero = _mm256_setzero_ps();
    __m256 one = _mm256_set1_ps(1.0f);
    __m256 minus_one = _mm256_set1_ps(-1.0f);
    __m256 eps = _mm256_set1_ps(1e-6f);

    __m256 mask_pos = _mm256_cmp_ps(x, eps, _CMP_GT_OQ);
    __m256 minus_eps = _mm256_sub_ps(zero, eps);
    __m256 mask_neg = _mm256_cmp_ps(x, minus_eps, _CMP_LT_OQ);

    __m256 result = zero;
    result = _mm256_blendv_ps(result, one, mask_pos);
    result = _mm256_blendv_ps(result, minus_one, mask_neg);

    return result;
}

__m256 MimoADRC::fal_vec(__m256 e, __m256 alpha, __m256 delta) {
    alignas(32) float e_arr[8];
    alignas(32) float alpha_arr[8];
    alignas(32) float delta_arr[8];
    alignas(32) float res_arr[8];

    _mm256_store_ps(e_arr, e);
    _mm256_store_ps(alpha_arr, alpha);
    _mm256_store_ps(delta_arr, delta);

    for (int i = 0; i < 8; i++) {
        float val = e_arr[i];
        float a = alpha_arr[i];
        float d = delta_arr[i];
        
        if (std::abs(val) <= d) {
            res_arr[i] = val / std::pow(d, 1.0f - a);
        } else {
            float s = (val > 0) ? 1.0f : -1.0f;
            res_arr[i] = std::pow(std::abs(val), a) * s;
        }
    }
    return _mm256_load_ps(res_arr);
}

__m256 MimoADRC::fhan_vec(__m256 x1, __m256 x2, __m256 r, __m256 h) {
    __m256 zero = _mm256_setzero_ps();
    __m256 two = _mm256_set1_ps(2.0f);
    __m256 eight = _mm256_set1_ps(8.0f);

    __m256 d = _mm256_mul_ps(r, _mm256_mul_ps(h, h));
    __m256 a0 = _mm256_mul_ps(h, x2);
    __m256 y = _mm256_add_ps(x1, a0);

    __m256i abs_mask_i = _mm256_set1_epi32(0x7FFFFFFF); 
    __m256 abs_mask = _mm256_castsi256_ps(abs_mask_i);
    __m256 abs_y = _mm256_and_ps(y, abs_mask);

    __m256 term = _mm256_add_ps(d, _mm256_mul_ps(eight, abs_y));
    __m256 a1 = _mm256_sqrt_ps(_mm256_mul_ps(d, term));

    __m256 sign_y = sign_vec(y);
    __m256 diff = _mm256_sub_ps(a1, d);
    __m256 half_diff = _mm256_div_ps(diff, two);
    __m256 a2 = _mm256_add_ps(a0, _mm256_mul_ps(sign_y, half_diff));

    __m256 sy_term1 = sign_vec(_mm256_add_ps(y, d));
    __m256 sy_term2 = sign_vec(_mm256_sub_ps(y, d));
    __m256 sy = _mm256_div_ps(_mm256_sub_ps(sy_term1, sy_term2), two);

    __m256 term_a = _mm256_sub_ps(_mm256_add_ps(a0, y), a2);
    __m256 a = _mm256_add_ps(_mm256_mul_ps(term_a, sy), a2);

    __m256 sa_term1 = sign_vec(_mm256_add_ps(a, d));
    __m256 sa_term2 = sign_vec(_mm256_sub_ps(a, d));
    __m256 sa = _mm256_div_ps(_mm256_sub_ps(sa_term1, sa_term2), two);

    __m256 sign_a = sign_vec(a);
    __m256 term_final = _mm256_sub_ps(_mm256_div_ps(a, d), sign_a);
    
    __m256 part1 = _mm256_mul_ps(_mm256_mul_ps(r, term_final), sa);
    __m256 part2 = _mm256_mul_ps(r, sign_a);
    
    __m256 neg_part1 = _mm256_sub_ps(zero, part1);
    __m256 result = _mm256_sub_ps(neg_part1, part2);

    return result;
}

// ==========================================
//          业务逻辑函数
// ==========================================

void MimoADRC::observe(const float* y_batch, const float* u_prev_batch) {
    __m256 h_vec = _mm256_set1_ps(h_);
    __m256 b0_vec = _mm256_set1_ps(b0_);
    __m256 beta1_vec = _mm256_set1_ps(beta1_);
    __m256 beta2_vec = _mm256_set1_ps(beta2_);
    __m256 beta3_vec = _mm256_set1_ps(beta3_);

    __m256 y_vec = _mm256_loadu_ps(y_batch);
    __m256 u_vec = _mm256_loadu_ps(u_prev_batch);

    __m256 z1_v = _mm256_load_ps(z1_);
    __m256 z2_v = _mm256_load_ps(z2_);
    __m256 z3_v = _mm256_load_ps(z3_);

    __m256 e = _mm256_sub_ps(z1_v, y_vec);

    __m256 alpha1 = _mm256_set1_ps(0.25f);
    __m256 fe1 = fal_vec(e, alpha1, h_vec);

    __m256 alpha2 = _mm256_set1_ps(0.5f);
    __m256 fe2 = fal_vec(e, alpha2, h_vec);

    __m256 term1 = _mm256_mul_ps(beta1_vec, e);
    __m256 dz1 = _mm256_mul_ps(h_vec, _mm256_sub_ps(z2_v, term1));
    z1_v = _mm256_add_ps(z1_v, dz1);

    __m256 term2 = _mm256_mul_ps(b0_vec, u_vec);
    __m256 term3 = _mm256_mul_ps(beta2_vec, fe2);
    __m256 sum2 = _mm256_sub_ps(_mm256_add_ps(z3_v, term2), term3);
    z2_v = _mm256_add_ps(z2_v, _mm256_mul_ps(h_vec, sum2));

    __m256 term4 = _mm256_mul_ps(beta3_vec, fe1);
    __m256 neg_term4 = _mm256_sub_ps(_mm256_setzero_ps(), term4); 
    z3_v = _mm256_add_ps(z3_v, _mm256_mul_ps(h_vec, neg_term4));

    _mm256_store_ps(z1_, z1_v);
    _mm256_store_ps(z2_, z2_v);
    _mm256_store_ps(z3_, z3_v);
}

void MimoADRC::calculateU0(const float* v0_batch, float* u0_out) {
    __m256 h_vec = _mm256_set1_ps(h_);
    __m256 r_vec = _mm256_set1_ps(r_);
    __m256 c_vec = _mm256_set1_ps(c_);
    
    __m256 v0_vec = _mm256_loadu_ps(v0_batch);
    
    __m256 v1_v = _mm256_load_ps(v1_);
    __m256 v2_v = _mm256_load_ps(v2_);
    __m256 z1_v = _mm256_load_ps(z1_);
    __m256 z2_v = _mm256_load_ps(z2_);

    __m256 v1_minus_v0 = _mm256_sub_ps(v1_v, v0_vec);
    __m256 fh = fhan_vec(v1_minus_v0, v2_v, r_vec, h_vec);

    v1_v = _mm256_add_ps(v1_v, _mm256_mul_ps(h_vec, v2_v));
    v2_v = _mm256_add_ps(v2_v, _mm256_mul_ps(h_vec, fh));

    __m256 e1 = _mm256_sub_ps(v1_v, z1_v);
    __m256 e2 = _mm256_sub_ps(v2_v, z2_v);
    
    __m256 c_times_e2 = _mm256_mul_ps(c_vec, e2);
    __m256 f_control = fhan_vec(e1, c_times_e2, r_vec, h_vec);
    
    __m256 u0_vec = _mm256_sub_ps(_mm256_setzero_ps(), f_control);

    _mm256_store_ps(v1_, v1_v);
    _mm256_store_ps(v2_, v2_v);
    _mm256_storeu_ps(u0_out, u0_vec);
}

void MimoADRC::getZ3(float* z3_out) {
    __m256 z3_v = _mm256_load_ps(z3_);
    _mm256_storeu_ps(z3_out, z3_v);
}