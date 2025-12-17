// #include "rclcpp/rclcpp.hpp"
// #include "mimo_adrc/MimoADRC.h" // <--- 换成向量版头文件
// #include "std_msgs/msg/float32.hpp"
// #include <cmath>
// #include <vector>

// class MimoNode : public rclcpp::Node {
// public:
//     MimoNode() : Node("mimo_vec_node"), count_(0) {
//         // 1. 实例化向量版 ADRC
//         adrc_vec_ = std::make_shared<MimoADRC>();
        
//         // 2. 初始化参数 (所有轴共用一套参数，常见于多旋翼/机械臂)
//         float h = 0.01f; 
//         adrc_vec_->init(h, 50.0f, 1.0f); 
        
//         adrc_vec_->setGains(
//             100.0f,   // beta1
//             300.0f,   // beta2
//             1000.0f,  // beta3
//             20.0f,    // k1 
//             10.0f,    // k2 
//             0.5f      // c
//         );

//         // 3. 初始化 8 个轴的仿真状态
//         // 我们用 vector 来管理 8 个轴的数据
//         x1_sim_.resize(8, 0.0f);
//         x2_sim_.resize(8, 0.0f);
//         u_prev_.resize(8, 0.0f); // 存放上一时刻的控制量

//         // 发布器 (只发布第0轴的数据用于绘图检查)
//         pub_f_ = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis0/f_real", 10);
//         pub_z3_ = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis0/z3_est", 10);

//         start_time_ = this->now();

//         // 4. 定时器
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10), 
//             std::bind(&MimoNode::control_loop, this));
            
//         RCLCPP_INFO(this->get_logger(), "AVX2 Vectorized Simulation Started (8 Axes)!");
//     }

// private:
//     float sign(float x) { return (x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f); }

//     // 单个轴的物理仿真 (还是用标量算，模拟物理世界是并行的)
//     void simulate_single_plant(int axis_idx, float u_control, float dt, double t) {
//         float b_plant = 1.0f;
        
//         // 故意给每个轴加一点不一样的干扰，看看控制器能不能扛住
//         // 轴0: 标准正弦干扰
//         // 轴1: 常数干扰
//         // ...
//         float f_val = 0.0f;
//         if (axis_idx == 0) {
//             f_val = 1.0f * std::cos(0.6f * t) * x1_sim_[axis_idx] + 0.5f * sign(std::sin(t));
//         } else {
//             f_val = 0.5f * axis_idx; // 简单的常数干扰
//         }

//         // 记录轴0的干扰用于发布
//         if (axis_idx == 0) f_val_axis0_ = f_val;

//         float dx1 = x2_sim_[axis_idx];
//         float dx2 = f_val + b_plant * u_control;

//         x1_sim_[axis_idx] += dx1 * dt;
//         x2_sim_[axis_idx] += dx2 * dt;
//     }

//     void control_loop() {
//         float h = 0.01f;
//         double t = (this->now() - start_time_).seconds();

//         // --- Step 1: 物理仿真 (模拟 8 个电机在动) ---
//         // 这一步必须用循环，因为物理世界是独立的
//         for (int i = 0; i < 8; i++) {
//             simulate_single_plant(i, u_prev_[i], h, t);
//         }

//         // --- Step 2: 准备数据 (装车) ---
//         // 我们把 8 个轴的数据收集到连续的数组里
//         float y_batch[8];
//         float u_prev_batch[8];
//         float v0_batch[8]; // 目标值

//         for (int i = 0; i < 8; i++) {
//             y_batch[i] = x1_sim_[i];      // 测量值
//             u_prev_batch[i] = u_prev_[i]; // 上次控制量
            
//             // 设定不同的目标：
//             // 轴0 -> 1.0
//             // 轴1 -> -1.0
//             // 轴2 -> 0.0
//             // ...
//             v0_batch[i] = (i % 2 == 0) ? 1.0f : -1.0f; 
//         }

//         // ===========================================
//         // === Step 3: AVX2 核心时刻 (一键处理8轴) ===
//         // ===========================================
        
//         // 3.1 观测 (8轴并行)
//         adrc_vec_->observe(y_batch, u_prev_batch);

//         // 3.2 获取扰动 z3 (用于补偿)
//         float z3_batch[8];
//         adrc_vec_->getZ3(z3_batch);

//         // 3.3 计算控制律 u0 (8轴并行)
//         float u0_batch[8];
//         adrc_vec_->calculateU0(v0_batch, u0_batch);

//         // ===========================================

//         // --- Step 4: 后处理 (卸货 + 限幅) ---
//         for (int i = 0; i < 8; i++) {
//             float b0 = 1.0f;
//             float u_final = (u0_batch[i] - z3_batch[i]) / b0;

//             // 限幅
//             if (u_final > 20.0f) u_final = 20.0f;
//             if (u_final < -20.0f) u_final = -20.0f;

//             u_prev_[i] = u_final; // 更新用于下一次
//         }

//         // --- Step 5: 打印日志 (只看轴0和轴1，验证它们互不干扰) ---
//         if (count_++ % 50 == 0) {
//             RCLCPP_INFO(this->get_logger(), 
//                 "Axis 0 [Tgt: 1.0] -> y: %.3f | u: %.2f || Axis 1 [Tgt: -1.0] -> y: %.3f | u: %.2f",
//                 x1_sim_[0], u_prev_[0], x1_sim_[1], u_prev_[1]);
//         }

//         // 发布轴0数据供绘图
//         std_msgs::msg::Float32 msg_f, msg_z3;
//         msg_f.data = f_val_axis0_;
//         msg_z3.data = z3_batch[0];
//         pub_f_->publish(msg_f);
//         pub_z3_->publish(msg_z3);
//     }

//     std::shared_ptr<MimoADRC> adrc_vec_; // 向量版指针
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_f_, pub_z3_;
//     rclcpp::Time start_time_;
    
//     // 8 个轴的状态
//     std::vector<float> x1_sim_;
//     std::vector<float> x2_sim_;
//     std::vector<float> u_prev_;
    
//     float f_val_axis0_; // 用于记录
//     int count_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MimoNode>());
//     rclcpp::shutdown();
//     return 0;
// }


// -------------------------------------------------------------
// #include "rclcpp/rclcpp.hpp"
// #include "mimo_adrc/MimoADRC.h"
// #include "std_msgs/msg/float32.hpp"
// #include <cmath>
// #include <vector>

// class MimoNode : public rclcpp::Node {
// public:
//     MimoNode() : Node("voliro_sim_node"), count_(0) {
//         // === 核心修改：创建两个独立的控制器 ===
        
//         // 1. 位置控制器 (控制 x, y, z)
//         // 只有前3个通道有效
//         adrc_pos_ = std::make_shared<MimoADRC>();
//         // b0 ≈ 1/m = 1/3.0 = 0.33
//         adrc_pos_->init(0.01f, 5.0f, 0.33f); 
//         adrc_pos_->setGains(10.0f, 40.0f, 100.0f, 2.0f, 5.0f, 0.8f);

//         // 2. 姿态控制器 (控制 roll, pitch, yaw)
//         // 只有前3个通道有效
//         adrc_att_ = std::make_shared<MimoADRC>();
//         // b0 ≈ 1/J = 1/0.05 = 20.0
//         adrc_att_->init(0.01f, 50.0f, 20.0f);
//         adrc_att_->setGains(100.0f, 300.0f, 1000.0f, 20.0f, 5.0f, 0.8f);

//         // 3. 初始化状态
//         state_pos_.resize(6, 0.0f);
//         state_vel_.resize(6, 0.0f);
        
//         // 4. 控制量记录
//         u_pos_prev_.resize(8, 0.0f); // 记录力
//         u_att_prev_.resize(8, 0.0f); // 记录力矩

//         start_time_ = this->now();
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10), 
//             std::bind(&MimoNode::control_loop, this));
            
//         RCLCPP_INFO(this->get_logger(), "Voliro-T Dual-Core Control Started!");
//     }

// private:
//     float sat(float x, float limit) {
//         if (x > limit) return limit;
//         if (x < -limit) return -limit;
//         return x;
//     }

//     void simulate_voliro_dynamics(float Fx, float Fy, float Fz, float Mx, float My, float Mz, float dt) {
//         float m = 3.0f;
//         float g = 9.81f;
//         float Jxx = 0.04f, Jyy = 0.04f, Jzz = 0.08f;
//         float Cd_lin = 0.5f;

//         // 获取姿态
//         float phi   = state_pos_[3];
//         float theta = state_pos_[4];
//         float psi   = state_pos_[5];

//         // 旋转矩阵 R (Body -> World)
//         float cphi = cos(phi), sphi = sin(phi);
//         float cth  = cos(theta), sth  = sin(theta);
//         float cpsi = cos(psi), spsi = sin(psi);

//         // 简化的旋转矩阵应用 (只列出关键项)
//         // World Forces
//         float Fx_w = (cth*cpsi)*Fx + (cpsi*sphi*sth - cphi*spsi)*Fy + (sphi*spsi + cphi*cpsi*sth)*Fz;
//         float Fy_w = (cth*spsi)*Fx + (cphi*cpsi + sphi*spsi*sth)*Fy + (cphi*spsi*sth - cpsi*sphi)*Fz;
//         float Fz_w = (-sth)*Fx     + (cth*sphi)*Fy                  + (cth*cphi)*Fz;

//         // 动力学方程
//         float ax = Fx_w / m - Cd_lin * state_vel_[0];
//         float ay = Fy_w / m - Cd_lin * state_vel_[1];
//         float az = Fz_w / m - Cd_lin * state_vel_[2] - g;

//         float alpha_x = Mx / Jxx;
//         float alpha_y = My / Jyy;
//         float alpha_z = Mz / Jzz;

//         // 积分
//         state_pos_[0] += state_vel_[0] * dt; state_vel_[0] += ax * dt;
//         state_pos_[1] += state_vel_[1] * dt; state_vel_[1] += ay * dt;
//         state_pos_[2] += state_vel_[2] * dt; state_vel_[2] += az * dt;
//         state_pos_[3] += state_vel_[3] * dt; state_vel_[3] += alpha_x * dt;
//         state_pos_[4] += state_vel_[4] * dt; state_vel_[4] += alpha_y * dt;
//         state_pos_[5] += state_vel_[5] * dt; state_vel_[5] += alpha_z * dt;
//     }

//     void control_loop() {
//         float h = 0.01f;
//         double t = (this->now() - start_time_).seconds();

//         // 1. 运行仿真 (使用上一时刻的控制量)
//         simulate_voliro_dynamics(
//             u_pos_prev_[0], u_pos_prev_[1], u_pos_prev_[2], // Fx, Fy, Fz
//             u_att_prev_[0], u_att_prev_[1], u_att_prev_[2], // Mx, My, Mz (注意这里用力矩控制器的输出)
//             h
//         );

//         // 2. 设定目标
//         // 位置目标: 悬停在 1.0m
//         float v0_pos[8] = {0.0f, 0.0f, 1.0f, 0, 0, 0, 0, 0};
//         // 姿态目标: 晃动起来
//         float v0_att[8] = {
//             0.5f * (float)sin(t), // Roll
//             0.3f * (float)cos(t), // Pitch
//             0.0f,                 // Yaw
//             0, 0, 0, 0, 0
//         };

//         // 3. 准备测量数据
//         float y_pos[8] = {state_pos_[0], state_pos_[1], state_pos_[2], 0,0,0,0,0};
//         float y_att[8] = {state_pos_[3], state_pos_[4], state_pos_[5], 0,0,0,0,0};

//         // ==========================================
//         //         双核处理 (Dual Core)
//         // ==========================================
        
//         // --- 核心 1: 位置控制 ---
//         adrc_pos_->observe(y_pos, u_pos_prev_.data());
        
//         float z3_pos[8]; adrc_pos_->getZ3(z3_pos);
//         float u0_pos[8]; adrc_pos_->calculateU0(v0_pos, u0_pos);
        
//         for(int i=0; i<3; i++) {
//             // u = (u0 - z3) / b0
//             // 这里的 b0 必须和 init 里的一致 (0.33)
//             float u = (u0_pos[i] - z3_pos[i]) / 0.33f;
//             u_pos_prev_[i] = sat(u, 60.0f); // 稍微放宽推力限制
//         }

//         // --- 核心 2: 姿态控制 ---
//         adrc_att_->observe(y_att, u_att_prev_.data());
        
//         float z3_att[8]; adrc_att_->getZ3(z3_att);
//         float u0_att[8]; adrc_att_->calculateU0(v0_att, u0_att);

//         for(int i=0; i<3; i++) {
//             // b0 = 20.0
//             float u = (u0_att[i] - z3_att[i]) / 20.0f;
//             u_att_prev_[i] = sat(u, 10.0f); // 力矩限制
//         }

//         // 4. 日志
//         if (count_++ % 50 == 0) {
//             RCLCPP_INFO(this->get_logger(), "---------------------------------------");
//             // 重点看 Z 轴是否稳在 1.00，以及 Fz 是否合理 (~29.4)
//             RCLCPP_INFO(this->get_logger(), 
//                 "POS Z: %.3f (Tgt 1.0) | Fz: %.2f", 
//                 state_pos_[2], u_pos_prev_[2]);
            
//             // 重点看 Roll 是否跟随正弦波
//             RCLCPP_INFO(this->get_logger(), 
//                 "ATT Roll: %.3f (Tgt %.3f)", 
//                 state_pos_[3], v0_att[0]);
//         }
//     }

//     std::shared_ptr<MimoADRC> adrc_pos_;
//     std::shared_ptr<MimoADRC> adrc_att_;
    
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Time start_time_;
    
//     std::vector<float> state_pos_; // 6维
//     std::vector<float> state_vel_; // 6维
//     std::vector<float> u_pos_prev_; // 记录力
//     std::vector<float> u_att_prev_; // 记录力矩
    
//     int count_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MimoNode>());
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "mimo_adrc/MimoADRC.h"
// #include "std_msgs/msg/float32.hpp"
// #include <cmath>
// #include <vector>
// #include <string>

// class MimoNode : public rclcpp::Node {
// public:
//     MimoNode() : Node("voliro_sim_node"), count_(0) {
//         // === 1. 初始化控制器 ===
//         adrc_pos_ = std::make_shared<MimoADRC>();
//         adrc_pos_->init(0.01f, 5.0f, 0.33f); 
//         // 稍微加大增益以应对复合高难度
//         adrc_pos_->setGains(15.0f, 60.0f, 200.0f, 6.0f, 10.0f, 0.8f); 

//         adrc_att_ = std::make_shared<MimoADRC>();
//         adrc_att_->init(0.01f, 50.0f, 20.0f);
//         adrc_att_->setGains(100.0f, 300.0f, 1000.0f, 20.0f, 5.0f, 0.8f);

//         // === 2. 初始化状态 ===
//         state_pos_.resize(6, 0.0f);
//         state_vel_.resize(6, 0.0f);
//         u_pos_prev_.resize(8, 0.0f);
//         u_att_prev_.resize(8, 0.0f);

//         // === 3. 初始化发布者 ===
//         // Topic 1: 真实风力 (Axis 1)
//         pub_wind_real_ = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis1/f_real", 10);
//         // Topic 2: 观测风力 (Axis 1)
//         pub_wind_est_  = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis1/z3_est", 10);
//         // Topic 3: Y轴位置 (看轨迹是否被风吹歪)
//         pub_pos_y_     = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis1/pos_y", 10);

//         start_time_ = this->now();
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10), 
//             std::bind(&MimoNode::control_loop, this));
            
//         RCLCPP_INFO(this->get_logger(), "=== Voliro-T ULTIMATE TEST ===");
//         RCLCPP_INFO(this->get_logger(), "Task: Figure-8 + Pitch 30deg + 5N Crosswind (5s-10s)");
//     }

// private:
//     float sat(float x, float limit) {
//         if (x > limit) return limit;
//         if (x < -limit) return -limit;
//         return x;
//     }

//     void simulate_voliro_dynamics(float Fx, float Fy, float Fz, float Mx, float My, float Mz, float dt, double t) {
//         float m = 3.0f;
//         float g = 9.81f;
//         float Jxx = 0.04f, Jyy = 0.04f, Jzz = 0.08f;
//         float Cd_lin = 0.5f;

//         // === 注入风干扰 ===
//         float dist_Fy_world = 0.0f;
//         if (t > 5.0 && t < 10.0) {
//             dist_Fy_world = 5.0f; // 5N 横风
//         }
        
//         // 记录真实扰动加速度 (供 rqt_plot 对比)
//         // 注意：这里我们除以 m，因为 z3 估算的是加速度项
//         f_real_val_ = dist_Fy_world / m;

//         // 动力学解算
//         float phi = state_pos_[3], theta = state_pos_[4], psi = state_pos_[5];
//         float cphi = cos(phi), sphi = sin(phi);
//         float cth  = cos(theta), sth  = sin(theta);
//         float cpsi = cos(psi), spsi = sin(psi);

//         float Fx_w = (cth*cpsi)*Fx + (cpsi*sphi*sth - cphi*spsi)*Fy + (sphi*spsi + cphi*cpsi*sth)*Fz;
//         float Fy_w = (cth*spsi)*Fx + (cphi*cpsi + sphi*spsi*sth)*Fy + (cphi*spsi*sth - cpsi*sphi)*Fz;
//         float Fz_w = (-sth)*Fx     + (cth*sphi)*Fy                  + (cth*cphi)*Fz;

//         float ax = Fx_w / m - Cd_lin * state_vel_[0];
//         // 关键点：风加在 Y 轴上
//         float ay = (Fy_w + dist_Fy_world) / m - Cd_lin * state_vel_[1];
//         float az = Fz_w / m - Cd_lin * state_vel_[2] - g;

//         float alpha_x = Mx / Jxx;
//         float alpha_y = My / Jyy;
//         float alpha_z = Mz / Jzz;

//         state_pos_[0] += state_vel_[0] * dt; state_vel_[0] += ax * dt;
//         state_pos_[1] += state_vel_[1] * dt; state_vel_[1] += ay * dt;
//         state_pos_[2] += state_vel_[2] * dt; state_vel_[2] += az * dt;
//         state_pos_[3] += state_vel_[3] * dt; state_vel_[3] += alpha_x * dt;
//         state_pos_[4] += state_vel_[4] * dt; state_vel_[4] += alpha_y * dt;
//         state_pos_[5] += state_vel_[5] * dt; state_vel_[5] += alpha_z * dt;
//     }

//     void control_loop() {
//         float h = 0.01f;
//         double t = (this->now() - start_time_).seconds();

//         // 1. 运行仿真 (带风)
//         simulate_voliro_dynamics(
//             u_pos_prev_[0], u_pos_prev_[1], u_pos_prev_[2], 
//             u_att_prev_[0], u_att_prev_[1], u_att_prev_[2], 
//             h, t
//         );

//         // 2. 目标生成 (8字 + 抬头)
//         float traj_x = 2.0f * sin(0.5f * t);      
//         float traj_y = 2.0f * sin(1.0f * t);      
//         float traj_z = 1.0f; 
//         float target_pitch = 0.5f; // 抬头 30度
        
//         float v0_pos[8] = {traj_x, traj_y, traj_z, 0}; 
//         float v0_att[8] = {0.0f, target_pitch, 0.0f, 0}; 

//         // 3. ADRC 计算
//         float y_pos[8] = {state_pos_[0], state_pos_[1], state_pos_[2], 0};
//         float y_att[8] = {state_pos_[3], state_pos_[4], state_pos_[5], 0};

//         adrc_pos_->observe(y_pos, u_pos_prev_.data());
//         float z3_pos[8]; adrc_pos_->getZ3(z3_pos);
//         float u0_pos[8]; adrc_pos_->calculateU0(v0_pos, u0_pos);
//         for(int i=0; i<3; i++) {
//             float u = (u0_pos[i] - z3_pos[i]) / 0.33f;
//             u_pos_prev_[i] = sat(u, 60.0f);
//         }

//         adrc_att_->observe(y_att, u_att_prev_.data());
//         float z3_att[8]; adrc_att_->getZ3(z3_att);
//         float u0_att[8]; adrc_att_->calculateU0(v0_att, u0_att);
//         for(int i=0; i<3; i++) {
//             float u = (u0_att[i] - z3_att[i]) / 20.0f;
//             u_att_prev_[i] = sat(u, 10.0f);
//         }

//         // 4. 发布数据 (用于 rqt_plot)
//         std_msgs::msg::Float32 msg;
        
//         // 蓝线：真实风扰 (Step信号 0 -> 1.67)
//         msg.data = f_real_val_; 
//         pub_wind_real_->publish(msg);

//         // 青线：ESO观测到的总扰动 (应该包含风 + 动力学耦合)
//         // 注意：因为飞机在剧烈运动，z3里不仅有风，还有"惯性"和"耦合"
//         // 所以它不会是一条死直线，而是一条在 1.67 附近波动的线
//         msg.data = z3_pos[1]; 
//         pub_wind_est_->publish(msg);
        
//         // 红线：实际 Y 轴轨迹 (应该保持正弦波，不被吹偏)
//         msg.data = state_pos_[1];
//         pub_pos_y_->publish(msg);

//         if (count_++ % 20 == 0) {
//             std::string status = (t > 5.0 && t < 10.0) ? "[WIND!]" : "[Calm]";
//             RCLCPP_INFO(this->get_logger(), 
//                 "%s T:%.1f | X:%.2f(Ref:%.2f) | Y:%.2f(Ref:%.2f) | PITCH:%.2f", 
//                 status.c_str(), t, state_pos_[0], traj_x, state_pos_[1], traj_y, state_pos_[4]);
//         }
//     }

//     std::shared_ptr<MimoADRC> adrc_pos_, adrc_att_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Time start_time_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_wind_real_, pub_wind_est_, pub_pos_y_;
    
//     std::vector<float> state_pos_, state_vel_, u_pos_prev_, u_att_prev_;
//     float f_real_val_;
//     int count_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MimoNode>());
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "mimo_adrc/MimoADRC.h"
#include "std_msgs/msg/float32.hpp"
// 新增头文件
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

#include <cmath>
#include <vector>
#include <string>
#include <memory>

class MimoNode : public rclcpp::Node {
public:
    MimoNode() : Node("voliro_sim_node"), count_(0) {
        // === 1. 初始化控制器 ===
        adrc_pos_ = std::make_shared<MimoADRC>();
        adrc_pos_->init(0.01f, 5.0f, 0.33f); 
        adrc_pos_->setGains(15.0f, 60.0f, 200.0f, 6.0f, 10.0f, 0.8f); 

        adrc_att_ = std::make_shared<MimoADRC>();
        adrc_att_->init(0.01f, 50.0f, 20.0f);
        adrc_att_->setGains(100.0f, 300.0f, 1000.0f, 20.0f, 5.0f, 0.8f);

        // === 2. 初始化状态 ===
        state_pos_.resize(6, 0.0f); // [x, y, z, r, p, y]
        state_vel_.resize(6, 0.0f);
        u_pos_prev_.resize(8, 0.0f);
        u_att_prev_.resize(8, 0.0f);

        // === 3. 初始化发布者 (用于 rqt_plot 数据分析) ===
        pub_wind_real_ = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis1/f_real", 10);
        pub_wind_est_  = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis1/z3_est", 10);
        pub_pos_y_     = this->create_publisher<std_msgs::msg::Float32>("/adrc/axis1/pos_y", 10);

        // === 4. 初始化 3D 可视化工具 ===
        // TF 广播器 (核心：告诉 Rviz 飞机在哪)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Marker 发布者 (告诉 Rviz 画个方块)
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        start_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&MimoNode::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "=== Voliro-T 3D Visualization Started! ===");
        RCLCPP_INFO(this->get_logger(), "Task: Omni-Directional 8-Figure with 30deg Pitch");
    }

private:
    float sat(float x, float limit) {
        if (x > limit) return limit;
        if (x < -limit) return -limit;
        return x;
    }

    // 辅助函数：欧拉角转四元数
    tf2::Quaternion get_quaternion(float roll, float pitch, float yaw) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return q;
    }

    void simulate_voliro_dynamics(float Fx, float Fy, float Fz, float Mx, float My, float Mz, float dt, double t) {
        float m = 3.0f;
        float g = 9.81f;
        float Jxx = 0.04f, Jyy = 0.04f, Jzz = 0.08f;
        float Cd_lin = 0.5f;

        // === 注入风干扰 (为了3D观赏性，这次风力稍微关小一点，或者保留测试) ===
        float dist_Fy_world = 0.0f;
        if (t > 10.0 && t < 15.0) { // 第10秒开始刮风
            dist_Fy_world = 3.0f; 
        }
        
        f_real_val_ = dist_Fy_world / m;

        // 动力学解算
        float phi = state_pos_[3], theta = state_pos_[4], psi = state_pos_[5];
        float cphi = cos(phi), sphi = sin(phi);
        float cth  = cos(theta), sth  = sin(theta);
        float cpsi = cos(psi), spsi = sin(psi);

        float Fx_w = (cth*cpsi)*Fx + (cpsi*sphi*sth - cphi*spsi)*Fy + (sphi*spsi + cphi*cpsi*sth)*Fz;
        float Fy_w = (cth*spsi)*Fx + (cphi*cpsi + sphi*spsi*sth)*Fy + (cphi*spsi*sth - cpsi*sphi)*Fz;
        float Fz_w = (-sth)*Fx     + (cth*sphi)*Fy                  + (cth*cphi)*Fz;

        float ax = Fx_w / m - Cd_lin * state_vel_[0];
        float ay = (Fy_w + dist_Fy_world) / m - Cd_lin * state_vel_[1];
        float az = Fz_w / m - Cd_lin * state_vel_[2] - g;

        float alpha_x = Mx / Jxx;
        float alpha_y = My / Jyy;
        float alpha_z = Mz / Jzz;

        state_pos_[0] += state_vel_[0] * dt; state_vel_[0] += ax * dt;
        state_pos_[1] += state_vel_[1] * dt; state_vel_[1] += ay * dt;
        state_pos_[2] += state_vel_[2] * dt; state_vel_[2] += az * dt;
        state_pos_[3] += state_vel_[3] * dt; state_vel_[3] += alpha_x * dt;
        state_pos_[4] += state_vel_[4] * dt; state_vel_[4] += alpha_y * dt;
        state_pos_[5] += state_vel_[5] * dt; state_vel_[5] += alpha_z * dt;
    }

    void control_loop() {
        float h = 0.01f;
        double t = (this->now() - start_time_).seconds();

        // 1. 运行仿真
        simulate_voliro_dynamics(
            u_pos_prev_[0], u_pos_prev_[1], u_pos_prev_[2], 
            u_att_prev_[0], u_att_prev_[1], u_att_prev_[2], 
            h, t
        );

        // 2. 目标生成 (8字 + 抬头 30度)
        float traj_x = 2.0f * sin(0.5f * t);      
        float traj_y = 2.0f * sin(1.0f * t);      
        float traj_z = 1.0f; 
        float target_pitch = 0.5f; // 抬头!
        
        float v0_pos[8] = {traj_x, traj_y, traj_z, 0}; 
        float v0_att[8] = {0.0f, target_pitch, 0.0f, 0}; 

        // 3. ADRC 计算
        float y_pos[8] = {state_pos_[0], state_pos_[1], state_pos_[2], 0};
        float y_att[8] = {state_pos_[3], state_pos_[4], state_pos_[5], 0};

        adrc_pos_->observe(y_pos, u_pos_prev_.data());
        float z3_pos[8]; adrc_pos_->getZ3(z3_pos);
        float u0_pos[8]; adrc_pos_->calculateU0(v0_pos, u0_pos);
        for(int i=0; i<3; i++) {
            float u = (u0_pos[i] - z3_pos[i]) / 0.33f;
            u_pos_prev_[i] = sat(u, 60.0f);
        }

        adrc_att_->observe(y_att, u_att_prev_.data());
        float z3_att[8]; adrc_att_->getZ3(z3_att);
        float u0_att[8]; adrc_att_->calculateU0(v0_att, u0_att);
        for(int i=0; i<3; i++) {
            float u = (u0_att[i] - z3_att[i]) / 20.0f;
            u_att_prev_[i] = sat(u, 10.0f);
        }

        // 4. 数据发布 (rqt_plot)
        std_msgs::msg::Float32 msg;
        msg.data = f_real_val_; pub_wind_real_->publish(msg);
        msg.data = z3_pos[1];   pub_wind_est_->publish(msg);
        msg.data = state_pos_[1]; pub_pos_y_->publish(msg);

        // ==========================================
        // === 5. 3D 可视化逻辑 (WSL Rviz2) ===
        // ==========================================
        
        // A. 广播坐标变换 (World -> Body)
        geometry_msgs::msg::TransformStamped t_stamped;
        t_stamped.header.stamp = this->now();
        t_stamped.header.frame_id = "world";
        t_stamped.child_frame_id = "base_link";

        t_stamped.transform.translation.x = state_pos_[0];
        t_stamped.transform.translation.y = state_pos_[1];
        t_stamped.transform.translation.z = state_pos_[2];

        tf2::Quaternion q = get_quaternion(state_pos_[3], state_pos_[4], state_pos_[5]);
        t_stamped.transform.rotation.x = q.x();
        t_stamped.transform.rotation.y = q.y();
        t_stamped.transform.rotation.z = q.z();
        t_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t_stamped);

        // B. 发布红色方块 Marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "voliro_drone";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 尺寸: 长宽0.5米，厚0.1米
        marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.1;
        // 颜色: 红色
        marker.color.a = 1.0; 
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;

        pub_marker_->publish(marker);

        if (count_++ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "T:%.1f | X:%.2f | PITCH:%.2f | TF Broadcasted!", t, state_pos_[0], state_pos_[4]);
        }
    }

    std::shared_ptr<MimoADRC> adrc_pos_, adrc_att_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    
    // 数据发布
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_wind_real_, pub_wind_est_, pub_pos_y_;
    
    // 3D 发布
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    
    std::vector<float> state_pos_, state_vel_, u_pos_prev_, u_att_prev_;
    float f_real_val_;
    int count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MimoNode>());
    rclcpp::shutdown();
    return 0;
}