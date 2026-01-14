import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from tqdm import tqdm

# Physical Design Parameters
L1 = 250e-3
L2 = 210e-3
d1 = 94.5e-3
d2 = 112.5e-3
d3 = 102.5e-3
d4 = L2 - 115.5e-3

# Forward Kinetics

def forward_kinetics(theta1, theta2):
    theta12=theta1+theta2
    d5 = math.sqrt(d1**2 + d4**2 - 2*d1*d4*math.cos(theta12))
    # print("d5 = ", d5)

    theta3_p1 = math.acos((d4**2 + d5**2 - d1**2)/(2*d4*d5))
    theta3_p2 = math.acos((d3**2 + d5**2 - d2**2)/(2*d3*d5))
    theta3 = theta3_p1 + theta3_p2

    # print(f"theta3 = {theta3} rad / {math.degrees(theta3)} deg")

    L = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(theta3))
    theta4 = math.asin(L1/L*math.sin(theta3))
    theta = theta1-theta4
    # print("L = ", L)
    # print(f"theta = {theta} rad / {math.degrees(theta)} deg")
    return (L, theta)

# (L,theta) = forward_kinetics(theta1, theta2)

def analytical_jacobian(theta1, theta2):
    """解析法计算雅可比矩阵: J = [[dL/dθ1, dL/dθ2], [dθ/dθ1, dθ/dθ2]]"""
    
    # 前向计算中间变量
    theta12 = theta1 + theta2
    
    # 数值稳定性处理
    d5_sq = max(0, d1**2 + d4**2 - 2*d1*d4*math.cos(theta12))
    d5 = math.sqrt(d5_sq) if d5_sq > 0 else 1e-10
    
    # 计算A和B，并限制在[-1,1]范围内
    A = (d4**2 + d5**2 - d1**2) / (2*d4*d5) if d5 > 0 else 0
    A = max(-1, min(1, A))
    
    B = (d3**2 + d5**2 - d2**2) / (2*d3*d5) if d5 > 0 else 0
    B = max(-1, min(1, B))
    
    theta3_p1 = math.acos(A)
    theta3_p2 = math.acos(B)
    theta3 = theta3_p1 + theta3_p2
    
    L_sq = max(0, L1**2 + L2**2 - 2*L1*L2*math.cos(theta3))
    L = math.sqrt(L_sq) if L_sq > 0 else 1e-10
    
    D = L1 * math.sin(theta3) / L if L > 0 else 0
    D = max(-1, min(1, D))
    
    # 解析求导计算
    # d(d5)/d(theta1) = d(d5)/d(theta2)
    dd5_dtheta = d1 * d4 * math.sin(theta12) / d5
    
    # d(A)/d(d5) 和 d(B)/d(d5)
    dA_dd5 = (d5**2 - d4**2 + d1**2) / (2 * d4 * d5**2) if d4 > 0 and d5 > 0 else 0
    dB_dd5 = (d5**2 - d3**2 + d2**2) / (2 * d3 * d5**2) if d3 > 0 and d5 > 0 else 0
    
    # d(theta3)/d(d5)
    term1 = -1 / math.sqrt(max(0, 1 - A**2)) * dA_dd5 if 1 - A**2 > 0 else 0
    term2 = -1 / math.sqrt(max(0, 1 - B**2)) * dB_dd5 if 1 - B**2 > 0 else 0
    dtheta3_dd5 = term1 + term2
    
    # d(theta3)/d(theta1) = d(theta3)/d(theta2)
    dtheta3_dtheta1 = dtheta3_dd5 * dd5_dtheta
    dtheta3_dtheta2 = dtheta3_dtheta1  # 对称性
    
    # d(L)/d(theta3)
    dL_dtheta3 = L1 * L2 * math.sin(theta3) / L if L > 0 else 0
    
    # d(L)/d(theta1) 和 d(L)/d(theta2)
    dL_dtheta1 = dL_dtheta3 * dtheta3_dtheta1
    dL_dtheta2 = dL_dtheta3 * dtheta3_dtheta2
    
    # d(D)/d(theta3) 和 d(D)/d(L)
    dD_dtheta3 = L1 * math.cos(theta3) / L if L > 0 else 0
    dD_dL = -L1 * math.sin(theta3) / L**2 if L > 0 else 0
    
    # d(D)/d(theta1) 和 d(D)/d(theta2)
    dD_dtheta1 = dD_dtheta3 * dtheta3_dtheta1 + dD_dL * dL_dtheta1
    dD_dtheta2 = dD_dtheta3 * dtheta3_dtheta2 + dD_dL * dL_dtheta2
    
    # d(theta4)/d(theta1) 和 d(theta4)/d(theta2)
    dtheta4_dtheta1 = dD_dtheta1 / math.sqrt(max(0, 1 - D**2)) if 1 - D**2 > 0 else 0
    dtheta4_dtheta2 = dD_dtheta2 / math.sqrt(max(0, 1 - D**2)) if 1 - D**2 > 0 else 0
    
    # d(theta)/d(theta1) 和 d(theta)/d(theta2)
    dtheta_dtheta1 = 1 - dtheta4_dtheta1
    dtheta_dtheta2 = -dtheta4_dtheta2
    
    return [[dL_dtheta1, dL_dtheta2],
            [dtheta_dtheta1, dtheta_dtheta2]]

def numerical_jacobian(theta1, theta2, h=1e-6):
    """数值差分法计算雅可比矩阵（中心差分）"""
    
    # 计算基准点
    L0, theta0 = forward_kinetics(theta1, theta2)
    
    # θ1方向的扰动
    L1_plus, theta1_plus = forward_kinetics(theta1 + h, theta2)
    L1_minus, theta1_minus = forward_kinetics(theta1 - h, theta2)
    
    # θ2方向的扰动
    L2_plus, theta2_plus = forward_kinetics(theta1, theta2 + h)
    L2_minus, theta2_minus = forward_kinetics(theta1, theta2 - h)
    
    # 中心差分计算偏导数
    dL_dtheta1 = (L1_plus - L1_minus) / (2 * h)
    dtheta_dtheta1 = (theta1_plus - theta1_minus) / (2 * h)
    
    dL_dtheta2 = (L2_plus - L2_minus) / (2 * h)
    dtheta_dtheta2 = (theta2_plus - theta2_minus) / (2 * h)
    
    return [[dL_dtheta1, dL_dtheta2],
            [dtheta_dtheta1, dtheta_dtheta2]]


def force_convertsion(F_L, T_theta, theta1, theta2):
    """
    将末端执行器的力/扭矩转换为关节电机扭矩
    
    参数:
        F_L (float): 沿L方向的线性力 (N)
        T_theta (float): 绕theta角的扭矩 (N·m)
        theta1 (float): 关节1角度 (rad)
        theta2 (float): 关节2角度 (rad)
    
    返回:
        tuple: (tau1, tau2) - theta1和theta2上的电机扭矩 (N·m)
    """
    # 计算解析雅可比矩阵 J = [[dL/dθ1, dL/dθ2], [dθ/dθ1, dθ/dθ2]]
    J = analytical_jacobian(theta1, theta2)
    
    # 提取雅可比矩阵元素
    dL_dtheta1, dL_dtheta2 = J[0][0], J[0][1]
    dtheta_dtheta1, dtheta_dtheta2 = J[1][0], J[1][1]
    
    # 根据虚功原理: τ = J^T · F
    # 其中 τ = [τ1, τ2]^T, F = [F_L, T_theta]^T
    tau1 = dL_dtheta1 * F_L + dtheta_dtheta1 * T_theta
    tau2 = dL_dtheta2 * F_L + dtheta_dtheta2 * T_theta
    
    return (tau1, tau2)


# # 测试代码
# if __name__ == "__main__":
#     # 给定输入角度
#     # theta1 = math.pi/4
#     # theta2 = math.pi/4

#     theta1 = 0.8
#     theta2 = 0.8203*2 - theta1
    
#     print("=== 前向运动学 ===")
#     L, theta = forward_kinetics(theta1, theta2)
#     print(f"L = {L:.6f}")
#     print(f"θ = {theta:.6f} rad ({math.degrees(theta):.2f}°)")
#     print()
    
#     print("=== 解析雅可比矩阵 ===")
#     J_analytical = numerical_jacobian(theta1, theta2)
#     print(f"[[{J_analytical[0][0]:.6e}, {J_analytical[0][1]:.6e}]")
#     print(f" [{J_analytical[1][0]:.6e}, {J_analytical[1][1]:.6e}]]")
#     print()
    
#     # print("=== 数值雅可比矩阵 ===")
#     # J_numerical = numerical_jacobian(theta1, theta2, h=1e-6)
#     # print(f"[[{J_numerical[0][0]:.6e}, {J_numerical[0][1]:.6e}]")
#     # print(f" [{J_numerical[1][0]:.6e}, {J_numerical[1][1]:.6e}]]")
#     # print()
    
#     # print("=== 结果对比 ===")
#     # print(f"最大元素误差: {max(abs(J_analytical[i][j] - J_numerical[i][j]) for i in range(2) for j in range(2)):.2e}")

#     F_L_test = -100.0      # N, 沿L方向向上的力
#     T_theta_test = 0.0  # N·m, 绕theta角的扭矩
    
#     # 计算电机扭矩
#     tau1, tau2 = force_convertsion(F_L_test, T_theta_test, theta1, theta2)
    
#     # 打印结果
#     print("=== 力/扭矩转换结果 ===")
#     print(f"输入 - F_L: {F_L_test} N, T_θ: {T_theta_test} N·m")
#     print(f"关节角度 - θ1: {math.degrees(theta1):.2f}°, θ2: {math.degrees(theta2):.2f}°")
#     print(f"输出电机扭矩:")
#     print(f"  τ1 (θ1轴): {tau1:.6f} N·m")
#     print(f"  τ2 (θ2轴): {tau2:.6f} N·m")
# ==================== Inverse Kinematics (Newton-Raphson) ====================
# ==================== Inverse Kinematics (Newton-Raphson) ====================
def inverse_kinematics_numerical(L_target, theta_target_deg, initial_guess_deg, 
                                 max_iter=50, tolerance=1e-5, verbose=False):
    """牛顿-拉夫逊法求逆运动学（角度输入输出为度）"""
    
    # 转换到弧度
    theta_target_rad = math.radians(theta_target_deg)
    theta1, theta2 = [math.radians(x) for x in initial_guess_deg]
    
    for i in range(max_iter):
        L_current, theta_current_deg = forward_kinetics(theta1, theta2)
        
        if L_current is None:
            if verbose:
                print(f"错误: 第{i+1}次迭代遇到无效配置")
            return theta1, theta2, False, i+1, float('inf')
        
        # 误差计算（目标theta转换为度）
        error_L = L_target - L_current
        error_theta = theta_target_deg - theta_current_deg
        error_norm = math.sqrt(error_L**2 + error_theta**2)
        
        if error_norm < tolerance:
            if verbose:
                print(f"\n>>> 收敛成功! 迭代次数: {i+1}")
                print(f"最终误差: {error_norm:.2e}")
            # 返回度
            return math.degrees(theta1), math.degrees(theta2), True, i+1, error_norm
        
        # 计算雅可比矩阵
        J = analytical_jacobian(theta1, theta2)
        det = J[0][0] * J[1][1] - J[0][1] * J[1][0]
        
        if abs(det) < 1e-12:
            if verbose:
                print(f"\n错误: 第{i+1}次迭代雅可比矩阵奇异 (det={det:.2e})")
            return math.degrees(theta1), math.degrees(theta2), False, i+1, error_norm
        
        # 更新关节角（弧度）
        inv_J = [[J[1][1]/det, -J[0][1]/det],
                 [-J[1][0]/det, J[0][0]/det]]
        
        delta_theta1 = inv_J[0][0] * error_L + inv_J[0][1] * error_theta
        delta_theta2 = inv_J[1][0] * error_L + inv_J[1][1] * error_theta
        
        theta1 += delta_theta1
        theta2 += delta_theta2
    
    # 返回度
    return math.degrees(theta1), math.degrees(theta2), False, max_iter, error_norm

# ==================== Dynamic Simulation (SI Units) ====================
def simulate_L_motion(theta_fixed_deg, l_start, l_end, duration, force_L=10.0, num_steps=100):
    """
    模拟L在固定theta下变化的运动过程（SI单位）
    
    参数:
        theta_fixed_deg: 固定的theta角度（度）
        l_start: 初始L长度 (m)
        l_end: 结束L长度 (m)
        duration: 运动时间（秒）
        force_L: L方向的恒定力 (N)
        num_steps: 离散化步数
    
    返回:
        dict: 包含时间、关节角、速度、扭矩、功率等数据（SI单位）
    """
    # 转换到弧度
    theta_fixed_rad = math.radians(theta_fixed_deg)
    
    # 生成时间数组
    dt = duration / num_steps
    time_array = [i * dt for i in range(num_steps + 1)]
    
    # 线性插值计算L
    L_array = [l_start + (l_end - l_start) * t / duration for t in time_array]
    
    # 恒定速度 (m/s)
    dL_dt = (l_end - l_start) / duration
    
    # 存储结果
    results = {
        'time': time_array,
        'L': L_array,
        'theta1_deg': [],
        'theta2_deg': [],
        'omega1_rad_s': [],
        'omega2_rad_s': [],
        'tau1_Nm': [],
        'tau2_Nm': [],
        'power1_W': [],
        'power2_W': []
    }
    
    # 遍历每个时间点
    prev_solution_deg = (30.0, 30.0)  # 初始猜测（度）
    
    for i, L_target in enumerate(L_array):
        # 求解逆运动学
        theta1_deg, theta2_deg, success, iters, error = inverse_kinematics_numerical(
            L_target, theta_fixed_deg, prev_solution_deg, 
            max_iter=50, tolerance=1e-5, verbose=False
        )
        
        if not success:
            if i % 10 == 0:
                print(f"警告: 第{i}个点IK失败, t={time_array[i]:.3f}s, error={error:.6e}")
            if results['theta1_deg']:
                theta1_deg = results['theta1_deg'][-1]
                theta2_deg = results['theta2_deg'][-1]
            else:
                theta1_deg, theta2_deg = prev_solution_deg
        
        # 保存解作为下一个点的初始猜测
        prev_solution_deg = (theta1_deg, theta2_deg)
        
        results['theta1_deg'].append(theta1_deg)
        results['theta2_deg'].append(theta2_deg)
        
        # 转换为弧度计算雅可比
        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg)
        
        J = analytical_jacobian(theta1_rad, theta2_rad)
        det_J = J[0][0] * J[1][1] - J[0][1] * J[1][0]
        
        # 计算关节速度 (rad/s) = J⁻¹ · [dL/dt, 0]
        if abs(det_J) < 1e-12:
            if i % 10 == 0:
                print(f"警告: t={time_array[i]:.3f}s 雅可比奇异")
            omega1 = results['omega1_rad_s'][-1] if results['omega1_rad_s'] else 0.0
            omega2 = results['omega2_rad_s'][-1] if results['omega2_rad_s'] else 0.0
        else:
            inv_J = [[J[1][1]/det_J, -J[0][1]/det_J],
                     [-J[1][0]/det_J, J[0][0]/det_J]]
            omega1 = inv_J[0][0] * dL_dt
            omega2 = inv_J[1][0] * dL_dt
        
        results['omega1_rad_s'].append(omega1)
        results['omega2_rad_s'].append(omega2)
        
        # 扭矩 τ = Jᵀ · F (N·m)
        tau1 = J[0][0] * force_L
        tau2 = J[0][1] * force_L
        
        results['tau1_Nm'].append(tau1)
        results['tau2_Nm'].append(tau2)
        
        # 功率 P = τ · ω (W)
        power1 = tau1 * omega1
        power2 = tau2 * omega2
        
        results['power1_W'].append(power1)
        results['power2_W'].append(power2)
    
    return results

# ==================== Plotting Function (SI + Deg) ====================
def plot_simulation(results, theta_fixed_deg, save_path=None):
    """绘制电机动态性能曲线（SI单位）"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), dpi=100)
    
    time = results['time']
    
    # 图1: 电机速度
    ax1 = axes[0]
    ax1.plot(time, results['omega1_rad_s'], 'b-', linewidth=2.5, label='Motor 1 (θ₁)')
    ax1.plot(time, results['omega2_rad_s'], 'r-', linewidth=2.5, label='Motor 2 (θ₂)')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    ax1.set_title(f'Motor Angular Velocity (θ fixed={theta_fixed_deg:.1f}°)', fontsize=14, fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='best', fontsize=11)
    ax1.tick_params(axis='both', which='major', labelsize=10)
    
    # 图2: 电机扭矩
    ax2 = axes[1]
    ax2.plot(time, results['tau1_Nm'], 'b-', linewidth=2.5, label='Motor 1 (θ₁)', alpha=0.9)
    ax2.plot(time, results['tau2_Nm'], 'r-', linewidth=2.5, label='Motor 2 (θ₂)', alpha=0.9)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Torque (N·m)', fontsize=12)
    ax2.set_title(f'Motor Torque (F_L={10}N)', fontsize=14, fontweight='bold')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='best', fontsize=11)
    ax2.tick_params(axis='both', which='major', labelsize=10)
    
    # 图3: 电机功率
    ax3 = axes[2]
    ax3.plot(time, results['power1_W'], 'b-', linewidth=2.5, label='Motor 1 (θ₁)')
    ax3.plot(time, results['power2_W'], 'r-', linewidth=2.5, label='Motor 2 (θ₂)')
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Power (W)', fontsize=12)
    ax3.set_title('Motor Power', fontsize=14, fontweight='bold')
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='best', fontsize=11)
    ax3.tick_params(axis='both', which='major', labelsize=10)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n图像已保存至: {save_path}")
    
    plt.show()

# ==================== Main Simulation Runner ====================
def run_simulation_example():
    """运行完整的仿真示例（SI单位）"""
    
    # 仿真参数（SI单位）
    theta_fixed = 0.0  # 固定theta=30°（输入为deg）
    l_start = 0.20      # 初始L长度 (m)
    l_end = 0.40        # 结束L长度 (m)
    duration = 1.0      # 1秒
    force_L = 10.0      # L方向力 (N)
    
    print("="*70)
    print("开始仿真：L在固定theta下变化 (SI单位)")
    print("="*70)
    print(f"固定θ: {theta_fixed:.1f}°")
    print(f"L范围: {l_start*100:.1f}cm → {l_end*100:.1f}cm")
    print(f"运动时间: {duration}s")
    print(f"L方向力: {force_L}N")
    print(f"数据点: 101个")
    print("="*70 + "\n")
    
    # 运行动态仿真
    results = simulate_L_motion(
        theta_fixed_deg=theta_fixed,
        l_start=l_start,
        l_end=l_end,
        duration=duration,
        force_L=force_L,
        num_steps=100
    )
    
    # 绘制结果
    plot_simulation(results, theta_fixed, save_path="motor_dynamics_SI.png")
    
    # 打印统计信息
    print("\n" + "="*70)
    print("仿真统计结果 (SI单位)")
    print("="*70)
    fmt = "{:<25s} {:>12s} {:>12s}"
    print(fmt.format("参数", "电机1 (θ₁)", "电机2 (θ₂)"))
    print("-"*70)
    
    print(fmt.format(
        "最大角速度 (rad/s)", 
        f"{max(results['omega1_rad_s']):>12.3f}", 
        f"{max(results['omega2_rad_s']):>12.3f}"
    ))
    print(fmt.format(
        "最大扭矩 (N·m)", 
        f"{max(results['tau1_Nm']):>12.4f}", 
        f"{max(results['tau2_Nm']):>12.4f}"
    ))
    print(fmt.format(
        "最大功率 (W)", 
        f"{max(results['power1_W']):>12.4f}", 
        f"{max(results['power2_W']):>12.4f}"
    ))
    print(fmt.format(
        "平均功率 (W)", 
        f"{sum(results['power1_W'])/len(results['power1_W']):>12.4f}", 
        f"{sum(results['power2_W'])/len(results['power2_W']):>12.4f}"
    ))
    print("="*70)
    
    return results

# ==================== Run Simulation ====================
if __name__ == "__main__":
    results = run_simulation_example()