clear,clc
addpath('..\Core');
%% 仿真参数
dt = 1e-4;
Ts = 0.01;
T = 10;
rng(1);         % 随机数种子
%% 系统参数（二阶系统：质量-弹簧-阻尼）
m = 1;          % 质量
c = 0.5;        % 阻尼系数
k = 2;          % 弹簧刚度
% 状态空间：x1=位置, x2=速度
A = [0 1; -k/m -c/m];     % 系统矩阵 (2x2)
B = [0; 1/m];             % 输入矩阵 (2x1)
Bd = [1 0; 0 1];          % 扰动输入矩阵 (2x2)
%% 扰动观测器增益
L = [-20 0; 0 -20];         % 观测增益 (2x2)
%% 初始条件
x_ini = [0.5; 0];         % 初始状态
x = x_ini;
%% 扰动信号（2x1）
d_fun = @(t) [0.3*sin(5*t) + 0.2; 0.5*sign(sin(2*pi*t))];
%% 仿真准备
tspan = 0:dt:T;
N = length(tspan);
u = zeros(1,N);           % 输入（设为零，仅观测扰动）
x1 = zeros(1,N);         % 状态1（位置）
x2 = zeros(1,N);         % 状态2（速度）
d1 = zeros(1,N);         % 真实扰动1
d2 = zeros(1,N);         % 真实扰动2
d1_hat = zeros(1,N);     % 估计扰动1
d2_hat = zeros(1,N);     % 估计扰动2
%% 初始化扰动观测器
DOB = LinearDOB(Ts, A, B, L, Bd, x_ini);
%% 仿真计算
for k = 1:N
    t = tspan(k);
    if mod(t, Ts) == 0
        d = d_fun(t);
        d1(k) = d(1);
        d2(k) = d(2);
        % 更新扰动观测器
        DOB.update(u(k), x);
        d1_hat(k) = DOB.d_hat(1);
        d2_hat(k) = DOB.d_hat(2);
    else
        d1(k) = d1(k-1);
        d2(k) = d2(k-1);
        d1_hat(k) = d1_hat(k-1);
        d2_hat(k) = d2_hat(k-1);
    end
    % 系统状态更新（欧拉法）
    dx = A*x + B*u(k) + Bd*d_fun(t);
    x = x + dt*dx;
    x1(k) = x(1);
    x2(k) = x(2);
end
%% 绘图设置
figure('Position', [100, 100, 800, 600]);
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 14);
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultTextFontSize', 14);
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

% 子图1：扰动1对比
subplot(2,1,1)
plot(tspan, d1, 'k-', 'LineWidth', 2, 'DisplayName', 'True Disturbance $d_1$'); hold on;
plot(tspan, d1_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated $\hat{d}_1$');
xlabel('$t$ (s)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
legend('Location', 'best', 'Box', 'on', 'BackgroundAlpha', 0.7);
grid on;
title('Disturbance 1 Tracking', 'FontWeight', 'normal', 'FontSize', 14);

% 子图2：扰动2对比
subplot(2,1,2)
plot(tspan, d2, 'k-', 'LineWidth', 2, 'DisplayName', 'True Disturbance $d_2$'); hold on;
plot(tspan, d2_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated $\hat{d}_2$');
xlabel('$t$ (s)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
legend('Location', 'best', 'Box', 'on', 'BackgroundAlpha', 0.7);
grid on;
title('Disturbance 2 Tracking', 'FontWeight', 'normal', 'FontSize', 14);

% 调整子图间距
subplot(2,1,1);
set(gca, 'Position', [0.10 0.60 0.80 0.35]);
subplot(2,1,2);
set(gca, 'Position', [0.10 0.10 0.80 0.35]);
