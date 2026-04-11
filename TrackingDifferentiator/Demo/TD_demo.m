clear,clc
addpath('..\Core');
%% 仿真参数
dt = 1e-4;
Ts = 0.02;
T = 12;
r = 50;         % 跟踪速率
h0 = 5*Ts;      % 滤波因子
n = 0.05;       % 噪声强度
rng(1);         % 随机数种子
%% 初始设置
f_fun = @(t) t + (t>2).*(2-t) + (t>4).*(2*t-8) + (t>6).*(12-2*t) + ...
             (t>8).*(24-3*t) + (t>10).*(3*t-30);
fd_fun = @(t) 1 + (t>2).*(-1) + (t>4).*(2) + (t>6).*(-2) + ...
             (t>8).*(-3) + (t>10).*(3);
%% 初始化
tspan = 0:dt:T;
N = length(tspan);
f = f_fun(tspan);
fd = fd_fun(tspan);
f_mea = zeros(1,N);
fd_eul = zeros(1,N);
x1 = zeros(1,N);
x2 = zeros(1,N);
TD = TrckDiff(Ts,r,h0);
%% 仿真计算
for k = 1:N
    if mod(tspan(k),Ts) == 0
        f_mea(k) = f(k)+n*randn;
        try
            fd_eul(k) = (f_mea(k) - f_mea(k-Ts/dt))/Ts;
        catch
            fd_eul(k) = 0;
        end
        TD.update(f_mea(k));
        [res1,res2] = TD.output;
        x1(k) = res2;
        x2(k) = res1;
    else
        f_mea(k) = f_mea(k-1);
        fd_eul(k) = fd_eul(k-1);
        x1(k) = x1(k-1);
        x2(k) = x2(k-1);
    end
end
%% 绘图设置
figure('Position', [100, 100, 800, 600]);  % 设置图形大小
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 14);
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultTextFontSize', 14);
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

% 子图1：信号跟踪
subplot(2,1,1)
plot(tspan, f, 'k-', 'LineWidth', 2, 'DisplayName', 'Original Signal'); hold on;
plot(tspan, f_mea, 'LineWidth', 1.5, 'DisplayName', 'Measured (with Noise)', 'Color', [0.5 0.5 0.5]);
plot(tspan, x1, 'r--', 'LineWidth', 2, 'DisplayName', 'TD (Tracking Signal)');
xlabel('$t$ (s)', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
legend('Location', 'best', 'Box', 'on', 'BackgroundAlpha', 0.7);
grid on;
set(gca, 'TickLabelInterpreter', 'latex');
title(sprintf('Signal Tracking $(\\mu = %.2f)$',n), 'FontWeight', 'normal', 'FontSize', 14);

% 子图2：微分跟踪
subplot(2,1,2)
plot(tspan, fd, 'k-', 'LineWidth', 2, 'DisplayName', 'Exact/Ideal Result)');
hold on;
plot(tspan, fd_eul, 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'DisplayName', 'Euler Result');
plot(tspan, x2, 'r--', 'LineWidth', 2, 'DisplayName', 'TD Result');
xlabel('$t$ (s)', 'FontSize', 12);
ylabel('Derivative', 'FontSize', 12);
legend('Location', 'best', 'Box', 'on', 'BackgroundAlpha', 0.7);
grid on;
set(gca, 'TickLabelInterpreter', 'latex');
title('Derivative Tracking', 'FontWeight', 'normal', 'FontSize', 14);

% 调整子图间距
subplot(2,1,1);
set(gca, 'Position', [0.10 0.59 0.80 0.31]);
subplot(2,1,2);
set(gca, 'Position', [0.10 0.10 0.80 0.31]);