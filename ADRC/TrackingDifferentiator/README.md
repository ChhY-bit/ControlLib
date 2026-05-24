# Tracking Differentiator

本项目是一个基于 MATLAB 的跟踪微分器（Tracking Differentiator, TD）实现，主要用于从含噪声的信号中提取微分信号。

## 项目结构

```
TrackingDifferentiator/
├── Core/           # 核心实现
│   └── TrckDiff.m  # 跟踪微分器类
├── Demo/           # 示例代码
│   └── TD_demo.m   # 使用示例与对比仿真
└── Doc/            # 文档资料
    ├── Document.md # 详细原理说明
    └── TD_demo.svg # 仿真结果图示
```

## 功能特点

- **信号跟踪**：能够实时跟踪输入信号
- **微分提取**：从信号中提取微分信息
- **噪声抑制**：相比传统差分方法具有更好的抗噪性能
- **面向对象**：采用 MATLAB 类封装，易于集成和使用

## 快速开始

```matlab
% 初始化
TD = TrckDiff(h, r, h0);  % h:采样周期, r:跟踪速率, h0:滤波因子

% 更新与输出
TD.update(input);         % 输入待微分信号
[diff, trck] = TD.output(); % 获取微分和跟踪结果
```

## 详细说明

- 核心算法原理与参数说明请参考 `Doc/Document.md`
- 完整使用示例请查看 `Demo/TD_demo.m`

## 作者

- **C.Yang** - [GitHub: https://github.com/ChhY-bit](https://github.com/ChhY-bit)
- 邮箱：yangchenhan.work@gmail.com / ych_0872@126.com

## 版本

v1.0 (2026-04-07)
