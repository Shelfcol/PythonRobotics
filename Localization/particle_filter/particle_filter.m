%粒子滤波(定位运动轨迹)
%在二维空间,假设运动物体的一组(非线性)运动位置、速度、加速度数据,用粒子滤波方法进行处理
clc,clear,close all
%% 参数设置
N = 200; %粒子总数
Q = 5; %过程噪声(控制误差)  状态转移方程中使用
R = 5; %测量噪声  由真实位置叠加测量噪声得到测量位置
T = 50; %测量时间(总步数)
theta = pi/T; %旋转角度
distance = 80/T; %每次走的距离(步长)
WorldSize = 100; %世界大小
X = zeros(2, T); %存储系统状态(每列存储二维位置坐标(x,y),共T个位置)
Z = zeros(2, T); %存储系统的观测状态(每列存储二维位置坐标(x,y),共T次测量)
P = zeros(2, N); %建立粒子群(每列存储当前粒子的二维位置坐标,共N个粒子)
PCenter = zeros(2, T); %所有粒子的中心位置
w = zeros(N, 1); %每个粒子的权重
err = zeros(2,T); %误差(第一行为粒子与真实路径误差  第二行为测量与真实路径误差)
X(:, 1) = [50; 20]; %初始系统状态 即初始位置在坐标(50,20)
Z(:, 1) = X(:,1) + wgn(2,1,10*log10(R)); %初始系统的观测状态(为真实位姿叠加高斯噪声)，
                                             %y = wgn(m,n,p) 产生一个m行n列的高斯白噪声的矩阵，p以dBW为单位指定输出噪声的强度
%% 初始化粒子群
for i = 1 : N
    P(:, i) = [WorldSize*rand; WorldSize*rand];%随机产生第i个粒子的坐标(rand为产生[0,1]之间均匀分布)
    dist = norm(P(:, i)-Z(:, 1)); %与测量位置相差的距离,返回二范数，这个与观测值有关系，如果观测值很不准的话，则这个也不准
    %求权重 (权重与距离的关系 为 均值是0,方差是sqrt(R)的高斯分布曲线)  因为均值为0且距离大于0 因此权重随着距离增加沿高斯曲线右侧递减
    w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R); %高斯方程
end
PCenter(:, 1) = sum(P, 2) / N;%t=1时刻(初始时刻)所有粒子的几何中心位置
% 初始状态(t=1)画图
err(1,1) = norm(X(:, 1) - PCenter(:, 1));%粒子群几何中心与系统真实状态的误差
err(2,1) = wgn(1, 1, 10*log10(R));
figure(1);
hold on
set(0,'defaultfigurecolor','w')
plot(X(1, 1), X(2, 1), 'r.', 'markersize',30) %真实的初始状态位置(红点表示)
%grid on
axis([0 100 0 100]);
set(gca,'XTick',0:10:100) %改变x轴坐标间隔显示 这里间隔为10
set(gca,'YTick',0:10:100) %改变y轴坐标间隔显示 这里间隔为10
plot(P(1, :), P(2, :), 'k.', 'markersize',5); %各个粒子位置(N个黑点)
plot(PCenter(1, 1), PCenter(2, 1), 'b.', 'markersize',25); %所有粒子的中心位置(蓝点表示)
legend('真实位置', '粒子群', '粒子群的几何中心');
title('初始状态');
hold off
%% 开始运动
for k = 2 : T %从t=2到T
    %模拟一个弧线运动的状态
    X(:, k) = X(:, k-1) + distance * [(-cos(k * theta)); sin(k * theta)] + wgn(2, 1, 10*log10(Q)); %状态方程
    Z(:, k) = X(:, k) + wgn(2, 1, 10*log10(R)); %观测方程(状态上叠加测量的高斯噪声) 
    %粒子滤波
    %预测
    for i = 1 : N
        P(:, i) = P(:, i) + distance * [-cos(k * theta); sin(k * theta)] + wgn(2, 1, 10*log10(Q));%粒子群带入状态方程
        dist = norm(P(:, i)-Z(:, k)); %粒子群中各粒子 与 测量位置 的距离
        w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R); %求权重(距离近权重大)
    end
    %归一化权重
    wsum = sum(w);
    w = w / wsum;
%     for i = 1 : N
%         w(i) = w(i) / wsum;
%     end
    %重采样（更新）
    for i = 1 : N
        wmax = 2 * max(w) * rand; %另一种重采样规则
        index = randi(N, 1);%生成一个在[1(默认值),N]之间均匀分布的伪随机整数
        while(wmax > w(index))
            wmax = wmax - w(index);
            index = index + 1;
            if index > N
                index = 1;
            end 
        end
        Pnext(:, i) = P(:, index); %得到新粒子放入临时集Pnext
    end
    P=Pnext;%用临时集Pnext更新粒子集P
    PCenter(:, k) = sum(P, 2) / N; %重采样后所有粒子的中心位置
    %计算误差
    err(1,k) = norm(X(:, k) - PCenter(:, k)); %粒子几何中心与系统真实状态的误差
    err(2,k) = norm(X(:, k) - Z(:, k));
    %画图
    figure(2);
    set(0,'defaultfigurecolor','w')
    clf;%清空figure(2)中的图像 以便循环重新画
    hold on
    plot(X(1, k), X(2, k), 'r.', 'markersize',30); %系统状态位置
    plot(P(1, :), P(2, :), 'k.', 'markersize',5); %各个粒子位置
    plot(PCenter(1, k), PCenter(2, k), 'b.', 'markersize',25); %所有粒子的中心位置
    axis([0 100 0 100]);
    title('运动过程');
    legend('真实状态', '粒子群', '粒子群的几何中心');
    hold off
    pause(0.1);%停0.1s开始下次迭代
end
%% 绘制轨迹
figure(3);
set(0,'defaultfigurecolor','w')
plot(X(1,:), X(2,:), 'r.-', Z(1,:), Z(2,:), 'g.-', PCenter(1,:), PCenter(2,:), 'b.-');
axis([0 100 0 100]);
set(gca,'XTick',0:10:100) %改变x轴坐标间隔显示 这里间隔为10
set(gca,'YTick',0:10:100) %改变y轴坐标间隔显示 这里间隔为10
legend('真实轨迹', '测量轨迹', '粒子群几何中心轨迹');
xlabel('横坐标 x'); ylabel('纵坐标 y');
%% 绘制误差
figure(4);
set(0,'defaultfigurecolor','w')
%set(gca,'FontSize',12);%设置图标字体大小
plot(err(1,:),'b.-');%err1为各时刻 真实位置与粒子群中心的几何距离
hold on
plot(err(2,:),'r.-');%err2为各时刻 真实位置与测量位置的几何距离
hold off
xlabel('步数 t');
legend('粒子群误差', '测量误差');
title('真实位置与粒子群中心的集合距离');
