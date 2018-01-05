function []=KalmanFilter()
filename='observation.txt';
fid=fopen(filename);
data=textscan(fid,'%f %f','HeaderLines',1);
fclose(fid);
R=data{1};
alpha=data{2};
T=0.1; g=9.8; kx=0.01; ky=0.05;
i=1;size=2;
X0=[0;50;500;0];X10=zeros(4,1);
D0=diag([100,100,100,100]);
De=diag([1.5*1.5,1.5*1.5]);D_delta=diag([100,0.0001]);H=zeros(2,4);
D=zeros(length(R),4);
phi=zeros(4,4);
gamma=zeros(4,2);
phi(1,1)=1;phi(3,3)=1;
gamma(1,1)=0.5*T*T;gamma(2,1)=T;gamma(3,2)=0.5*T*T;gamma(4,2)=T;
while i<length(R)+1
    figure(1);
    plot(R(i)*sin(alpha(i)),R(i)*cos(alpha(i)),'bo');hold on;
    %数据准备
    phi(1,2)=T-kx*T*T*X0(2);
    phi(2,2)=1-2*kx*T*X0(2);
    phi(3,4)=T+ky*T*T*X0(4);
    phi(4,4)=1+2*ky*T*X0(4);
    
    %一步预测——状态预测
    X10(1)=X0(1)+X0(2)*T+0.5*(-kx*X0(2)*X0(2))*T*T;
    X10(2)=X0(2)-kx*X0(2)*X0(2)*T;
    X10(3)=X0(3)+X0(4)*T+0.5*(ky*X0(4)*X0(4)-g)*T*T;
    X10(4)=X0(4)+(ky*X0(4)*X0(4)-g)*T;
    %预测方差
    D10=phi*D0*phi'+gamma*De*gamma';
    %绘制
    %figure(1);
    plot(X10(1),X10(3),'g*');hold on;
    %观测值逐次更新
    j=1;
    square=X10(1)*X10(1)+X10(3)*X10(3);
    H(1,1)=X10(1)/sqrt(square);
    H(1,3)=X10(3)/sqrt(square);
    H(2,1)=X10(3)/square;
    H(2,3)=-X10(1)/square;
    %Z(1,1)=R(i);Z(2,1)=alpha(i);
    V=zeros(2,1);
    while j<size+1
        tempD=D10;tempX=X10;
        %增益矩阵
        h=H(j,:);
        K=tempD*h'/(h*tempD*h'+D_delta(j,j));
        %新息序列
        if(j==1)
            V(1,1)=R(i)-sqrt(tempX(1)*tempX(1)+tempX(3)*tempX(3));
        end
        if(j==2)
            V(2,1)=alpha(i)-atan(tempX(1)/tempX(3));
        end
        %状态滤波
        X10=tempX+K*V(j,1);
        %滤波方差
        D10=(eye(4)-K*h)*tempD;
        j=j+1;
    end
    X0=X10;D0=D10;
    D(i,1)=D0(1,1);D(i,2)=D0(2,2);D(i,3)=D0(3,3);D(i,4)=D0(4,4);
    %绘制
    plot(X10(1),X10(3),'r+');hold on;
    i=i+1;
end
title('扩展的卡尔曼滤波估计物体轨迹图');
xlabel('X/m');ylabel('Y/m');
legend('观测值','预测值','滤波值');
figure(2);
subplot(2,2,1);
plot(1:length(R),D(:,1),'b.');
title('X滤波方差变化');xlabel('时间');ylabel('X/m*m');
subplot(2,2,2);
plot(1:length(R),D(:,3),'g.');
title('Y滤波方差变化');xlabel('时间');ylabel('Y/m*m');
subplot(2,2,3);
plot(1:length(R),D(:,2),'r.');
title('水平方向速度滤波方差变化');xlabel('时间');ylabel('V/(m*m/(s*s))');
subplot(2,2,4);
plot(1:length(R),D(:,4),'c.');
title('竖直方向速度滤波方差变化');xlabel('时间');ylabel('V/(m*m/(s*s))');
fid1=figure(1);
set(fid1,'position',[100 80 600 600]);
fid1=figure(2);
set(fid1,'position',[700 80 600 600]);