function []=readXY(filename)
fid=fopen(filename);
data=textscan(fid,'%f %f');
fclose(fid);
 %计算XY均值方差
 X=data{1};
 Y=data{2};
 aveX=mean(X);
 aveY=mean(Y);
 varX=var(X);
 varY=var(Y);
 %输出均值方差结果
 disp(['aveX=', num2str(aveX),'m,aveY=', num2str(aveY),'m']);
 disp(['var(X)=', num2str(varX),'m*m,var(Y)=', num2str(varY),'m*m']);
 %绘制X时间序列图
 subplot(2, 2, 1);
 plot(1:200,X);
 title('X坐标时间序列图');
 xlabel('时间/s');
 ylabel('X/m');
 %绘制X分布柱状图
 subplot(2, 2, 2);
 hist(X,10);
 title('X坐标分布柱状图');
 xlabel('X/m');
 ylabel('频率');
  %绘制Y时间序列图
 subplot(2, 2, 3)
 plot(1:200,Y);
 title('Y坐标时间序列图');
 xlabel('时间/s');
 ylabel('Y/m');
  %绘制X分布柱状图
 subplot(2, 2, 4);
 hist(Y,10);
 title('Y坐标分布柱状图');
 xlabel('Y/m');
 ylabel('频率');