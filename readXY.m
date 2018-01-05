function []=readXY(filename)
fid=fopen(filename);
data=textscan(fid,'%f %f');
fclose(fid);
 %����XY��ֵ����
 X=data{1};
 Y=data{2};
 aveX=mean(X);
 aveY=mean(Y);
 varX=var(X);
 varY=var(Y);
 %�����ֵ������
 disp(['aveX=', num2str(aveX),'m,aveY=', num2str(aveY),'m']);
 disp(['var(X)=', num2str(varX),'m*m,var(Y)=', num2str(varY),'m*m']);
 %����Xʱ������ͼ
 subplot(2, 2, 1);
 plot(1:200,X);
 title('X����ʱ������ͼ');
 xlabel('ʱ��/s');
 ylabel('X/m');
 %����X�ֲ���״ͼ
 subplot(2, 2, 2);
 hist(X,10);
 title('X����ֲ���״ͼ');
 xlabel('X/m');
 ylabel('Ƶ��');
  %����Yʱ������ͼ
 subplot(2, 2, 3)
 plot(1:200,Y);
 title('Y����ʱ������ͼ');
 xlabel('ʱ��/s');
 ylabel('Y/m');
  %����X�ֲ���״ͼ
 subplot(2, 2, 4);
 hist(Y,10);
 title('Y����ֲ���״ͼ');
 xlabel('Y/m');
 ylabel('Ƶ��');