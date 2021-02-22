Standard_unit_distance=pi*0.945/90;%单位信号脉冲对应实际物理距离，以m为单位
t_unit=0.01;%速度改变频率，该值越小越能反映实际情况中的模拟变化
time=200;%时间跨度，以s为单位
t=time*1000/t_unit;%速度变更总次数
v1=1;%起始真实速度，km/h
v2=500;%终点真实速度，km/h
a_yunjiasu=(v2-v1)/(time*3.6);%标准单位m/s^-2,假设匀加速运动
v=zeros(1,t);%记录各个速度阶
v_start=zeros(1,t);%记录迭代公式里的初速度
m=zeros(1,t);%记录各个速度阶对应的脉冲上升沿间隔
L=zeros(1,t);%存储输入脉冲
L1=zeros(1,t);%记录各个输入脉冲之间的时间间隔
for n=1:1:t %产生各个速度阶
    v(n)=v1+(n-1)*(v2-v1)/(t-1);
    m(n)=1000*3.6*Standard_unit_distance/v(n);%毫秒为基本时间单位
end
v_start(1)=v1/3.6;
L_count=1;
for n=1:1:t %产生输入脉冲矩阵
    L(L_count)=L(L_count)+((sqrt(v_start(L_count)^2+2*a_yunjiasu*Standard_unit_distance)-v_start(L_count))/a_yunjiasu);
    %防滑器采集脉冲波形仿真，以匀加速运动为模型
    L1(L_count)=(sqrt(v_start(L_count)^2+2*a_yunjiasu*Standard_unit_distance)-v_start(L_count))*1000/a_yunjiasu;
    if L(L_count)>(time)
        break;
    end %time的限时
    v_start(L_count+1)=v(1+floor(1000*L(L_count)/t_unit))/3.6;
    L(L_count+1)=L(L_count);
    L_count=L_count+1;
end
L_length=0;
for n=1:1:t
    if L(n)==0
        L_length=n-1;
        break;
    end
end
input=[0,1000*L(1,1:(L_length-1))];%毫秒为基本时间单位
input_voltage=zeros(1,L_length);
for n=1:1:L_length
    input_voltage(n)=v(1+floor(input(n)/t_unit));
end
%%输入脉冲仿真

All_Pulse_Num=L_length; %原信号脉冲上升沿数
Origin_Signal=zeros(1,All_Pulse_Num); %以毫秒为基本时间单位     %length(1:0.25:550.75)
Origin_Signal(1,:)=input; %四路通道源信号脉冲上升沿时间节点
Velocity=zeros(1,All_Pulse_Num); %输出速度信息
Circles=1; %起始采样周期数
Circus_Change=zeros(1,All_Pulse_Num);
Enable=0; %整个测速系统的使能开关
Circles_remaining=1;%实时记录当前所剩采样周期数
Standard_Signal_Fre=0.75e3;%标准精密脉冲频率
Standard_Signal_Per=1/Standard_Signal_Fre;%标准精密脉冲周期
Rtime1=40;%Timer1
Rtime2=0;%Timer2
Current_Sampling_begin=1;%记录当前采样开始的时间节点
Current_Sampling_internal=0;%记录当前采样的真实时间间隔
CCHL=[0,0];%标准脉冲计数（实际应用算法）
CCHL_Begin=0;%记录当前采样开始时的计数
CCHL_Count=0;%记录当前采样对应的标准脉冲的计数
Standard_Signal_Pattern=1+rand(1)>0.5;%用来决定标准脉冲计数计时的误差为+1或是-1
count_2=1;%单路通道
NUM=0;
for count_1=1:1:All_Pulse_Num 
    if count_1==1
        Enable=1;
    else
        CCHL(1)=mod(ceil((Origin_Signal(count_2,count_1)-Origin_Signal(count_2,1))*Standard_Signal_Fre),65536);
        CCHL(2)=mod(floor((Origin_Signal(count_2,count_1)-Origin_Signal(count_2,1))*Standard_Signal_Fre),65536);
        Circles_remaining=Circles_remaining-1;
        if Circles_remaining==0
            Current_Sampling_internal=Origin_Signal(count_2,count_1)-Origin_Signal(count_2,Current_Sampling_begin);
            Rtime2=floor(Current_Sampling_internal);
            Rectify_CCHL=Rtime2*Standard_Signal_Fre;
            CCHL_Count=CCHL(Standard_Signal_Pattern)-CCHL_Begin;
            while (CCHL_Count<Rectify_CCHL)
                CCHL_Count=CCHL_Count+65536;
            end
            CCHL_Begin=CCHL(Standard_Signal_Pattern);
            Current_Sampling_begin=count_1;
            Velocity(count_2,count_1)=1000*3.6*Circles*Standard_unit_distance/(CCHL_Count*Standard_Signal_Per);
            NUM=NUM+1;
            Rtime1=ceil(Rtime1-Current_Sampling_internal);
            if Rtime1<=0
                if Circles~=1
                    Circles=Circles/2;
                    Circus_Change(1,count_1)=-1;
                end
            end
            if Rtime1>=21
                Circles=Circles*2;
                Circus_Change(1,count_1)=1;
            end
            Rtime1=40;
            Circles_remaining=Circles;
        end
    end
end
%%应用测速算法

R1=zeros(6,NUM);
NUM_count=1;
for r=1:1:All_Pulse_Num
    if Velocity(count_2,r)~=0
        R1(1,NUM_count)=Velocity(count_2,r);
        R1(2,NUM_count)=Origin_Signal(1,r);
        R1(3,NUM_count)=input_voltage(1,r);
        R1(5,NUM_count)=input(1,r)/1000;
        R1(6,NUM_count)=Circus_Change(1,r);
        NUM_count=NUM_count+1;
    end
end
R1(4,:)=R1(1,:)-R1(3,:);
j=abs(R1(4,:)./R1(1,:))*100;
figure(1)
plot(R1(3,:),R1(4,:));
xlabel('真实速度（km/h）');
ylabel('误差（km/h）');
hold on
plot(R1(3,:),-5*R1(6,:)/100,'r');
hold off
legend('真实误差','采样周期加倍点');

figure(2)
plot(input/1000,input_voltage);
xlabel('时间轴（s）');
ylabel('真实速度（km/h）');
legend('真实速度');

figure(3)
plot(R1(3,:),j);
xlabel('真实速度（km/h）');
ylabel('误差（%）');
hold on
plot(R1(3,:),0.01*R1(6,:),'r');
hold off
legend('真实误差百分比','采样周期加倍点');

figure(4)
xlabel('时间轴（s）');
ylabel('算法速度（km/h）');
plot(R1(5,:),R1(1,:),'b');
xlabel('时间轴（s）');
ylabel('算法速度（km/h）');
legend('算法速度');
%%数据可视化表达