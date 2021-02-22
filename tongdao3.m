All_Pulse_Num=2200; %原信号脉冲上升沿数
Standard_unit_distance=pi*0.945/90; %单位信号脉冲对应实际物理距离
Origin_Signal=zeros(4,All_Pulse_Num); %以毫秒为基本时间单位     %length(1:0.25:550.75)
Origin_Signal(1,:)=1:0.25:550.75; %四路通道源信号脉冲上升沿时间节点
Origin_Signal(2,:)=1:0.25:550.75;
Origin_Signal(3,:)=1:0.25:550.75;
Origin_Signal(4,:)=1:0.25:550.75;
Flag=zeros(4,All_Pulse_Num);
Voltage=zeros(4,All_Pulse_Num); %输出速度信息
Circles=1; %起始采样周期数
Enable=zeros(1,4); %整个测速系统的使能开关
Circles_remaining=1;%实时记录当前所剩采样周期数
Standard_Signal_Fre=0.75e3;%标准精密脉冲频率
Standard_Signal_Per=1/Standard_Signal_Fre;%标准精密脉冲周期
Standard_Signal_Count=[0,0];%标准脉冲计数（理想算法）
Rtime1=40;%Timer1
Rtime2=0;%Timer2
Current_Sampling_begin=1;%记录当前采样开始的时间节点
Current_Sampling_internal=0;%记录当前采样的真实时间间隔
CCHL=zeros(4,2);%标准脉冲计数（实际应用算法）
CCHL_Begin=0;%记录当前采样开始时的计数
CCHL_Count=0;%记录当前采样对应的标准脉冲的计数
Error_pro1=0.5;%用来预估测速系统中可能出现的错误
Standard_Signal_Pattern=1+rand(1)>0.5;%用来决定标准脉冲计数计时的误差为+1或是-1
Master=1;
Slave=[2,3,4];

for count_1=1:1:All_Pulse_Num %主通道的测速算法
        if count_1==1
            Enable(1)=1;
            %Flag(count_1)=1;
        else
            CCHL(Master,1)=mod(ceil((Origin_Signal(Master,count_1)-Origin_Signal(Master,1))*Standard_Signal_Fre),65536);
            CCHL(Master,2)=mod(floor((Origin_Signal(Master,count_1)-Origin_Signal(Master,1))*Standard_Signal_Fre),65536);
            Circles_remaining=Circles_remaining-1;
            if Circles_remaining==0
                Flag(1,count_1)=1;
                Current_Sampling_internal=Origin_Signal(Master,count_1)-Origin_Signal(Master,Current_Sampling_begin);
                Standard_Signal_Count(1)=floor(Current_Sampling_internal*Standard_Signal_Fre);
                Standard_Signal_Count(2)=ceil(Current_Sampling_internal*Standard_Signal_Fre);
                Rtime2=floor(Current_Sampling_internal);
                Rectify_CCHL=Rtime2*Standard_Signal_Fre;
                CCHL_Count=CCHL(Master,Standard_Signal_Pattern)-CCHL_Begin;
                while (CCHL_Count<Rectify_CCHL)
                    CCHL_Count=CCHL_Count+65536;
                end
                Flag(2,count_1)=CCHL_Count;
                Flag(3,count_1)=Current_Sampling_internal;
                %Flag(4,count_1)=Origin_Signal(Master,count_1);
                CCHL_Begin=CCHL(Master,Standard_Signal_Pattern);
                Current_Sampling_begin=count_1;
                Voltage(Master,count_1)=1000*Circles*Standard_unit_distance/(CCHL_Count*Standard_Signal_Per);
                Rtime1=Rtime1-Current_Sampling_internal;
                if Rtime1<=0
                    if Circles~=1
                        Circles=Circles/2;
                    end
                end
                if Rtime1>=21
                    Circles=Circles*2;
                end
                Rtime1=40;
                Circles_remaining=Circles;
            end
        end
end
    
for count_2=2:1:4 %4路通道，假设每一路各自采用测速算法，使用独立的寄存器，各路之间无主从关系
    %新假设：各寄存器之间存在主从关系，1通道为主机，其余为从
    %3路从通道，所有通道时间跨度不一致,各自用一套计时设备
    Circles=0;
    %Circles_remaining=1;
    Standard_Signal_Count=[0,0];
    %Rtime1=40;
    Rtime2=0;
    Current_Sampling_begin=1;
    Current_Sampling_internal=0;
    CCHL_Begin=0;
    CCHL_Count=0;
    Standard_Signal_Pattern=1+rand(1)>0.5;
    Slave_Enable=0;
    for count_3=1:1:All_Pulse_Num %每一从路的测速算法
        if count_3==1
            Enable(count_2)=1;
        else
            Circles=Circles+1;
            if Slave_Enable==1
                CCHL(count_2,1)=mod(ceil((Origin_Signal(count_2,count_3)-Origin_Signal(count_2,1))*Standard_Signal_Fre),65536);
                CCHL(count_2,2)=mod(floor((Origin_Signal(count_2,count_3)-Origin_Signal(count_2,1))*Standard_Signal_Fre),65536);
                Current_Sampling_internal=Origin_Signal(count_2,count_3)-Origin_Signal(count_2,Current_Sampling_begin);
                Standard_Signal_Count(1)=floor(Current_Sampling_internal*Standard_Signal_Fre);
                Standard_Signal_Count(2)=ceil(Current_Sampling_internal*Standard_Signal_Fre);
                Rtime2=floor(Current_Sampling_internal);
                Rectify_CCHL=Rtime2*Standard_Signal_Fre;
                CCHL_Count=CCHL(count_2,Standard_Signal_Pattern)-CCHL_Begin;
                while (CCHL_Count<Rectify_CCHL)
                    CCHL_Count=CCHL_Count+65536;
                end
                CCHL_Begin=CCHL(count_2,Standard_Signal_Pattern);
                Current_Sampling_begin=count_3;
                Voltage(count_2,count_3-1)=1000*Circles*Standard_unit_distance/(CCHL_Count*Standard_Signal_Per);
                Circles=0;
                Slave_Enable=0;
            end
            if Flag(1,count_3)==1%准确的捕获当前采样过程所占时间
                if Origin_Signal(count_2,count_3)>=Origin_Signal(Master,count_3)
                    CCHL(count_2,1)=mod(ceil((Origin_Signal(count_2,count_3)-Origin_Signal(count_2,1))*Standard_Signal_Fre),65536);
                    CCHL(count_2,2)=mod(floor((Origin_Signal(count_2,count_3)-Origin_Signal(count_2,1))*Standard_Signal_Fre),65536);
                    Current_Sampling_internal=Origin_Signal(count_2,count_3)-Origin_Signal(count_2,Current_Sampling_begin);
                    Standard_Signal_Count(1)=floor(Current_Sampling_internal*Standard_Signal_Fre);
                    Standard_Signal_Count(2)=ceil(Current_Sampling_internal*Standard_Signal_Fre);
                    Rtime2=floor(Current_Sampling_internal);
                    Rectify_CCHL=Rtime2*Standard_Signal_Fre;
                    CCHL_Count=CCHL(count_2,Standard_Signal_Pattern)-CCHL_Begin;
                    while (CCHL_Count<Rectify_CCHL)
                        CCHL_Count=CCHL_Count+65536;
                    end
                    CCHL_Begin=CCHL(count_2,Standard_Signal_Pattern);
                    Current_Sampling_begin=count_3;
                    Voltage(count_2,count_3)=1000*Circles*Standard_unit_distance/(CCHL_Count*Standard_Signal_Per);
                    Circles=0;
                else
                    Slave_Enable=1;
                end
            end
        end
    end
end