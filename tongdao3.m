All_Pulse_Num=2200; %ԭ�ź�������������
Standard_unit_distance=pi*0.945/90; %��λ�ź������Ӧʵ���������
Origin_Signal=zeros(4,All_Pulse_Num); %�Ժ���Ϊ����ʱ�䵥λ     %length(1:0.25:550.75)
Origin_Signal(1,:)=1:0.25:550.75; %��·ͨ��Դ�ź�����������ʱ��ڵ�
Origin_Signal(2,:)=1:0.25:550.75;
Origin_Signal(3,:)=1:0.25:550.75;
Origin_Signal(4,:)=1:0.25:550.75;
Flag=zeros(4,All_Pulse_Num);
Voltage=zeros(4,All_Pulse_Num); %����ٶ���Ϣ
Circles=1; %��ʼ����������
Enable=zeros(1,4); %��������ϵͳ��ʹ�ܿ���
Circles_remaining=1;%ʵʱ��¼��ǰ��ʣ����������
Standard_Signal_Fre=0.75e3;%��׼��������Ƶ��
Standard_Signal_Per=1/Standard_Signal_Fre;%��׼������������
Standard_Signal_Count=[0,0];%��׼��������������㷨��
Rtime1=40;%Timer1
Rtime2=0;%Timer2
Current_Sampling_begin=1;%��¼��ǰ������ʼ��ʱ��ڵ�
Current_Sampling_internal=0;%��¼��ǰ��������ʵʱ����
CCHL=zeros(4,2);%��׼���������ʵ��Ӧ���㷨��
CCHL_Begin=0;%��¼��ǰ������ʼʱ�ļ���
CCHL_Count=0;%��¼��ǰ������Ӧ�ı�׼����ļ���
Error_pro1=0.5;%����Ԥ������ϵͳ�п��ܳ��ֵĴ���
Standard_Signal_Pattern=1+rand(1)>0.5;%����������׼���������ʱ�����Ϊ+1����-1
Master=1;
Slave=[2,3,4];

for count_1=1:1:All_Pulse_Num %��ͨ���Ĳ����㷨
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
    
for count_2=2:1:4 %4·ͨ��������ÿһ·���Բ��ò����㷨��ʹ�ö����ļĴ�������·֮�������ӹ�ϵ
    %�¼��裺���Ĵ���֮��������ӹ�ϵ��1ͨ��Ϊ����������Ϊ��
    %3·��ͨ��������ͨ��ʱ���Ȳ�һ��,������һ�׼�ʱ�豸
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
    for count_3=1:1:All_Pulse_Num %ÿһ��·�Ĳ����㷨
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
            if Flag(1,count_3)==1%׼ȷ�Ĳ���ǰ����������ռʱ��
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