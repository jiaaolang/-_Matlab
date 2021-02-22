Standard_unit_distance=pi*0.945/90;%��λ�ź������Ӧʵ��������룬��mΪ��λ
t_unit=0.01;%�ٶȸı�Ƶ�ʣ���ֵԽСԽ�ܷ�ӳʵ������е�ģ��仯
time=200;%ʱ���ȣ���sΪ��λ
t=time*1000/t_unit;%�ٶȱ���ܴ���
v1=1;%��ʼ��ʵ�ٶȣ�km/h
v2=500;%�յ���ʵ�ٶȣ�km/h
a_yunjiasu=(v2-v1)/(time*3.6);%��׼��λm/s^-2,�����ȼ����˶�
v=zeros(1,t);%��¼�����ٶȽ�
v_start=zeros(1,t);%��¼������ʽ��ĳ��ٶ�
m=zeros(1,t);%��¼�����ٶȽ׶�Ӧ�����������ؼ��
L=zeros(1,t);%�洢��������
L1=zeros(1,t);%��¼������������֮���ʱ����
for n=1:1:t %���������ٶȽ�
    v(n)=v1+(n-1)*(v2-v1)/(t-1);
    m(n)=1000*3.6*Standard_unit_distance/v(n);%����Ϊ����ʱ�䵥λ
end
v_start(1)=v1/3.6;
L_count=1;
for n=1:1:t %���������������
    L(L_count)=L(L_count)+((sqrt(v_start(L_count)^2+2*a_yunjiasu*Standard_unit_distance)-v_start(L_count))/a_yunjiasu);
    %�������ɼ����岨�η��棬���ȼ����˶�Ϊģ��
    L1(L_count)=(sqrt(v_start(L_count)^2+2*a_yunjiasu*Standard_unit_distance)-v_start(L_count))*1000/a_yunjiasu;
    if L(L_count)>(time)
        break;
    end %time����ʱ
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
input=[0,1000*L(1,1:(L_length-1))];%����Ϊ����ʱ�䵥λ
input_voltage=zeros(1,L_length);
for n=1:1:L_length
    input_voltage(n)=v(1+floor(input(n)/t_unit));
end
%%�����������

All_Pulse_Num=L_length; %ԭ�ź�������������
Origin_Signal=zeros(1,All_Pulse_Num); %�Ժ���Ϊ����ʱ�䵥λ     %length(1:0.25:550.75)
Origin_Signal(1,:)=input; %��·ͨ��Դ�ź�����������ʱ��ڵ�
Velocity=zeros(1,All_Pulse_Num); %����ٶ���Ϣ
Circles=1; %��ʼ����������
Circus_Change=zeros(1,All_Pulse_Num);
Enable=0; %��������ϵͳ��ʹ�ܿ���
Circles_remaining=1;%ʵʱ��¼��ǰ��ʣ����������
Standard_Signal_Fre=0.75e3;%��׼��������Ƶ��
Standard_Signal_Per=1/Standard_Signal_Fre;%��׼������������
Rtime1=40;%Timer1
Rtime2=0;%Timer2
Current_Sampling_begin=1;%��¼��ǰ������ʼ��ʱ��ڵ�
Current_Sampling_internal=0;%��¼��ǰ��������ʵʱ����
CCHL=[0,0];%��׼���������ʵ��Ӧ���㷨��
CCHL_Begin=0;%��¼��ǰ������ʼʱ�ļ���
CCHL_Count=0;%��¼��ǰ������Ӧ�ı�׼����ļ���
Standard_Signal_Pattern=1+rand(1)>0.5;%����������׼���������ʱ�����Ϊ+1����-1
count_2=1;%��·ͨ��
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
%%Ӧ�ò����㷨

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
xlabel('��ʵ�ٶȣ�km/h��');
ylabel('��km/h��');
hold on
plot(R1(3,:),-5*R1(6,:)/100,'r');
hold off
legend('��ʵ���','�������ڼӱ���');

figure(2)
plot(input/1000,input_voltage);
xlabel('ʱ���ᣨs��');
ylabel('��ʵ�ٶȣ�km/h��');
legend('��ʵ�ٶ�');

figure(3)
plot(R1(3,:),j);
xlabel('��ʵ�ٶȣ�km/h��');
ylabel('��%��');
hold on
plot(R1(3,:),0.01*R1(6,:),'r');
hold off
legend('��ʵ���ٷֱ�','�������ڼӱ���');

figure(4)
xlabel('ʱ���ᣨs��');
ylabel('�㷨�ٶȣ�km/h��');
plot(R1(5,:),R1(1,:),'b');
xlabel('ʱ���ᣨs��');
ylabel('�㷨�ٶȣ�km/h��');
legend('�㷨�ٶ�');
%%���ݿ��ӻ����