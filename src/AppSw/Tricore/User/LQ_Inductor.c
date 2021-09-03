/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 ��ƽ    ̨�������������ܿƼ�TC264DA���İ�
 ����    д��chiusir
 ��E-mail��chiusir@163.com
 ������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
 �������¡�2020��10��28��
 �������Ϣ�ο����е�ַ��
 ����    վ��http:// www.lqist.cn
 ���Ա����̡�http:// longqiu.taobao.com
 ------------------------------------------------
 ��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
 ��Target �� TC264DA/TC264D
 ��Crystal�� 20.000Mhz
 ��SYS PLL�� 200MHz
 ________________________________________________________________
 ����iLLD_1_0_1_11_0�ײ����,

 ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
 ����CIFΪTC264DA�����⣬�����Ĵ������TC264D
 ����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
 ������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
 =================================================================
 ����������Ƶ��ַ��https:// space.bilibili.com/95313236
 =================================================================
 ʹ��˵����
 ����ѧ��ʾ���������ڵ�����ֻ������ֳ���
 ������ԴΪ��
 ģ�飺����TC264DA���İ塢����ĸ�塢˫·ȫ�ŵ��������˫������512������TFT1.8��Ļ�����������·���ģ�飻
 ��ģ�����ֻ������־��ɣ�
 ��зֲ���
 ||----------��------------------------------------��--------------||
 ||---------��--------------------------------------��-------------||
 ||--------��----------------------------------------��------------||
 ||-------1----����2����� -------------�Ҳ��2����� -----1-----------||
 ||------��--------------------------------------------��----------||
 ||-----��----------------------------------------------��---------||
 ||----��------------------------------------------------��--------||
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <IfxCpu.h>
#include <LQ_ADC.h>
#include <LQ_CCU6.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <Main.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_MotorServo.h>
#include <LQ_GPIO_LED.h>
#include <LQ_Inductor.h>
#include <LQ_GPT12_ENC.h>
#include <LQ_PID.h>

//pid_param_t * LOCA_PID;

sint16 TempAngle = 0;        // ���ݵ��ƫ�������������ʱ���ֵ
sint16 LastAngle = 0;        // ����������ǰ����Чƫ�Ʒ���

//sint16 a=0,b=0;
sint16 Lleft1 = 0, Lleft2 = 0, Lright2 = 0, Lright1 = 0, Lleft3 = 0, Lright3 = 0;  // ���ƫ����
sint16 LnowADC[6];           // ��е�ǰADC��ֵ

sint16 ad_max[6] = {2500, 4040, 4040, 4040, 4040, 2500}; // �°��ӷŵ������궨���ֵ,�ᱻ����ˢ��//2500
sint16 ad_min[6] = {0, 0, 0, 0, 0, 0}; // �°��Ӿ���Ҫ�궨��Сֵ,�ᱻ����ˢ��//120

uint8 CircleNumber = 1;    // �뻷������0������Ĭ��1�� ;���ĸ���һ���ڱ���ǰ��������ʱ��֪����
uint8 TangentPoint = 1;    // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
uint8 EnterCircle = 1;     // �������  Ĭ�� 0���ɽ�����1���Խ���
uint8 OutCircle = 0;       // �������   Ĭ��0���ɳ�����1���Գ���
uint8 LeftRightCircle = 0; // ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�

uint8 Fork = 0;    // �ֲ��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
uint8 EnterFork = 1;     // �������  Ĭ�� 0���ɽ�����1���Խ���
uint8 OutFork = 1;       // �������   Ĭ��0���ɳ�����1���Գ���
uint8 LeftRightFork = 1; // ��໷�����Ҳ໷ Ĭ����

sint32 TangentPointpulse = 0; // ���������ʱ����1
sint32 EnterCirclePulse = 0;  // ���������ʱ����2
sint32 OutCirclePulse = 0;    // ���������ʱ����3
sint32 EnterCircleOKPulse = 0;// ���������ʱ����4
sint16 LowSpeed = 0;          // Բ��/ʮ�ֿڼ���

uint16 MagneticField = 0;     // �ų�ǿ��    magnetic field intensity,�ж�Բ���Ƿ����
uint16 ALL_MagneticField = 0;

sint16 OffsetDelta = 0;
sint32 My_Pulse=0;
sint32 po_Pulse;
sint32 Out_Pulse=0;

sint32 Start=1;
sint32 Start_pulse=0;
sint32 End_pulse=0;
sint32 Back_pulse=0;

sint16 absolute(sint16 a,sint16 b);

/*************************************************************************
 *  �������ƣ�void InductorInit (void)
 *  ����˵�����ĸ����ADC��ʼ��������
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void InductorInit (void)
{
    ADC_InitConfig(ADC0, 100000); // ��ʼ��,������100kHz
    ADC_InitConfig(ADC1, 100000); // ��ʼ��
    ADC_InitConfig(ADC2, 100000); // ��ʼ��
    ADC_InitConfig(ADC3, 100000); // ��ʼ��

    ADC_InitConfig(ADC4, 100000); // ��ʼ��
    ADC_InitConfig(ADC5, 100000); // ��ʼ��
}
/*************************************************************************
 *  �������ƣ�void InductorNormal(void)
 *  ����˵�����ɼ���е�ѹ����һ����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��   ע��Ҫ�ȱ궨�˷ŵķŴ����������ĸ�һ�»��߽ӽ�
 *************************************************************************/

void My_Normal(void){
    LnowADC[0] = ADC_Read(ADC0);  // ����1����У��������н�Լ30�ȣ��ɼ�������е�ADֵ
    LnowADC[1] = ADC_Read(ADC1);  // ����2����У���ֱ������

    LnowADC[2] = ADC_Read(ADC4);  // �Ҳ��3����У���ֱ������
    LnowADC[3] = ADC_Read(ADC5);  // �Ҳ��3����У��������н�Լ30��

    LnowADC[4] = ADC_Read(ADC2);  // �Ҳ��2����У���ֱ������
    LnowADC[5] = ADC_Read(ADC3);  // �Ҳ��1����У��������н�Լ30��



    BatVolt       = ADC_Read(ADC7);  // ˢ�µ�ص�ѹ

    LnowADC[0]=(LnowADC[0] <10? 10:LnowADC[0]);  //�ĸ����ֵ�޷�
    LnowADC[1]=(LnowADC[1] <10? 10:LnowADC[1]);

    LnowADC[2]=(LnowADC[2] <10? 10:LnowADC[2]);
    LnowADC[3]=(LnowADC[3] <10? 10:LnowADC[3]);

    LnowADC[4]=(LnowADC[4] <10? 10:LnowADC[4]);
    LnowADC[5]=(LnowADC[5] <10? 10:LnowADC[5]);


    Lleft1 = (LnowADC[0] - ad_min[0]) * 100 / (ad_max[0] - ad_min[0]);     // ��ƫ������һ����0--100����
    Lleft2 = (LnowADC[1] - ad_min[1]) * 100 / (ad_max[1] - ad_min[1]);

    Lleft3 = (LnowADC[2] - ad_min[2]) * 100 / (ad_max[2] - ad_min[2]);
    Lright3 = (LnowADC[3] - ad_min[3]) * 100 / (ad_max[3] - ad_min[3]);

    Lright2 = (LnowADC[4] - ad_min[4]) * 100 / (ad_max[4] - ad_min[4]);
    Lright1 = (LnowADC[5] - ad_min[5]) * 100 / (ad_max[5] - ad_min[5]);

    MagneticField = Lleft1 + Lleft2 + Lright2 + Lright1;// �ų�����ǿ��
}
void InductorNormal (void)
{
    LnowADC[0] = ADC_Read(ADC0);  // ����1����У��������н�Լ30�ȣ��ɼ�������е�ADֵ
    LnowADC[1] = ADC_Read(ADC1);  // ����2����У���ֱ������

    LnowADC[2] = ADC_Read(ADC4);  // �Ҳ��3����У���ֱ������
    LnowADC[3] = ADC_Read(ADC5);  // �Ҳ��3����У��������н�Լ30��

    LnowADC[4] = ADC_Read(ADC2);  // �Ҳ��2����У���ֱ������
    LnowADC[5] = ADC_Read(ADC3);  // �Ҳ��1����У��������н�Լ30��
    BatVolt       = ADC_Read(ADC7);  // ˢ�µ�ص�ѹ

    LnowADC[0]=(LnowADC[0] <10? 10:LnowADC[0]);  //�ĸ����ֵ�޷�
    LnowADC[1]=(LnowADC[1] <10? 10:LnowADC[1]);

    LnowADC[2]=(LnowADC[2] <10? 10:LnowADC[2]);
    LnowADC[3]=(LnowADC[3] <10? 10:LnowADC[3]);

    LnowADC[4]=(LnowADC[4] <10? 10:LnowADC[4]);
    LnowADC[5]=(LnowADC[5] <10? 10:LnowADC[5]);


    if (LnowADC[0] < ad_min[0])
        ad_min[0] = LnowADC[0];     // ˢ����Сֵ
    else if (LnowADC[0] > ad_max[0])
        ad_max[0] = LnowADC[0];     // ˢ�����ֵ
    if (LnowADC[1] < ad_min[1])
        ad_min[1] = LnowADC[1];
    else if (LnowADC[1] > ad_max[1])
        ad_max[1] = LnowADC[1];

    if (LnowADC[2] < ad_min[2])
        ad_min[2] = LnowADC[2];     // ˢ����Сֵ
    else if (LnowADC[2] > ad_max[2])
        ad_max[2] = LnowADC[2];     // ˢ�����ֵ
    if (LnowADC[3] < ad_min[3])
        ad_min[3] = LnowADC[3];
    else if (LnowADC[3] > ad_max[3])
        ad_max[3] = LnowADC[3];

    if (LnowADC[4] < ad_min[4])
        ad_min[4] = LnowADC[4];
    else if (LnowADC[4] > ad_max[4])
        ad_max[4] = LnowADC[4];
    if (LnowADC[5] < ad_min[5])
        ad_min[5] = LnowADC[5];
    else if (LnowADC[5] > ad_max[5])
        ad_max[5] = LnowADC[5];

    Lleft1 = (LnowADC[0] - ad_min[0]) * 100 / (ad_max[0] - ad_min[0]);     // ��ƫ������һ����0--100����
    Lleft2 = (LnowADC[1] - ad_min[1]) * 100 / (ad_max[1] - ad_min[1]);

    Lleft3 = (LnowADC[2] - ad_min[2]) * 100 / (ad_max[2] - ad_min[2]);
    Lright3 = (LnowADC[3] - ad_min[3]) * 100 / (ad_max[3] - ad_min[3]);

    Lright2 = (LnowADC[4] - ad_min[4]) * 100 / (ad_max[4] - ad_min[4]);
    Lright1 = (LnowADC[5] - ad_min[5]) * 100 / (ad_max[5] - ad_min[5]);

    MagneticField = Lleft1 + Lleft2 + Lright2 + Lright1;// �ų�����ǿ��

    ALL_MagneticField=Lleft1 + Lleft2 + Lright2 + Lright1+Lleft3 + Lleft3;


    //PidLocCtrl();
// if(MagneticField<50){
//    if(Lleft2 + Lright2<20 && MagneticField<50)      // ����ֵ��С��20����Ÿ�Զ������������ֱ����ͼ��� MagneticField>70˵��˫��
//        {
//            if (Lleft1 + Lleft2 > Lright2 + Lright1)
//            {
//                TempAngle = Servo_Left_Max;
//                LED_Ctrl(LEDALL,RVS);
//            }
//            else
//            {
//                TempAngle =Servo_Right_Min ;
//                LED_Ctrl(LEDALL,RVS);
//            }
//            LastAngle =TempAngle;// ������Ч����������ƫ�Ʒ���
//        }
//        else if ((Lleft2 > 13) && (Lright2 > 13))   // С���������������м�20
//        {
//            TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *10; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
//        }
//        else                                        //  С���������������������һ��һС����Ҫ�ϴ�̶ȿ���ת��
//        {
//            TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) * 20; //  һ�����ת����ƣ���ֵԽ��ת��Խ��9
//            LED_Ctrl(LED0,RVS);
//        }
//}

if(MagneticField){
    if(Lleft2 + Lright2<40 )      // <50����ֵ��С��20����Ÿ�Զ������������ֱ����ͼ��� MagneticField>70˵��˫��&&(Lleft2 - Lright2<-5||Lleft2 - Lright2>5)
    {



        if (Lleft1 + Lleft2 > Lright2 + Lright1)//&&(Lleft2 - Lright2>5&&Lleft2 - Lright2<-5&&Lleft1+Lleft2>20)
        {

            TempAngle = Servo_Left_Max;
            LED_Ctrl(LEDALL,RVS);

           // Fork=66;
        }
        if (Lleft1 + Lleft2 < Lright2 + Lright1)//&&(Lleft2 - Lright2>5&&Lleft2 - Lright2<-5&&Lleft1+Lleft2>20)
        {

            TempAngle =Servo_Right_Min ;
            LED_Ctrl(LEDALL,RVS);
           // Fork=66;

        }

        LastAngle =TempAngle;// ������Ч����������ƫ�Ʒ���
    }
    else if ((Lleft2 > 10) && (Lright2 > 10))   // С���������������м�20
    {
        TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *7; // 7 ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
    }
    else                                        //  С���������������������һ��һС����Ҫ�ϴ�̶ȿ���ת��
    {
        TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) * 11; // 12 һ�����ת����ƣ���ֵԽ��ת��Խ��9
        LED_Ctrl(LED0,RVS);
    }

//    //ʮ������
//    if(ALL_MagneticField>170&&ALL_MagneticField<300)
//    {
//
//
//        if (absolute(Lleft2,Lright2)>40)//&&(Lleft2 - Lright2>5&&Lleft2 - Lright2<-5&&Lleft1+Lleft2>20)
//    {
//
//        TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *2;
//
//
//    }
//        TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *6; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
//    }

}


}

void Last(void){
    if(Start==1&&CircleNumber!=0){
        Start_pulse=RAllPulse;

        while (RAllPulse <  Start_pulse+1000){
                    MotorDuty1 = 3050-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 0.5  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������2450
                    MotorCtrl(MotorDuty1);
                }
        Start_pulse=RAllPulse;

        while (RAllPulse <  Start_pulse+3700){
                   ServoCtrl(1050);
                   MotorDuty1 = 3050-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 0.5  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������2450
                   MotorCtrl(MotorDuty1);

               }

        Lleft3=9;//�ɼ�������Ϊ����ǰ�� ���ж�Ϊ����
        Lright3=19;

        Start=0;
//        My_Pulse=RAllPulse;
//        while ((RAllPulse < My_Pulse + 1000))   //
//                   {
//                       My_Normal();
//                       if ((Lleft2 > 15) && (Lright2 > 15))   // С���������������м�20
//                          {
//                              TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *7; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
//                              ServoCtrl(TempAngle);
//
//                          }
//
//                   }
    }
    //My_Pulse=0;

    if(Game_Over==1&&CircleNumber==0)
        {
        Back_pulse=RAllPulse;
        MotorCtrl(-1000);
        delayms(1000);

        while(RAllPulse>Back_pulse+3200){
        InductorNormal();
        MotorDuty1 = -2850-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 2  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������1650
        MotorCtrl(MotorDuty1);
        }

        while(RAllPulse<Back_pulse+3200){
        InductorNormal();
        MotorDuty1 = 2850-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 2  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������1650
        MotorCtrl(MotorDuty1);
        }


        End_pulse=RAllPulse;
        while (RAllPulse > End_pulse-3000){

            ServoCtrl(1050);
            MotorDuty1 = -2850-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 2  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������1650
            MotorCtrl(MotorDuty1);
        }

        End_pulse=RAllPulse;
        while (RAllPulse > End_pulse-300){

            ServoCtrl(1250);
            MotorDuty1 = -2850-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 2  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������1650
            MotorCtrl(MotorDuty1);
        }

        while(1){
        MotorCtrl(0);
        }
        }


}

sint16 temp;
sint16 absolute(sint16 a,sint16 b){
    if(a-b>=0)
        temp=a-b;
    if(a-b<0)
        temp=b-a;
    return temp;
}

void po (void)
{
    po_Pulse=RAllPulse;

    // �����µ��ж�
    if (MagneticField >20&&(ECPULSE2<0||ECPULSE2==0)&&Game_Over!=1)
    {
        LED_Ctrl(LED0,RVS);
       while (RAllPulse <  po_Pulse+12000){

        InductorNormal();
        //���ƶ��
        if((Lleft2 > 13) && (Lright2 > 13))      // ����ֵ��С��20����Ÿ�Զ������������ֱ����ͼ��� MagneticField>70˵��˫��
             {
                 TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *20; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
                 //ServoCtrl(TempAngle);
             }
             else                                        //  С���������������������һ��һС����Ҫ�ϴ�̶ȿ���ת��
             {
                 TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) * 30; //  һ�����ת����ƣ���ֵԽ��ת��Խ��9
                 LED_Ctrl(LED0,RVS);
                 //ServoCtrl(TempAngle);
             }
         ServoCtrl(TempAngle);
         //���Ƶ��

         MotorDuty1 = 7500-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 30  - LowSpeed;//200    35

         MotorCtrl(MotorDuty1);   // ����˫�������

        }
//       po_Pulse=RAllPulse;
//       while (RAllPulse <  po_Pulse+1500){
//
//               InductorNormal();
//               //���ƶ��
//               if((Lleft2 > 13) && (Lright2 > 13))      // ����ֵ��С��20����Ÿ�Զ������������ֱ����ͼ��� MagneticField>70˵��˫��
//                    {
//                        TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *13; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
//                        //ServoCtrl(TempAngle);
//                    }
//                    else                                        //  С���������������������һ��һС����Ҫ�ϴ�̶ȿ���ת��
//                    {
//                        TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) * 15; //  һ�����ת����ƣ���ֵԽ��ת��Խ��9
//                        LED_Ctrl(LED0,RVS);
//                        //ServoCtrl(TempAngle);
//                    }
//                ServoCtrl(TempAngle);
//                //���Ƶ��
//                MotorDuty1 = -2800-(BatVolt * 11 / 25-750)*Kbat  - LowSpeed;  // �в��ٿ��ƣ���ת
//
//
//                MotorCtrl(MotorDuty1);   // ����˫�������
//
//               }

         if (MagneticField < 2)     // �ж��Ƿ�������30
             {
                 MotorCtrl(0);        // �������ͣ��
                 delayms(200);
             }
    }

}

void ForkDetect (void)
{
    // ����Y�������ж�Lleft2<20 && Lright2<20&&(Lleft1+Lright1<15)&&LeftRightCircle==0&&((Lleft2-Lright2<5)||Lleft2-Lright2>5)
    if (ECPULSE2>300&&Lleft2<20 && Lright2<20&&LeftRightCircle==0&&(absolute(Lleft2,Lright2)<8)&&(absolute(Lleft3,Lright3)<8)) // MagneticField <50 &&
    {//350

        Fork=1;

        if(LeftRightFork == 1)
        {
                ServoCtrl(Servo_Left_Max);
                delayms(300);
                LeftRightFork = 2;
        }

        else if(LeftRightFork == 2){
                ServoCtrl(Servo_Right_Min);
                delayms(300);
                LeftRightFork = 1;

        }

        MotorCtrl(-3000);        // �������ͣ��
        delayms(200);


        while(Fork){

             MotorDuty1 = MotorDuty1 = MotorDuty1 = 7500-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 17  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������;  // ��Ϊ�����ת��//2300
             MotorCtrl(MotorDuty1);
             //My_Normal();
             InductorNormal();

             if(Lleft3 + Lright3+Lleft2 + Lright2<60)      // ����ֵ��С��20����Ÿ�Զ������������ֱ����ͼ��� MagneticField>70˵��˫��
                 {
//                     if(Lleft2<0){
//                         Lleft2=0;
//                     }
//                     if(Lright2<0){
//                         Lright2=0;
//                     }
                     if  (Lleft3+Lleft2 > Lright3+Lright2)
                     {
                         TempAngle = Servo_Left_Max;
                         //ServoCtrl(TempAngle);
                         LED_Ctrl(LEDALL,RVS);
                     }
                     else
                     {
                         TempAngle =Servo_Right_Min ;
                         //ServoCtrl(TempAngle);
                         LED_Ctrl(LEDALL,RVS);
                     }
                     LastAngle =TempAngle;// ������Ч����������ƫ�Ʒ���
                 }
                 else if ((Lleft3+Lleft2 > 75) && (Lright3+Lright2 > 75))   // С���������������м�20
                 {
                     TempAngle = Servo_Center_Mid + (Lleft3+Lleft2 - Lright2-Lright3) *6; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��13
                     //ServoCtrl(TempAngle);
                 }
                 else                                        //  С���������������������һ��һС����Ҫ�ϴ�̶ȿ���ת��
                 {
                     TempAngle = Servo_Center_Mid + (Lleft3+Lleft2 - Lright2-Lright3) * 10; //  һ�����ת����ƣ���ֵԽ��ת��Խ��20
                     LED_Ctrl(LED0,RVS);
                     //ServoCtrl(TempAngle);
                 }

             ServoCtrl(TempAngle);


             if (MagneticField < 2)     // �ж��Ƿ�������30
                 {
                     MotorCtrl(0);        // �������ͣ��
                     delayms(200);
                 }
             //��
             if(MagneticField>100){

                 Fork=0;


                 if(LeftRightFork == 2)
                 {
                       //LeftRightFork = 2;
                       //ServoCtrl(Servo_Left_Max);

                 }
                 if(LeftRightFork == 1)
                 {
                       //LeftRightFork = 1;
                       //ServoCtrl(Servo_Left_Max);

                 }
                 delayms(200);

                 break;
             }

        }



    }
 }

/*************************************************************************
 *  �������ƣ�void CircleDetect void
 *  ����˵����ʶ�𲢽���Բ���ĸ�����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void CircleDetect (void)
{
    if (CircleNumber) // �뻷������0������Ĭ��2��
    {
        // �����е������ж�
        if (MagneticField > 190&&LeftRightCircle==0&&(Lleft1 <20||Lright1<20)&&(Lleft2>50||Lright2>50)&&(RAllPulse-Out_Pulse>10000||Out_Pulse==0)) // ֱ�����룬��ֵ����̫�����׶�ʧ�е�200     140
        {
            if ((Lleft1 + Lleft2 > Lright2 + Lright1)&&(Lleft2 >45)&&(Lright2>45))     // ����뻷���������У�����Lleft1 + Lleft2 > Lright2 + Lright1
            {
                LeftRightCircle = 1;  // ��໷Ϊ1

            }
            else if(((Lleft1 + Lleft2 < Lright2 + Lright1)&&(Lleft2 >45)&&(Lright2>45)))// 55�Ҳ��뻷����
            {
                LeftRightCircle = 2;  // �Ҳ໷Ϊ2


            }
            //delayms(200);
            My_Pulse=RAllPulse;
            while ((RAllPulse <  My_Pulse+1000)||(!(MagneticField > 205&&MagneticField <215)))   //Ϊʲô�������䣨�е㷶Χ�� �����е����ǰ��3000���壬��Լ50cm��
                        {
                            My_Normal();
                            if ((Lleft2 > 15) && (Lright2 > 15))   // С���������������м�20
                               {
                                   TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *5; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
                                   ServoCtrl(TempAngle);
                               }


                        }

            TangentPointpulse = RAllPulse; // ��ȡ��ǰ������ֵ
            TangentPoint = 0;             // ��ֹ�ٴζ�ȡ��ǰ������ֵ


            while ((RAllPulse < TangentPointpulse + 3500) &&( TangentPoint == 0))   // �����е����ǰ��3000���壬��Լ50cm��
            {

                EnterCircle = 1;      // ͨ���е����򣬿����뻷
                My_Normal();
                if ((Lleft2 > 15) && (Lright2 > 15))   // С���������������м�20
                   {
                       TempAngle = Servo_Center_Mid + (Lleft2 - Lright2) *5; //  ����ƫ������ֵС���ȴ�ǣ���ֱֹ��ҡ��
                       ServoCtrl(TempAngle);

                   }

            }
        }


        // ��Լ1.2���������Ч������Ҫ����ʶ���е�
        if ((RAllPulse > TangentPointpulse + 8000)) // Լ1.2���������Ч
        {
            EnterCircle = 0;   // Լ1.2���������Ч
            TangentPoint = 1;  // ����ʶ���е�

        }

        if ((RAllPulse < Out_Pulse + 10000)) // ���������������
        {
            LeftRightCircle = 0;

        }


        if ((EnterCircle) ) // Լ1.2�����ٴη���ǿ�ų����뻷&& (MagneticField > 160)
        {
            LowSpeed = 500;    // ����
            // ������뻷����
            if (LeftRightCircle == 1)     // ����뻷���������У�����Lleft1 + Lleft2 > Lright2 + Lright1
            {
                LeftRightCircle = 1;  // ��໷Ϊ1
                EnterCircle = 0;      // �뻷���ֹ�ٴ��뻷

                EnterCirclePulse = RAllPulse;
                ServoCtrl(Servo_Left_Max);     // ���PWM�����ת������ǿ���
                while (RAllPulse < EnterCirclePulse + 2500)//2800
                {
                    delayms(1);       // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������

                }
            }
            else if(LeftRightCircle == 2)// 55�Ҳ��뻷����
            {
                LeftRightCircle = 2;  // �Ҳ໷Ϊ2
                EnterCircle = 0;  // �뻷���ֹ�ٴ��뻷
                EnterCirclePulse = RAllPulse;

                ServoCtrl(1050);        // ���PWM�����ת������ǿ���1150
                while (RAllPulse < EnterCirclePulse + 1500) // 1800�õ����Ҳ�ı�������ʵ���ߵľ����һ���
                {
                    delayms(1);   // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������

                }
            }
            EnterCircleOKPulse = RAllPulse; // ������
        }
        // ��������
        if ((LeftRightCircle > 0) && (RAllPulse > EnterCircleOKPulse + 3000))
        {
            EnterCircleOKPulse = 10000000; //��ֹ�ٴγ���ʹ��
            OutCircle = 1;  // ��������Գ���
        }
        if ((OutCircle) && (MagneticField > 180)) // �뻷��־Ϊ������뻷//140
        {
            LowSpeed = 400;        // ����
            // ����������
            if (LeftRightCircle == 1)   //����뻷
            {
                //OutCircle = 0;    // �뻷���ֹ�ٴ��뻷

                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Left_Max);     // ���PWM�����ת������ǿ���
                while (RAllPulse < OutCirclePulse + 1200)//2800
                {
                    delayms(1); // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
                OutCirclePulse = RAllPulse;
               // ServoCtrl(Servo_Center_Mid-Servo_Delta*2/3);     // ���PWM�����������
                while (RAllPulse < OutCirclePulse + 600)//700
                {
                    delayms(1);         // ���Ƕ�ǰ��600���壬Լ10cm������512�����������1��5790������
                }
                CircleNumber--;         // ������
                TangentPoint = 1;       // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
                EnterCircle = 0;        // �������  Ĭ�� 0���ɽ�����1���Խ���
                OutCircle = 0;          // �������   Ĭ��0���ɳ�����1���Գ���
                LeftRightCircle = 0;    // ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
                LowSpeed = 0;           // �ָ��ٶ�

                Out_Pulse=RAllPulse;

            }
            // �Ҳ��������
            else if (LeftRightCircle == 2)  //�ұ��뻷
            {
                //OutCircle = 0;     // �뻷���ֹ�ٴ��뻷
                OutCirclePulse = RAllPulse;
                ServoCtrl(Servo_Right_Min);     // ���PWM�����ת������ǿ���
                while (RAllPulse < OutCirclePulse + 1200)
                {
                    delayms(1);   // 2500���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
                }
                OutCirclePulse = RAllPulse;
              //  ServoCtrl(Servo_Center_Mid+Servo_Delta*2/3);     // ���PWM�����������
                while (RAllPulse < OutCirclePulse + 600)
                {
                    delayms(1);         // 1400���Ƕ�ǰ��600���壬Լ10cm������512�����������1��5790������
                }
                CircleNumber--;         // ������
                TangentPoint = 1;       // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
                EnterCircle = 0;        // �������  Ĭ�� 0���ɽ�����1���Խ���
                OutCircle = 0;          // �������   Ĭ��0���ɳ�����1���Գ���
                LeftRightCircle = 0;    // ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
                LowSpeed = 0;           // �ָ��ٶ�

//                while(1){
//                    LED_Ctrl(LEDALL,RVS);
//                    delayms(1000);
//                };

                Out_Pulse=RAllPulse;


            }
        }
    }
}

//void LOCA_Init()
//{
//    LOCA_PID ->kp = 1;
//    LOCA_PID ->ki = 0;
//    LOCA_PID ->kd = 0;
//}
/*************************************************************************
 *  �������ƣ�void TFT_Show_EleMag_Info(void)
 *  ����˵������ʾ����������Ϣ
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��11��18��
 *  ��    ע��
 *************************************************************************/
void TFT_Show_EleMag_Info(void)
{
    char txt[16] = "X:";

    sint16 mps = 0, dmm = 0;    // �ٶȣ�m/s,������ֵ
    sint16 pulse100 = 0;
    uint16 bat=0;

    dmm = (sint16) (RAllPulse * 100 / 579);           // ����512�����������1��5790�����壬��ֵ̫�󣬳���100
    pulse100 = (sint16) (RAllPulse / 100);
    sprintf(txt, "AP:%05d00", pulse100);              //
    TFTSPI_P8X16Str(3, 1, txt, u16RED, u16BLACK);     // ��ʾ����ƫ�����

    NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // ��ȡSTM0 ��ǰʱ�䣬�õ�����
    mps = (sint16) (dmm / (NowTime / 1000));          // �����ٶ�mm/s
    // ������Ϣ
    sprintf(txt, "%04d %04d %04d ", TempAngle, ECPULSE1, ECPULSE2);   // ��ʾ����Ƕ���ֵ�����ռ�ձ���ֵ����������ֵ
    TFTSPI_P8X16Str(1, 0, txt, u16WHITE, u16BLACK);      // �ַ�����ʾ
    //��ʾ����й�һ�����ƫ����  ��ǰ����е�ѹֵ ����п�������ʷ��Сֵ ����п�������ʷ���ֵ
    sprintf(txt, "0:%04d %04d %04d ", Lleft1, absolute(Lleft2,Lright2), ad_max[0]);
    TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);
    sprintf(txt, "1:%04d %04d %04d ", Lleft2, Lleft3, ad_max[1]);//LnowADC[1]
    TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);
    sprintf(txt, "2:%04d %04d %04d ", Lright2, Lright3, ad_max[4]);
    TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);
    sprintf(txt, "3:%04d %04d %04d ", Lright1, absolute(Lleft3,Lright3), ad_max[5]);
    TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

    sprintf(txt, "Ring num: %d ", CircleNumber);
    TFTSPI_P8X16Str(2, 6, txt, u16GREEN, u16BLACK);
    sprintf(txt, "M:%03d Q:%d J:%d ", MagneticField, ALL_MagneticField, EnterCircle);//MagneticField, OutCircle, EnterCircle
    TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);
    if (LeftRightCircle == 1)
        TFTSPI_P8X16Str(0, 8, "Left Ring ", u16WHITE, u16BLACK);
    else if (LeftRightCircle == 2)
        TFTSPI_P8X16Str(0, 8, "Right Ring", u16WHITE, u16BLACK);
    else if(Fork==1 &&LeftRightFork==1)
        TFTSPI_P8X16Str(0, 8, "No Ring  Fork=1 1", u16WHITE, u16BLACK);
    else if(Fork==1 &&LeftRightFork==2)
            TFTSPI_P8X16Str(0, 8, "No Ring  Fork=1 2", u16WHITE, u16BLACK);
    else if(Fork==0 &&LeftRightFork==1)
        TFTSPI_P8X16Str(0, 8, "No Ring  Fork=0 1", u16WHITE, u16BLACK);
    else if(Fork==0 &&LeftRightFork==2)
        TFTSPI_P8X16Str(0, 8, "No Ring  Fork=0 2", u16WHITE, u16BLACK);

    bat = BatVolt * 11 / 25;  // x/4095*3.3*100*5.7
    sprintf(txt, "B:%d.%02dV %d.%02dm/s", bat / 100, bat % 100, mps / 1000, (mps / 10) % 100);  // *3.3/4095*3
    TFTSPI_P8X16Str(0, 9, txt, u16PURPLE, u16BLACK);   // �ַ�����ʾ
}
/*************************************************************************
 *  �������ƣ�void ElectroMagneticCar(void)
 *  ����˵������ų�˫������ٿ���
 -->1.�����㷨���򵥵ķֶα��������㷨����ѧ��ʾ�����㷨��
 ---2.�����㷨��PID����Ӧ�ÿ����㷨����ѧ��ʾ�����㷨��
 ---3.�߶��㷨���ݶ��Ľ�����ȺЭͬ�����㷨��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע������2�����
 *************************************************************************/
void ElectroMagneticCar (void)
{
    sint16 bat=0;

    CircleNumber = 1;   // �뻷������0������Ĭ��1��
    TangentPoint = 1;   // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
    EnterCircle = 0;    // �������  Ĭ�� 0���ɽ�����1���Խ���
    OutCircle = 0;      // �������   Ĭ��0���ɳ�����1���Գ���
    LeftRightCircle = 0;// ��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
    LowSpeed = 0;       // �ٶȲ�



    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=0;             // CPU1�� 0ռ��/1�ͷ� TFT
    CircleNumber = SetCircleNum();  // ������Ҫ����Բ���ĸ�����

    // ��������Ҫ���ó���⣬�����ǹ̶�ִ�У�
    // �������Ҫ�ɻɹܺ��ⲿ�ж����ʵ��
    // ���������У��ɻɹ���ͨ��Բ������������������ܴ����Ŀ�����
   // OutInGarage(OUT_GARAGE,ReadOutInGarageMode()); // ���Գ��⣬��������������⣬��֮�Ҳ�����
    //OutInGarage(IN_GARAGE,ReadOutInGarageMode());  // �������

    TFTSPI_CLS(u16BLACK);            // ����
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=1;             // CPU1�� 0ռ��/1�ͷ� TFT

    RAllPulse = 0;                  // ȫ�ֱ����������������
    NowTime = STM_GetNowUs(STM0);   // ��ȡSTM0 ��ǰʱ��

    //Reed_Init();
    while (1)
    {
        InductorNormal();           // �ɼ���е�ѹ������һ����
        Last();
        ForkDetect();
        if (MagneticField > 160)    // ֱ�����룬��ֵ����̫�����׶�ʧ�е�220
        {
            LowSpeed = 1500;         // ����500 //֮ǰע����
        }
        else if (MagneticField < 160)
        {
            LowSpeed = 0; // �ָ��ٶ�
        }
//        if(MagneticField > 100)
//        {
//            LowSpeed = 500;
//        }
//        else if (MagneticField > 80)
//                {
//                    LowSpeed = 0; // �ָ��ٶ�
//                }
        CircleDetect();             // ʶ�𲢽���Բ���ĸ�����
        //po();
        //ForkDetect();

        ServoDuty = TempAngle;
        ServoCtrl(ServoDuty);       // ���PWM�����ת������ǿ���
        //
        OffsetDelta = (Lleft2 - Lright2);  // ֱ��ƫ��

        if (MagneticField > 70)//ֱ���ٶ�
               {

                    //MotorDuty1 = 3550-bat - ECPULSE2 * 4  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������2550 4
            MotorDuty1 = 7500-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 9  - LowSpeed;
               }

        else if(MagneticField > 90&&MagneticField<110)
                {
                    //MotorDuty1 = 3550-bat - ECPULSE2 * 4  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������2550 2
            MotorDuty1 = 7500-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 17  - LowSpeed;
                }

        else{
            MotorDuty1 = 7500-(BatVolt * 11 / 25-750)*Kbat - ECPULSE2 * 20  - LowSpeed;
        }
//        if(MagneticField < 40)//������ٶ�
//               {
//                      MotorDuty1 = 2350-bat - ECPULSE2 * 4  - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ����������
//               }
        bat=(BatVolt * 11 / 25-750)*Kbat;


       // MotorDuty2 = MtTargetDuty-bat - ECPULSE2 * Kencoder + OffsetDelta * Koffset - LowSpeed;  // �в��ٿ��ƣ���תƫ��Ϊ�����Ҳ����

        if (MagneticField < 5)     // �ж��Ƿ�������30
        {
            MotorCtrl(0);        // �������ͣ��
            delayms(200);
        }
        else
        {

            MotorCtrl(MotorDuty1);   // ����˫�������
            // MotorCtrl(MtTargetDuty-TempAngle*8/5, MtTargetDuty+TempAngle*8/5);// ���ֳ����޶��
        }

       /* if(Game_Over)
        {
            OutInGarage(IN_GARAGE, ReadOutInGarageMode());
        }*/
    } // WHILE(1)
} // MAIN()



//    while (1){
//        ServoCtrl(Servo_Left_Max);     // ���PWM�����ת������ǿ���
//        while (RAllPulse <  5000)
//        {
//            delayms(1); // ���Ƕ�ǰ��1200���壬Լ20cm������512�����������1��5790������
//        }
//        ServoCtrl(1450);
//    }
