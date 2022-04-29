/***********************************************************************
     Public LuoXio (luoxiaoxiao@stu.xmu.edu.cn). 2021 Mar 4
                    该例程用于电机的PID控制和卡尔曼滤波
***********************************************************************/

/* ##################  完善中  勿动  ###################### */
#include <iostream>
using namespace std;



//--------------------------------------Kalman Filter Start---------------------------------------//

// ********************* 卡尔曼变量的定义  结构体 "Kalman_Args" ****************************

struct Kalman_Args{
   float K1 = 0.02;
   float position = 0;          // ***位置预测*** 初始值 0
   float speed = 0;             // ***速度预测*** 初始值 0
   float Q_position = 1;    // 过程噪声协方差
   float Q_inputspeed = 1;       //  过程噪声协方差
   float R_position = 0.001;      // 测量噪声协方差
   float dt = 1;             // 采样时间
   char C_0 = 1;
   float Q_bias = 0;            // 漂移值 先 默认为 0 
   float position_err = 0;
   float PCt_0 = 0;
   float PCt_1 = 0;
   float E = 0;
   float K_0 = 0;
   float K_1 = 0;
   float t_0 = 0;
   float t_1 = 0;
   float Pdot[4] = {0,0,0,0};
   float PP[2][2] = {{1, 0},{0, 1}};
};

struct Kalman_Args kalman_args;

/************************************************************************************
   函数功能： 二维卡尔曼滤波实现（速度预测/位置预测）
   入口参数： 1：手柄速度输出（speed） 2：电机编码器输出（position）
   返回值： 无
**************************************************************************************/
void Kalman_Filter(float encoder_position, float input_speed){
    kalman_args.position += (input_speed - kalman_args.Q_bias) * kalman_args.dt; //  先验估计

    kalman_args.Pdot[0] =  kalman_args.Q_position -  kalman_args.PP[0][1] -  kalman_args.PP[1][0];
    kalman_args.Pdot[1] = - kalman_args.PP[1][1];
    kalman_args.Pdot[2] = - kalman_args.PP[1][1];
    kalman_args.Pdot[3] =  kalman_args.Q_inputspeed;

    kalman_args.PP[0][0] +=  kalman_args.Pdot[0] *  kalman_args.dt;
    kalman_args.PP[0][1] +=  kalman_args.Pdot[1] *  kalman_args.dt;
    kalman_args.PP[1][0] +=  kalman_args.Pdot[2] *  kalman_args.dt;
    kalman_args.PP[1][1] +=  kalman_args.Pdot[3] *  kalman_args.dt;

    kalman_args.PCt_0 =  kalman_args.C_0 *  kalman_args.PP[0][0];
    kalman_args.PCt_1 =  kalman_args.C_0 *  kalman_args.PP[1][0];
    kalman_args.E =  kalman_args.R_position +  kalman_args.C_0 *  kalman_args.PP[0][0];
    kalman_args.K_0 =  kalman_args.PCt_0 /  kalman_args.E;
    kalman_args.K_1 =  kalman_args.PCt_1 /  kalman_args.E;
    kalman_args.t_0 =  kalman_args.PCt_0;
    kalman_args.t_1 =  kalman_args.C_0 *  kalman_args.PP[0][1];

    kalman_args.position_err = encoder_position -  kalman_args.position;               //    position error
    kalman_args.position += kalman_args.K_0 * kalman_args.position_err;             // 计算最优位置
    kalman_args.Q_bias +=  kalman_args.K_1 *  kalman_args.position_err;             // 计算最有零飘
    kalman_args.speed = input_speed -  kalman_args.Q_bias;                                    // 计算最优速度

    kalman_args.PP[0][0] -=  kalman_args.K_0 *  kalman_args.t_0; // 后验估计误差协方差
    kalman_args.PP[0][1] -=  kalman_args.K_0 *  kalman_args.t_1;
    kalman_args.PP[1][0] -=  kalman_args.K_1 *  kalman_args.t_0;
    kalman_args.PP[1][1] -=  kalman_args.K_1 *  kalman_args.t_1; 
} 


//--------------------------------------Kalman Filter End---------------------------------------//







//--------------------------------------PID Start---------------------------------------//

// ********************* 串级PID变量的定义  结构体 "Pid_Args" "Pid_Solve" ****************************
struct Pid_Args{
   int velocity_Kp = 10;
   int velocity_Ki = 10;
   int velocity_Kd = 10;

   int position_Kp = 10;
   int position_Ki = 10;
   int position_Kd = 10;
};

struct Pid_Solve{
   int velocity_compensate = 0;   

   int position_compensate = 0;
};

struct Pid_Args PID;
struct Pid_Solve Pid_Solve;

/*********************************************************************************************************
   函数功能： 使用卡尔曼滤波后的 速度/位置 信息 进行 PID 误差矫正
   入口参数： PD-->>>--1：位置误差 2：位置误差的微分（速度）/ PI-->>>---1：速度误差 2: 速度误差的积分（位移）
   返回值： 位置补偿/速度补偿    >>>>【串级PID】
**********************************************************************************************************/

int position_PD(float position_bais, float speed){   // 位置环 PID 整定
   int position_compensate;
   position_compensate = PID.position_Kp*position_bais + PID.position_Kd*speed;
   if(speed == 0){
 	position_compensate = 0;    // 位置补偿过程只能在手柄出现操作时启动
  }
   return position_compensate;
}

int velocity_PI(float velocity_bais, float position_bais){    // 速度环 PID 整定
   int velocity_compensate;
   velocity_compensate = PID.velocity_Kp*velocity_bais + PID.velocity_Ki*position_bais;
   return velocity_compensate;
}


int output_limit(int *arg_position, int *arg_velocity){       // PID 参数限制 
   int amplitude = 500;    // 暂定 500
   if(*arg_position<-amplitude) *arg_position = -amplitude;   
   if(*arg_position>amplitude) *arg_position  =  amplitude;   
   if(*arg_velocity<-amplitude) *arg_velocity = -amplitude;
   if(*arg_velocity>amplitude) *arg_velocity  =  amplitude;    
}

//--------------------------------------PID End---------------------------------------//




int main(int argc, char **argv){
   Pid_Solve.position_compensate = position_PD(100, 6);                // PID test
   Pid_Solve.velocity_compensate = velocity_PI(3, 300);
   output_limit(&Pid_Solve.position_compensate, &Pid_Solve.velocity_compensate);
   cout << "position solve result:" << Pid_Solve.position_compensate << endl;
   cout << "velocity solve result:" << Pid_Solve.velocity_compensate<< endl;
   for (int  i = 0; i < 20; i++)                                       // kalman_filter test   y = 0.5gt^2
   {
      int position_test = 5 * i * i;
      int speed_test = 10 * i;
      Kalman_Filter(position_test, speed_test);
      // cout << "position_predict: " << kalman_args.position << endl;
      cout << "speed_predict: " << kalman_args.speed << endl;
   }
}
