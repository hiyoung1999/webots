/*
 * File:          pid.c
 * Date:          2021.08.10
 * Description:   PD Controller test
 * Author:        HiYoung
 * Modifications: 2021.08.10
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#define TIME_STEP 64

int main(int argc, char **argv) 
{
  wb_robot_init();

  /*��������*/
  WbDeviceTag motor = wb_robot_get_device("linear motor1");//ֱ�ߵ��
  WbDeviceTag sensor = wb_robot_get_device("position sensor1");//λ�ô�����
  wb_position_sensor_enable(sensor, TIME_STEP);//λ�ô�����ʹ��

  double last_pos = 0;
  /*�ٽ�����*/
  //double kp = 100;
  //double kd = 301;
  /*Ƿ����*/
  double kp = 100;
  double kd = 200;
  /*������*/
  /*double kp = 100;
  double kd = 400;*/
  while (wb_robot_step(TIME_STEP) != -1) 
  {
      double pos = wb_position_sensor_get_value(sensor);
      double force = kp * (0 - pos) + kd *(last_pos - pos);
      /*     F     = Kp * x           + Kd * dx            */
      last_pos = pos;
      if (force > 500)
          force = 500;
      else if (force < -500)
          force = -500;
      wb_motor_set_force(motor, force);
      printf("%f\r\n", pos);
  };
  wb_robot_cleanup();
  return 0;
}
