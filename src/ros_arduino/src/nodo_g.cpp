#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include <sstream>

std_msgs::String entrada1;
std_msgs::String entrada2;
std_msgs::String entrada3;
std_msgs::String entrada4;
  //Funciones callbacks para tomar los diferentes datos que llegan de diferentes nodos
void Callback_a(const std_msgs::String::ConstPtr& dato_recibido1)
{
  entrada1.data = dato_recibido1->data;
}

void Callback_b(const std_msgs::String::ConstPtr& dato_recibido2)
{
  entrada2.data = dato_recibido2->data;
}

void Callback_c(const std_msgs::String::ConstPtr& dato_recibido3)
{
  entrada3.data = dato_recibido3->data;
}

void Callback_d(const std_msgs::String::ConstPtr& dato_recibido4)
{
  entrada4.data = dato_recibido4->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nodo_g");

  ros::NodeHandle n;

  //subscriber a los 3 nodos
  ros::Subscriber sub1_String = n.subscribe("topic_d_g", 10, Callback_a);
  ros::Subscriber sub2_String = n.subscribe("topic_e_g", 10, Callback_b);
  ros::Subscriber sub3_String = n.subscribe("topic_f_g", 10, Callback_c);
  ros::Subscriber sub4_String = n.subscribe("topic_n2_g", 10, Callback_d);

  //Se define la frecuencia
  ros::Rate loop_rate(0.5);

  //variables de salida
  std_msgs::String salida1;
  std_msgs::String salida2;
  std_msgs::String salida3;
  std_msgs::String salida4;
  salida1.data = "";
  salida2.data = "";
  salida3.data = "";
  salida4.data = "";

  while (ros::ok())
  {

  //entrada1 es el sensor de temperatura
  if (entrada1.data == "frio")
  {
   salida1.data = "cold";
  }
  else if (entrada1.data == "tibio")
  {
   salida1.data = "warm";
  }
  else
  {
   salida1.data = "hot";
  }

  //entrada2 es el sensor de velocidad (tacometro)
  if (entrada2.data == "lento")
  {
   salida2.data = "slow";
  }
  else if (entrada2.data == "normal")
  {
   salida2.data = "normal";
  }
  else
  {
   salida2.data = "speed";
  }

  //entrada3 es el estado
  if (entrada3.data == "apagado")
  {
   salida3.data = "off";
  }
  else 
  {
   salida3.data = "on";
  }

  //entrada4 nueva
  if (entrada4.data == "hola")
  {
   salida4.data = "hello";
  }
  else if (entrada4.data == "mundo")
  {
   salida4.data = "world";
  }
  else if (entrada4.data == "ros")
  {
   salida4.data = "ROS";
  }
  else
  {
   salida4.data = "JOEL_LAURA_WILLIAM";
  }

    //Debug para imprimir cada variable
    std::stringstream ss1;
    ss1 <<  "Tempereture: [" <<salida1<< "]";
    std_msgs::String msg1;
    msg1.data = ss1.str();
    ROS_INFO("%s",msg1.data.c_str());

    std::stringstream ss2;
    ss2 <<  "Speed: [" <<salida2<< "]";
    std_msgs::String msg2;
    msg2.data = ss2.str();
    ROS_INFO("%s",msg2.data.c_str());

    std::stringstream ss3;
    ss3 <<  "State: [" <<salida3<< "]";
    std_msgs::String msg3;
    msg3.data = ss3.str();
    ROS_INFO("%s",msg3.data.c_str());

    std::stringstream ss4;
    ss4 <<  "random: [" <<salida4<< "]";
    std_msgs::String msg4;
    msg4.data = ss4.str();
    ROS_INFO("%s",msg4.data.c_str());
    ROS_INFO("=======================");

    //Rate
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
