#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "std_msgs/Int8.h" 

//Funcion principal
int main(int argc, char **argv)
{ 
  //Se inicializa el primer nodo "nodo_p"
  ros::init(argc, argv, "nodo_p");
  ros::NodeHandle n;
  
  //Se crean los publisher
  ros::Publisher pub1_Int32 =   n.advertise<std_msgs::Int32>("topic_p_a", 10);
  ros::Publisher pub2_Float64 = n.advertise<std_msgs::Float64>("topic_p_b", 10);
  ros::Publisher pub3_Bool =    n.advertise<std_msgs::Bool>("topic_p_c", 10);

  ros::Publisher pub4_Int8 =    n.advertise<std_msgs::Int8>("topic_p_n1", 10);

  //Se define la frecuencia
  ros::Rate loop_rate(30);

  //variables emuladas
  //Sensor de temperatura
  std_msgs::Int32 sensor1;
  sensor1.data = 0;
  //Sensor de velocidad
  std_msgs::Float64 sensor2;
  sensor2.data  = 0.0;
  //Sensor de estado
  std_msgs::Bool sensor3;
  sensor3.data = false;

  //Variable Int8 
  std_msgs::Int8 sensor4;
  sensor4.data = 0;

  //variable para la emulacion de la variable 
  int count_bool = 0;
 
  while (ros::ok())
  {
    //parte del arduino
    sensor1.data++;
    if (sensor1.data > 1024)
    {
      sensor1.data = 0;
    }

    sensor2.data = sensor2.data + 0.1;
    if (sensor2.data > 10)
    {
      sensor2.data = 0;
    }

    count_bool++;
    if (count_bool > 90)
    {
     count_bool = 0;
    }
    if (count_bool < 45)
    {
     sensor3.data = true;
    }
    else 
    {
     sensor3.data = false;
    }


    sensor4.data = sensor4.data + 1;
    if (sensor4.data > 64)
    {
      sensor4.data = 0;
    }

   //Se imprime en pantalla
    ROS_INFO("SENSOR#1_INT32: [%d]",sensor1.data);
    ROS_INFO("SENSOR#2_FLOAT64: [%f]",sensor2.data);
    ROS_INFO("SENSOR#3_BOOL: [%d]",sensor3.data);
    ROS_INFO("SENSOR#4_INT8: [%d]",sensor4.data);
    ROS_INFO("==========================");

    //Se publica en los topics respectivos
    pub1_Int32.publish(sensor1);
    pub2_Float64.publish(sensor2);
    pub3_Bool.publish(sensor3);
    pub4_Int8.publish(sensor4);

     
    //Rate
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
