#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include <sstream>
//String para tomar el dato que llega del subscriber
std_msgs::String entrada;
//Funcion callback para tomar el dato
void Callback_a(const std_msgs::String::ConstPtr& dato_recibido)
{
  entrada.data = dato_recibido->data;
}

int main(int argc, char **argv)
{
 //se inicializa el nodo
  ros::init(argc, argv, "nodo_n2");

  //Se crean los publisher
  ros::NodeHandle n;
  ros::Publisher pub1_String =   n.advertise<std_msgs::String>("topic_n2_g", 10);

  //subscriber
  ros::Subscriber sub1_String = n.subscribe("topic_n1_n2", 10, Callback_a);

  //Se define la frecuencia
  ros::Rate loop_rate(10);

  //variables emuladas
  std_msgs::String salida;
  salida.data = "";

  while (ros::ok())
  {
    salida.data = entrada.data;
    //Debug
    std::stringstream ss;
    ss <<  "entrada: [" <<entrada<< "] = [" <<salida<< "]";

    std_msgs::String msg;
    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());
  
    ROS_INFO("=======================");
    
    //Se publica 
    pub1_String.publish(salida);

     
    //Rate
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
