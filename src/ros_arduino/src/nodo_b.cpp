#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include <sstream>

float entrada;
 //Funcion callback para tomar el dato del subscriber (Float64)
void Callback_a(const std_msgs::Float64::ConstPtr& dato_recibido)
{
  entrada = dato_recibido->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nodo_b");

  //Se crean los publisher
  ros::NodeHandle n;
  ros::Publisher pub1_String =   n.advertise<std_msgs::String>("topic_b_e", 10);

  //subscriber
  ros::Subscriber sub1_Float = n.subscribe("topic_p_b", 10, Callback_a);

  //Se define la frecuencia
  ros::Rate loop_rate(10);

  //variables emuladas
  std_msgs::String salida;
  salida.data = "";

  while (ros::ok())
  {
    //Logica para categorizar la velocidad
    if (entrada < 3.5)
    {
     salida.data = "lento";
    }
    else if (entrada < 7)
    {
     salida.data = "normal";
    }
    else
    {
     salida.data = "rapido";
    }
    
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
