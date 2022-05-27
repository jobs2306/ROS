#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include <sstream>

bool entrada;
  //Funcion callback para tomar el dato del subscriber (Bool)
void Callback_a(const std_msgs::Bool::ConstPtr& dato_recibido)
{
  entrada = dato_recibido->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nodo_c");

  //Se crean los publisher
  ros::NodeHandle n;
  ros::Publisher pub1_String =   n.advertise<std_msgs::String>("topic_c_f", 10);

  //subscriber
  ros::Subscriber sub1_Bool = n.subscribe("topic_p_c", 10, Callback_a);

  //Se define la frecuencia
  ros::Rate loop_rate(10);

  //variables emuladas
  std_msgs::String salida;
  salida.data = "";

  while (ros::ok())
  {
    //Logica
    if (entrada == true)
    {
     salida.data = "prendido";
    }
    else
    {
     salida.data = "apagado";
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
