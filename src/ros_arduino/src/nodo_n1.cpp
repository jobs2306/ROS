#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "std_msgs/Int8.h" 


#include <sstream>
 //Variable de entrada para obtener el dato recibido
int entrada;
 //Funcion callback para tomar el dato del subscriber (Int32)
void Callback_a(const std_msgs::Int8::ConstPtr& dato_recibido)
{
  entrada = dato_recibido->data;
}

//Funcion principal
int main(int argc, char **argv)
{
  //Se inicializa el nodo
  ros::init(argc, argv, "nodo_n1");

  //Se crean los publisher
  ros::NodeHandle n;
  ros::Publisher pub1_String =   n.advertise<std_msgs::String>("topic_n1_n2", 10);

  //subscriber
  ros::Subscriber sub1_Int8 = n.subscribe("topic_p_n1", 10, Callback_a);

  //Se define la frecuencia
  ros::Rate loop_rate(10);

  //variables emuladas
  std_msgs::String salida;
  salida.data = "";

  while (ros::ok())
  {
    //Logica para especificar que hacer con la variable de entrada
    if (entrada < 9)
    {
     salida.data = "hola";
    }
    else if (entrada < 17)
    {
     salida.data = "mundo";
    }
    else if (entrada < 32)
    {
     salida.data = "ros"; 
    }
    else
    {
     salida.data = "JLW";
    }
    
    //Debug para imprimer en pantalla
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
