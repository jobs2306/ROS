Archivo ReadMe para explicar como correr cada codigo dentro de este repositorio

Este repositorio es de proyectos de ROS, por lo tanto se necesita tener instalado ROS para poder ejecutarlos, todo se hizo en la versión
Kinetic (Ubuntu 16.04), se recomienda utilizar Ubunto 20.04 o superior para aplicaciones con arduino. Para cada paquete se necesitan configuraciones especificas que se explicaran a detalle. 
Se recomienda instalar el terminal yakuake para poder utilizar varios terminales al tiempo

Para ejecutar un archivo se realiza de la siguiente manera:
En un terminal se ejecuta primeramente la linea "roscore" y en otra "rosrun nombrePaquete NombreArchivo"

Configuración rbx1-kinetic-devel-beta:
Carpeta de iniciación para utilizar ROS, los archivos son descargados de http://wiki.ros.org/ROS/Tutorials , se recomienda seguir cada tutorial para poder entender los siguientes paquetes

Configuración beginner_tutorials:
Nodo para aprender a usar nodos, para verlos en pantalla se utiliza "rqt_graph"
talker.py : Este archivo es un nodo que envia información a otro nodo
intermedio.py : Este archivo es un nodo que recibe información de un nodo y envia a otro 
listener.py : Este archivo es un nodo que recibe información 
Nota: Para poder ejecutarlos se debe configurar el archivo CMakeLists.txt, en las ultimas lineas se deben agregar el nombre del archivo
que se quiera utilizar

Configuración ros_arduino:
Los archivos dentro de este paquete son una aplicación de los nodos esta vez en formato c++, se recomienda ejecutar uno a uno y luego ejecutar rqt_graph para ver los nodos y verificar cada funcionamiento
A nivel general las aplicaciones son ir enviando distintos tipos de variables entre nodo y nodo.

Configuración opencv_example:
Primeramente para correr cada codigo se debe instalar OpenCV, para Ubunto 16.04 se recomienda el siguiente video:
https://www.youtube.com/watch?v=2Pboq2LFoaI&t=149s
Tambien se debe instalar usb_cam mediante "sudo apt-get install ros-kinetic-usb-cam"
Hay 2 formas de ejecutar este paquete, el primero es mediante el launch con la función "roslaunch opencv_example opencv_example_testing",
el archivo launch se debe modificar para especificar cual codigo correr (descomentar la opción que se requiera usar), en la linea que se especifica la camara "video0" se debe indicar cual camara se debe colocar, si video0 o video1 o la opción que salga en la carpeta dev de raiz 
La segunda opción es corriendo directamente linea a linea, primeramente "rosrun usb_cam usb_cam_node", luego el nodo a correr 
"rosrun opencv_example opencv_change_contrast_hh", en este caso se ejecuta el nodo que cambia el contraste y brillo mediante un scrollbar

# ROS
proyectos de ROS
