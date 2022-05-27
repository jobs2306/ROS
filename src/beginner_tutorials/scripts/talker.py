#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

#Linea para indicar que es un python script
import rospy				
# Para almacenar (a simple string container) for publishing
from std_msgs.msg import String		

def talker():
    sumador = 0;
    #Se conecta al "topic1", indica que el tipo de mensaje es string y el largo del mensaje
    pub = rospy.Publisher('topic1', String, queue_size=10) 
    #Inicializa el nodo "talker" y agrega el anonymous para que sea unico
    rospy.init_node('talker', anonymous=True)
    #velocidad de transferencia de los mensajes
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #Se escribe el mensaje que se quiere enviar
        N = str(sumador)
        hello_str = N + "hola %s" % rospy.get_time()
        #Se imprime en la consola
        rospy.loginfo(hello_str)	
        #se publica en el "topic1"
        pub.publish(hello_str)		
        rate.sleep()
        sumador = sumador + 1;

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:	#"C
        pass
