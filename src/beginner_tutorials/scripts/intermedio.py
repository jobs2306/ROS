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
import rospy
from std_msgs.msg import String

##talk

#Funcion talker que se llamara desde "intermedio", la idea es enviar la informacion que se recibe del "topic1" al "topic2" mediante
#la comunicacio entre las funciones
def talker(data):
    #Se conecta al "topic2" en la funcion del que escucha
    pub = rospy.Publisher('topic2', String, queue_size=10)	
    rate = rospy.Rate(10) # 10hz
    #Se toma el dato que se recibio de la funcion "intermedio" y se publica en pantalla y en el  "topic2"
    mensaje_str = data.data + "verificar"
    rospy.loginfo(mensaje_str)
    pub.publish(mensaje_str)

##listen

#Funcion "intermedio" que escucha lo que se envia del "topic1"
def intermedio():
    #Se inicializa el nodo intermedio que escucha y habla
    rospy.init_node('listentalk', anonymous=True)
    #Se subscribe al "topic1" y la informacion que llega se envia a la funcion "talk"
    rospy.Subscriber('topic1', String, talker)

    ## spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    intermedio()





