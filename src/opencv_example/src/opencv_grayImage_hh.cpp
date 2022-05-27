/* EXAMPLE 2 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Load, Modify, and Save an Image"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html#load-save-image
/*
/*   Aditional: Presentation "ROS_Lecture10.pptx"
/*   File: Annexed
/* 
/*   it was modificated by Hernán Hernández to be use like own template 

Este codigo vuelve a tonos grises cualquier imagen, en este caso la imagen a convertir viene directamente de la imagen de una 
camara conectada al pc (Puede ser la camara principal del computador, antes de utilizar estos codigos se debe verificar cuales camaras
están conectados al pc y darles permiso)
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>  //Libreria para poder trabajar con imagenes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_grayImage_hh"   //Se define nodo para publicar la imagen en gris
#define    OPENCV_WINDOW1       "Original Image"    //Ventana que muestra la imagen original 
#define    OPENCV_WINDOW2       "Gray Image"       //Ventana que muestra la imagen modificada digitalmente (escala de grises)

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Imagen obtenida de la camara 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Imagen que se publica en el nodo (en escala de grises)

// CLASS: Image Conver (OpenCV)
//Esta clase es llamada desde Main(), aquí es donde se va a hacer todo el trabajo de imagen para convertir la imagen a escala de grises
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_; 

    	// Image used 
    	image_transport::ImageTransport it_; // Objeto utilizado para el proceso digital de la imagen
    	image_transport::Subscriber topic1_sub__image_input; //Imagen obtenida de la camara, se utiliza para obtener la información del Topic
    	image_transport::Publisher topic1_pub__image_output; // Imagen procesada, se utiliza para publicar en el topic

    public:

	/* Constructor Method. 
	   TODO */
  	ImageConverter() : it_(nh_)
  	{
    	    // Topics declaration
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);  //Se inicializa el topic que se subscrible
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1); //Se inicializa el topic que publica 
		   
    	cv::namedWindow(OPENCV_WINDOW1); //Ventana para mostrar la imagen cruda
	    cv::namedWindow(OPENCV_WINDOW2); //Ventana para mostrar la imagen trabajada
  	}

	/* Metodo destructor para eliminar las cerrar abiertas*/
  	~ImageConverter()
  	{
	    /
    	cv::destroyWindow(OPENCV_WINDOW1);
	    cv::destroyWindow(OPENCV_WINDOW2);
  	}

	/*Función para trabajar la imagen optenida del topic "TOPIC1_SUB__IMAGE_INPUT" */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la imagen obtenida de la camara 
  	{
	    // Mediante este metodo se convierte de una imagen de ROS a una imagen de OpenCV para poder trabajarla	    
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    	    }
	    catch (cv_bridge::Exception& e)
    	    {
		// Imprime un error en caso que sea detectado 
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	    }

	    /****************************/ 
	    /* Procesando la imagen digitalmente */
	    /****************************/
    	    // Se convierten los datos de ptr a datos Mat para poder utilizarlos
	    cv::Mat cvMat_OriginalImage_ptr = cv_OriginalImage_ptr->image;

	    // Transforma la imagen original a escala de grises
	    cv::Mat cvMat_GrayImage_ptr;
	    cv::cvtColor(cvMat_OriginalImage_ptr, cvMat_GrayImage_ptr, CV_BGR2GRAY); 

	    /*********************************/ 
	    /* Fin del proceso digital de la imagen */
	    /*********************************/

    	    // Se actualiza la información de las ventanas abiertas "Windows1" y "Windows2"
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_ptr);
    	    cv::waitKey(3);

          //Se convierte la imagen de Opencv Mat a a Bridge y luego a formato de Topic para poder enviarlo por el nodo 
	    cv_bridge::CvImage cv_GrayImage; // it's needed use the Class CvImage not CvImagePtr
	    cv_GrayImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    cv_GrayImage.encoding = sensor_msgs::image_encodings::MONO8; // MONO8 AND MONO16 are equal to GRAY format
	    cv_GrayImage.image = cvMat_GrayImage_ptr; // data

		   //Se publica la imagen modificada en el nodo de salida
	    topic1_pub__image_output.publish(cv_GrayImage.toImageMsg());
  	}
};

//---Main---
int main(int argc, char** argv)
{
    // Se inicializa Ros
    ros::init(argc, argv, NODE_NAME);
  
    // Se incia la clase ImageConverter para procesar la imagen
    ImageConverter ic;

    // While true. Getting data from subscribe Topic
    ros::spin();
    return 0;
}
