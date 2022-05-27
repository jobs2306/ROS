/* EXAMPLE 3 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Changing the contrast and brightness of an image!"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html#basic-linear-transform
/*
/*   For implement the scrollbar/trackbar was used the tutorial: "Adding a Trackbar to our applications!"
/*   From = https://docs.opencv.org/2.4/doc/tutorials/highgui/trackbar/trackbar.html
/*
/*   it was modificated by Hernán Hernández to be use like own template 

Mediante este codigo se puede modificar el brillo y contraste de una imagen, en este caso la imagen a convertir viene directamente de la imagen de una 
camara conectada al pc (Puede ser la camara principal del computador, antes de utilizar estos codigos se debe verificar cuales camaras
están conectados al pc y darles permiso)
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>   //Libreria para poder trabajar con imagenes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_change_contrast_hh"   //Nombre del nodo de salida
#define    OPENCV_WINDOW1       "Original Image"   //Nombre de la ventana que tiene la imagen original
#define    OPENCV_WINDOW2       "New Image (Contrast & brightness)" //Nonbre de la ventana con la imagen modifica

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Nombre del topic para obtener la imagen original  
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Nombre del topic para el topic de salida

//***STATIC FUNCTION: Trackbar Method for Alpha value. High-GUI of OpenCV***
double Alpha = 1.5; // Simple contrast control. Value from 1.0 to 3.0 
int trackbar1_slider; // where is stored the actual trackbar value

static void trackbar1_func(int, void*)
{
    // scale trackbar value [0 - 100%] to alpha value [0.0 - 3.0]
    Alpha = trackbar1_slider*3.0/100.0;
}

//Metodo para modificar un valor Beta para modificar el nivel del brillo
int Beta = 30;  // variable para el control del nivel de brillo, va de 0 a 100
int trackbar2_slider; // Donde es guardada el valor de trackbar 

static void trackbar2_func(int, void*)
{
    // Cambio del valor Beta del trackbar
    Beta = trackbar2_slider;
}

//***Clase para hacer el trabajo digital de la imagen***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;

    	// Image used 
    	image_transport::ImageTransport it_; // Objeto utilizado para hacer el procesamiento digital de la imagen
    	image_transport::Subscriber topic1_sub__image_input; // Topic para recibir la imagen sin procesar
    	image_transport::Publisher topic1_pub__image_output; // Topic para publicar la imagen procesada

    public:

	/* Metodo constructor 
	   TODO */
  	ImageConverter() : it_(nh_)
  	{
    	    // Se configuran los topics 
       	topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); 
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

	    // Creación de las ventanas para mostrar la imagen original y la imagen procesada
    	    cv::namedWindow(OPENCV_WINDOW1);
	    cv::namedWindow(OPENCV_WINDOW2);

	    // Create a new Scrollbar/trackbar
		// Se crea un nuevo scrollbar para modificar los valores de brillo y contraste
	    int trackbar_maxValue = 100; // In percent.
		//Trackbar para modificar el valor de Alpha
	    cv::createTrackbar("Alpha [0-100%]", OPENCV_WINDOW2, &trackbar1_slider, trackbar_maxValue, trackbar1_func); // Note the following: 1) Our Trackbar has a label "Alpha", 2) The Trackbar is located in the window “OPENCV_WINDOW2”, 3) The Trackbar values will be in the range from 0 to "trackbar_maxValue" (the minimum limit is always zero), 4) The numerical value of Trackbar is stored in "trackbar_slider", and 5) Whenever the user moves the Trackbar, the callback function on_trackbar is called
	    //Trackbar para modificar el valor de Beta
		cv::createTrackbar("Beta [0-100%]", OPENCV_WINDOW2, &trackbar2_slider, trackbar_maxValue, trackbar2_func); 
  	}

	/* Metodo destructor para cerrar las ventanas abiertas*/
  	~ImageConverter()
  	{
	    // close the GUI Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);
	    cv::destroyWindow(OPENCV_WINDOW2);
  	}

	/* associate to "TOPIC1_SUB__IMAGE_INPUT" which get  Image get from camera (raw) */
	//Función para hacer el proceso de imagen y publicar la imagen en el topic de publicación
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la imagen obtenida de la camara
  	{
		// Se convierte la imagen de un formato de Topic a formato de OpenCV (Ptr)
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    	    }
	    catch (cv_bridge::Exception& e)
    	    {
		// Print a error if it is detected
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	    }

	    /****************************/ 
	    /*Procesamiento digital de la imagen */
	    /****************************/
	   
		// Se convierte los datos a un formato Mat para procesar la imagen
	    cv::Mat cvMat_Image_ptr = cv_OriginalImage_ptr->image;

 	    //Se realiza el proceso de la imagen utilizando Alpha y Beta
		 cv::Mat cvMat_NewImage_ptr = cv::Mat::zeros(cvMat_Image_ptr.size(), cvMat_Image_ptr.type()); // Matrix of 0 of the same size
	    for( int y = 0; y < cvMat_Image_ptr.rows; y++ )
    	    { 
		for( int x = 0; x < cvMat_Image_ptr.cols; x++ )
         	{ 
		    for( int c = 0; c < 3; c++ )
              	    {
      			cvMat_NewImage_ptr.at<cv::Vec3b>(y,x)[c] = 
				cv::saturate_cast<uchar>( Alpha*(cvMat_Image_ptr.at<cv::Vec3b>(y,x)[c]) + Beta );
             	    }
    		}
    	    }
	    /*********************************/ 
	    /* Fin del proceso digital de la imagen */
	    /*********************************/

			//Actualización de las ventanas
     	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

   	       cv::imshow(OPENCV_WINDOW2, cvMat_NewImage_ptr);
	       ROS_INFO("Alpha %f ------ Beta %d", Alpha, Beta);
	       cv::waitKey(3);

			// Se convierte la imagen de formato Mat a Bridge y luego a formato Topic para publicar 
	    cv_bridge::CvImage cv_NewImage; // it's needed use the Class CvImage not CvImagePtr
	    cv_NewImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Same format as Original image 
	    cv_NewImage.image = cvMat_NewImage_ptr; // data
    	    // Se publica en el nodo publisher la imagen procesada
	    topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
  	}
};

//***Main***
int main(int argc, char** argv)
{
    // Init ROS 
    ros::init(argc, argv, NODE_NAME);
  
    // Se llama a la clase ImageConverter que hace todo el trabajo de procesamiento digital
    ImageConverter ic;

    // While true. Getting data from subscribe Topic
    ros::spin();
    return 0;
}
