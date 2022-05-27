/* EXAMPLE 4 USING OPENCV
/*
/*   Base: Smoothing Images
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html#smoothing
/*
/*   Video explain the Gaussian Blur Filter 
/*   From = https://www.youtube.com/watch?v=7LW_75E3A1Q
/*
/*   it was modificated by Hernán Hernández to be use like own template 

Este codigo suaviza una imagen, en este caso la imagen a trabajar viene directamente de la imagen de una 
camara conectada al pc (Puede ser la camara principal del computador, antes de utilizar estos codigos se debe verificar cuales camaras
están conectados al pc y darles permiso)
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h> //Libreria para poder trabajar con imagenes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_smoothingImages_hh"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "New Image (Filter Applied)"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Imagen obtenida de la camara. 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Imagen de salida

// Se define los filtros.
#define    HOMOGENEOUS_BLUR	0
#define    GAUSSIAN_BLUR	1
#define    MEDIAN_BLUR		2
#define    BILATERAL_BLUR	3
int Filter_Selected = 0; // Los filtros van de 0 a 3.

// Se hace un trackbar para definir el valor del tamaño del Kernel aplicado a los filtros
int Kernel_length_slider = 3; // Valor del slider
static void trackbar1_func(int, void*)
{
   //Kernel_length_slider from 0 to 30 (Define when the trackbar is create
}

//***STATIC FUNCTION: Trackbar to define the filter used
static void trackbar2_func(int, void*)
{
    // Min value 1. Max value 3 (Defined upon) 
    // Filter_Selected
}

//***Clase ImageConverter para el procesamiento de la imagen***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;

    	// Image used 
    	image_transport::ImageTransport it_; // Objeto para el proceso de la imagen
    	image_transport::Subscriber topic1_sub__image_input; // Imagen obtenida de la camara
    	image_transport::Publisher topic1_pub__image_output; // Imagen procesada

    public:

	/* Metodo constructor
	   TODO */
  	ImageConverter() : it_(nh_)
  	{
    	    // Declaración de topicos
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); 
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

	    // Creación de la ventana GUI
    	    cv::namedWindow(OPENCV_WINDOW1);
	    cv::namedWindow(OPENCV_WINDOW2);

	    // Creación del scrollbar para modificar los valores del filtro y Kernel
	    int trackbar1_maxValue = 50; // In units.
	    cv::createTrackbar("Kernel length [1-50]", OPENCV_WINDOW2, &Kernel_length_slider, trackbar1_maxValue, trackbar1_func); // Comments: View "opencv_change_contrast_hh.cpp" code
	    int trackbar2_maxValue = 3; // In units.
	    cv::createTrackbar("Filter [1-3]", OPENCV_WINDOW2, &Filter_Selected, trackbar2_maxValue, trackbar2_func);
  	}

	/* Metodo destructor */
  	~ImageConverter()
  	{
	    // close the GUI Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);
	    cv::destroyWindow(OPENCV_WINDOW2);
  	}

	/*Función para el procesado de las imagenes */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la imagen obtenida del nodo
  	{
	    // Convert ROS image (Topic) to OpenCV image (Ptr)	    
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
	    /* Procesamiento de la imagen */
	    /****************************/
   
    	    // Los datos de la imagen se pasan a formato Mat para poder procesarla
	    cv::Mat cv_OriginalImage_Mat = cv_OriginalImage_ptr->image;
	    
	    // New image to be processed of cv::Mat class
	    cv::Mat cv_ImageProcessed_Mat = cv_OriginalImage_Mat.clone();

	    //** Filtros aplicados **/
	    int l = kernel_format(Kernel_length_slider);
	    //Switch para seleccionar el tipo de filtro aplicado
	    switch(Filter_Selected)
	    {
		// Homogeneous blur
		case 0:
		    cv::blur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), cv::Point(-1,-1));
	    	    ROS_INFO("FILTER APPLIED: ** 0. Homogeneous blur **");
		break;
		// Applying Gaussian blur
		case GAUSSIAN_BLUR:
		    cv::GaussianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), 0, 0);
		    ROS_INFO("FILTER APPLIED: ** 1. Gaussian blur **");
		break;
		// Applying Median blur
		case MEDIAN_BLUR:
		    cv::medianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l);
		    ROS_INFO("FILTER APPLIED: ** 2. Median blur **");
		break;
		// Applying Bilateral Filter
		case BILATERAL_BLUR:
		    cv::bilateralFilter(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l, l*2, l/2 );
		    ROS_INFO("FILTER APPLIED: ** 3. Bilateral blur **");
		break;	
	    }	
	    ROS_INFO("    KERNEL_LENGTH: %d x %d", l, l);
	    ROS_INFO(" ");
	    //** Fin de filtros aplicados **/ 
	
	    /*********************************/ 
	    /* Fin del procesamiento de la imagen */
	    /*********************************/

    	    // Actualización de las ventanas de la GUI
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

   	    cv::imshow(OPENCV_WINDOW2, cv_ImageProcessed_Mat);
	    cv::waitKey(3);

    	    // Convert OpenCV image (Mat) to OpenCV image (Bridge) to ROS image (Topic)  
	    //cv_bridge::CvImage cv_NewImage; // it's needed use the Class CvImage not CvImagePtr
	    //cv_NewImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    //cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Same format as Original image 
	    //cv_NewImage.image = cvMat_NewImage_ptr; // data
    	    // Output modified video stream
	    //topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
  	}
	
	/* Only take the odd numbers */
	int kernel_format(int value)
	{ 
	    if(value%2 == 0)
		return value + 1;
	    else 
		return value;
	}
};

//***Main***
int main(int argc, char** argv)
{
    // Init ROS 
    ros::init(argc, argv, NODE_NAME);
  
    // Init object from class ImageConverter, defined above
    ImageConverter ic;

    // While true. Getting data from subscribe Topic
    ros::spin();
    return 0;
}
