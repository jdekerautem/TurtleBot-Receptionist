/* Inspired from http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 
 * and from https://github.com/mc-jesus/face_detect_n_track
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <std_msgs/UInt16.h>

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;

const cv::String	WINDOW_NAME("Camera video");
// Assuming the working directory is the catkin workspace
const cv::String	FACE_CASCADE_FILE("./src/ros_opencv_converter/src/haarcascade_frontalface_default.xml");
const double		TICK_FREQUENCY = cv::getTickFrequency();

class FaceDetector
{
	ros::NodeHandle nh_face;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	// image_transport::Publisher image_pub_;

	// Publisher for the position of the face
	ros::Publisher face_positionX_pub_;
	ros::Publisher face_positionY_pub_;
	ros::Publisher face_width_pub_;

public:
	// Count to prevent displaying FPS at every loop
	int count_before_displaying;

	// Pointer to cascade classifiers used to detect face
	cv::CascadeClassifier *face_cascade_classifier;

	// Vector used to store detected faces
	std::vector<cv::Rect> faces;

	// Measure time and fps
	int64 start_time, end_time;
	double average_FPS;

	// Rectangle around tracked face and ROI around it
	cv::Rect face, face_roi;

	// Matrix holding camera frame
	cv::Mat frame, face_template, matching_result;

	// Template matching failsafe
	bool template_matching_running;
	int64 template_matching_start_time, template_matching_current_time;

	// Flag indicating if a face was found
	bool found_face;

	FaceDetector() : it_(nh_face), count_before_displaying(0), average_FPS(0), 
		template_matching_running(false), template_matching_start_time(0), template_matching_current_time(0), 
		found_face(false), face_cascade_classifier(new cv::CascadeClassifier(FACE_CASCADE_FILE))
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &FaceDetector::detectFace, this);
		// image_pub_ = it_.advertise("/face_detector/output_video", 1);

		// Topic for Publishing the position of the face. Will buffer 2 messages before throwing away old ones
<<<<<<< HEAD
		face_positionX_pub_ = nh_face.advertise<std_msgs::Int8>("/face_detector/face_position_X", 2);
		face_positionY_pub_ = nh_face.advertise<std_msgs::Int8>("/face_detector/face_position_Y", 2);
		face_width_pub_ = nh_face.advertise<std_msgs::Int8>("/face_detector/face_width", 2);
=======
		face_positionX_pub_ = nh_face.advertise<std_msgs::UInt16>("/face_detector/face_position_X", 2);
		face_positionY_pub_ = nh_face.advertise<std_msgs::UInt16>("/face_detector/face_position_Y", 2);
		face_width_pub_ = nh_face.advertise<std_msgs::UInt16>("/face_detector/face_width", 2);
>>>>>>> f2146f717613478f0318e2d14e711e299968b0e7
	}

	~FaceDetector()
	{
		cv::destroyWindow(WINDOW_NAME);
		delete face_cascade_classifier;
	}

	void detectFaceAllSizes(cv::Mat &frame)
	{
	    // Minimum face size is 1/5th of screen height
	    // Maximum face size is 2/3rds of screen height
	    face_cascade_classifier->detectMultiScale(frame, faces, 1.1, 3, 0,
	        cv::Size(frame.rows / 5, frame.rows / 5),
	        cv::Size(frame.rows * 2 / 3, frame.rows * 2 / 3));

	    if (faces.empty()) return;

	    found_face = true;

	    // Locate biggest face;
	    face = biggestFace(faces); 

	    // Copy face template
	    face_template = frame(face).clone(); 
	    
	    // Calculate roi
	    face_roi = doubleRectSize(face, cv::Rect(0, 0, frame.cols, frame.rows)); 
	}

	// Returns rectangle with biggest surface area in array of rects
	cv::Rect biggestFace(std::vector<cv::Rect> &faces_arr) 
	{
	    int biggest = 0;
	    for (unsigned int i = 0; i < faces_arr.size(); i++) {
	        if (faces_arr[i].area() > faces_arr[biggest].area()) {
	            biggest = i;
	        }
	    }
	    return faces_arr[biggest];
	}

	void showFrame(cv::Mat &frame)
	{
	    cv::imshow(WINDOW_NAME, frame);
	    if (cv::waitKey(25) == 27) exit(0);
	}

	void startMeasuringTime() 
	{
	    start_time = cv::getTickCount();
	}

	void stopMeasuringTime() 
	{
	    end_time = cv::getTickCount();
	    double time_per_frame = (double)((end_time - start_time) / TICK_FREQUENCY);
	    double curr_FPS = 1. / time_per_frame;
	    average_FPS = (3 * average_FPS + curr_FPS) / 4;

	    if (count_before_displaying == 40){
	        std::cout << "Average FPS = " << average_FPS <<endl;
	        count_before_displaying = 0;
	    }else{
	        count_before_displaying++;
	    }
	}

	cv::Rect doubleRectSize(cv::Rect &input_rect, cv::Rect keep_inside)
	{
	    cv::Rect output_rect;
	    // Double rect size
	    output_rect.width	= input_rect.width * 2;
	    output_rect.height	= input_rect.height * 2;

	    // Center rect around original center
	    output_rect.x = input_rect.x - input_rect.width / 2;
	    output_rect.y = input_rect.y - input_rect.height / 2;

	    // Handle edge cases
	    if (output_rect.x < keep_inside.x) {
	        output_rect.width += output_rect.x;
	        output_rect.x = keep_inside.x;
	    }
	    if (output_rect.y < keep_inside.y) {
	        output_rect.height += output_rect.y;
	        output_rect.y = keep_inside.y;
	    }
	    
	    if (output_rect.x + output_rect.width > keep_inside.width) {
	        output_rect.width = keep_inside.width - output_rect.x;
	    }
	    if (output_rect.y + output_rect.height > keep_inside.height) {
	        output_rect.height = keep_inside.height- output_rect.y;
	    }

	    return output_rect;
	}

	void detectFaceAroundRoi(cv::Mat &frame)
	{
	    // Detect faces sized +/-20% off biggest face in previous search
	    face_cascade_classifier->detectMultiScale(frame(face_roi), faces, 1.1, 3, 0,
	        cv::Size(face.width * 8 / 10, face.height * 8 / 10),
	        cv::Size(face.width * 12 / 10, face.width * 12 / 10));

	    if (faces.empty())
	    {
	        // Activate template matching if not already started and start timer
	        template_matching_running = true;
	        if (template_matching_start_time == 0)
	            template_matching_start_time = cv::getTickCount();
	        return;
	    }

	    // Turn off template matching if running and reset timer
	    template_matching_running = false;
	    template_matching_current_time = template_matching_start_time = 0;

	    // Get detected face
	    face = biggestFace(faces); 

	    // Add roi offset to face
	    face.x += face_roi.x; 
	    face.y += face_roi.y;

	    // Get face template
	    face_template = frame(face).clone(); 

	    // Calculate roi
	    face_roi = doubleRectSize(face, cv::Rect(0, 0, frame.cols, frame.rows));
	}

	void detectFacesTemplateMatching(cv::Mat &frame)
	{
	    // Calculate duration of template matching
	    template_matching_current_time = cv::getTickCount();
	    double duration = (double)(template_matching_current_time - template_matching_start_time) / TICK_FREQUENCY;

	    // If template matching lasts for more than 2 seconds face is possibly lost
	    // so disable it and redetect using cascades
	    if (duration > 2) {
	        found_face = false;
	        template_matching_running = false;
	        template_matching_start_time = template_matching_current_time = 0;
    	}

	    // Template matching with last known face 
	    cv::matchTemplate(frame(face_roi), face_template, matching_result, CV_TM_CCOEFF);
	    cv::normalize(matching_result, matching_result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	    double min, max;
	    cv::Point min_loc, max_loc;
	    cv::minMaxLoc(matching_result, &min, &max, &min_loc, &max_loc);

	    // Add roi offset to face position
	    max_loc.x += face_roi.x;
	    max_loc.y += face_roi.y;

	    // Get detected face
	    face = cv::Rect(max_loc.x, max_loc.y, face.width, face.height);

	    // Get new face template
	    face_template = frame(face).clone();

	    // Calculate face roi
	    face_roi = doubleRectSize(face, cv::Rect(0, 0, frame.cols, frame.rows));
	}

	void drawRectAroundFace(cv::Mat &frame)
	{
	    cv::rectangle(frame, face, Scalar(255,0,0));

	    std_msgs::UInt16 face_positionX, face_positionY, face_width;
	    face_positionX.data = face.x;
	    face_positionY.data = face.y;
	    face_width.data = face.width;

	    // Publish the position of the face
	    face_positionX_pub_.publish(face_positionX);
	    face_positionY_pub_.publish(face_positionY);
	    face_width_pub_.publish(face_width);
	}

  	void detectFace(const sensor_msgs::ImageConstPtr& msg)
  	{
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception with face_detector: %s", e.what());
	      return;
	    }

	  	//-----------------------------------------------------------------------------------------------------------------------------
		// Create cascade classifier for face
	    // face_cascade_classifier = new cv::CascadeClassifier(FACE_CASCADE_FILE);
	    if (face_cascade_classifier->empty()) {
	        std::cout << "Error creating FACE cascade classifier. Make sure the file "<< FACE_CASCADE_FILE 
	        	<<" is in working directory/src/ros_opencv_converter/src/"<<endl;
	        exit(1);
	    }

	    // Find initial face on screen...
	    if (! found_face) 
	    {
<<<<<<< HEAD
	    	std_msgs::Int8 face_width;
=======
	    	std_msgs::UInt16 face_width;
>>>>>>> f2146f717613478f0318e2d14e711e299968b0e7
	    	face_width.data = 0;
	    	face_width_pub_.publish(face_width);
	        startMeasuringTime();
	        detectFaceAllSizes(cv_ptr->image); // Detect using cascades over whole image
	        showFrame(cv_ptr->image); 
	        stopMeasuringTime();
	    }

        // Once the face is found...
        if (found_face)
        {
            startMeasuringTime();
            detectFaceAroundRoi(cv_ptr->image); // Detect using cascades only in ROI
            if (template_matching_running) { // If Haar detection failed...
                detectFacesTemplateMatching(cv_ptr->image); // Detect using template matching
            }
            drawRectAroundFace(cv_ptr->image);
            showFrame(cv_ptr->image);
            stopMeasuringTime();
        }
		//-----------------------------------------------------------------------------------------------------------------------------

	    // Update GUI Window
	    cv::imshow(WINDOW_NAME, cv_ptr->image);
	    cv::waitKey(3);
	    
	    // Output modified video stream
	    // image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	char cwd[1024];
	if (getcwd(cwd, sizeof(cwd)) != NULL)
	   fprintf(stdout, "Current working dir: %s\n", cwd);

	ros::init(argc, argv, "face_detector");
	FaceDetector facedetector;
	ros::spin();
	return 0;
}