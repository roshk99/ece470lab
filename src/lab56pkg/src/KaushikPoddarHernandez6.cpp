 #include "lab56pkg/lab56.h"
#include <algorithm>
extern ImageConverter* ic_ptr; //global pointer from the lab56.cpp

#define SPIN_RATE 20  /* Hz */

bool isReady=1;
bool pending=0;

float SuctionValue = 0.0;

bool leftclickdone = 1;
bool rightclickdone = 1;

/*****************************************************
* Functions in class:
* **************************************************/	

//constructor(don't modify) 
ImageConverter::ImageConverter():it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera_node/image_raw", 1, 
    	&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);   
    pub_command=nh_.advertise<ece470_ur3_driver::command>("ur3/command",10);
    sub_position=nh_.subscribe("ur3/position",1,&ImageConverter::position_callback,this); 

	sub_io_states=nh_.subscribe("ur_driver/io_states",1,&ImageConverter::suction_callback,this);
	
	srv_SetIO = nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");


    driver_msg.destination=lab_invk(-.3,-.3,0.2,-90);

	//publish the point to the robot
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
	driver_msg.duration = 3.0;
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
										  // the subscriber will not receive this message.
	spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO_STREAM("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}
	ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
	
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM("Ready for new point");

}

//destructor(don't modify)
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady;
	pending=msg->pending;
}

void ImageConverter::suction_callback(const ur_msgs::IOStates::ConstPtr& msg)
{
	SuctionValue = msg->analog_in_states[0].state;
}


//subscriber callback function, will be called when there is a new image read by camera
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    } 
    // create an gray scale version of image
    Mat gray_image;
	cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );  
    // convert to black and white img, then associate objects:  

// FUNCTION you will be completing
    Mat bw_image = thresholdImage(gray_image); // bw image from own function

// FUNCTION you will be completing
    Mat associate_image = associateObjects(bw_image); // find associated objects

    // Update GUI Window
    imshow("Image window", cv_ptr->image);
    imshow("gray_scale", gray_image);
    imshow("black and white", bw_image);
    imshow("associate objects", associate_image);
    waitKey(3);
    // Output some video stream
    image_pub_.publish(cv_ptr->toImageMsg());
} 

/*****************************************************
	 * Function for Lab 5
* **************************************************/	
// Take a grayscale image as input and return an thresholded image.
// You will implement your algorithm for calculating threshold here.
Mat ImageConverter::thresholdImage(Mat gray_img)
{
	Mat bw_img  = gray_img.clone(); // copy input image to a new image
	/*if (leftclickdone == 0 || rightclickdone == 0) {
		return bw_img;
	}*/
		int   totalpixels;

		totalpixels	  = gray_img.rows*gray_img.cols;			// total number of pixels in image

		int H[256] = {0};

		for (int r=0; r<gray_img.rows; r++) {
			for (int c=0; c<gray_img.cols; c++) {
				H[gray_img.data[r*gray_img.cols + c]]++;

			}
		}

		float mu = 0;
		for (int i=0; i<256; i++) {
			mu+=(float)i*H[i]/totalpixels;
		}

		int zt = 0;
		float zsigma_B = 0;

		float P_1, q0_1, P_2, q0_2, mu_0_1, mu_0_2, mu_1_1, mu_1_2, sigma_B;
		P_1 = H[0]/(float)totalpixels;
		q0_1 = P_1;
		mu_0_1 = 0;
		mu_1_1 = mu*totalpixels/(totalpixels-H[0]);

		//t = z+1
		for (int t=1; t<256; t++) {
			P_2 = H[t]/(float)totalpixels;
			q0_2 = q0_1 + P_2;
			mu_0_2 = t*P_2/q0_2 + q0_1/q0_2*mu_0_1;
			mu_1_2 = (mu - q0_2*mu_0_2) / (1 - q0_2);
			sigma_B = q0_2*(1-q0_2)*pow((mu_0_2-mu_1_2),2);
			
			if (sigma_B > zsigma_B) {
				zt = t;
				zsigma_B = sigma_B;
			}
			P_1 = P_2;
			q0_1 = q0_2;
			mu_0_1 = mu_0_2;
			mu_1_1 = mu_1_2;
		}
		
		uchar graylevel; // use this variable to read the value of a pixel
		//std::cout<<zt<<std::endl;
		// threshold the image
		for(int i=0; i<totalpixels; i++)
		{
			graylevel = gray_img.data[i];	
			if(graylevel>zt) bw_img.data[i]= 255; // set rgb to 255 (white)
			else             bw_img.data[i]= 0; // set rgb to 0   (black)
		}	
	return bw_img;	
}
/*****************************************************
	 * Function for Lab 5
* **************************************************/
// Take an black and white image and find the object it it, returns an associated image with different color for each image
// You will implement your algorithm for rastering here
Mat ImageConverter::associateObjects(Mat bw_img)
{	
	Mat associate_img = Mat::zeros( bw_img.size(), CV_8UC3 ); // function will return this image
	/*if (leftclickdone == 0 || rightclickdone == 0) {
		return associate_img;
	}*/
	//initiallize the variables you will use
	int height,width; // number of rows and colums of image
	int red, green, blue; //used to assign color of each objects
	//uchar pixel; //used to read pixel value of input image 
	height = bw_img.rows;
	width = bw_img.cols;
	int num = 0;
	// initialize an array of labels, assigning a label number to each pixel in the image
	// this create a 2 dimensional array pixellabel[row][col]
	/*int ** pixellabel = new int*[height];
	for (int i=0;i<height;i++) {
		pixellabel[i] = new int[width];
	}

	num = 0;
	// creating a demo image of colored lines
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++) 
		{
			pixellabel[row][col] = num;
		}
		num++;
		if (num == 10) {
			num = 0;
		}
	}*/

	int label[10000];
	int *equiv[10000];
	for (int i=0; i<10000; i++) {
		equiv[i] = &label[i];
	}

	int pixellabel[height][width];
	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++) {
			if (bw_img.data[r*width + c] == 255) {
				pixellabel[r][c] = -1;
			}
			else {
				pixellabel[r][c] = 0;
			}
		}
	}

	int labelnum = 1;
	int pixel, left, above, smallerbaselabel, min_val, max_val;
	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++) {
			pixel = pixellabel[r][c];
			
			if (c==0) {
				left = -1;
			}
			else {
				left = pixellabel[r][c-1];
			}
			if (r==0) {
				above = -1;
			}
			else {
				above = pixellabel[r-1][c];
			}
			
			if (pixel != -1) {
				if (left == -1 && above == -1) {
					pixellabel[r][c] = labelnum;
					label[labelnum] = labelnum;
					labelnum++;
					//cout << "Labelnum:" << labelnum << endl;
				}
				if (left != -1 && above == -1) {
					pixellabel[r][c] = left;
				}
				if (left == -1 && above != -1) {
					pixellabel[r][c] = above;
				}
				if (left != -1 && above != -1) {
					smallerbaselabel = min(*equiv[left], *equiv[above]);
					if (smallerbaselabel == *equiv[left]) {
						min_val = left;
						max_val = above;
					}
					else {
						min_val = above;
						max_val = left;
					}
					//cout << min_val << "," << max_val << endl;
					pixellabel[r][c] = smallerbaselabel;
					*equiv[max_val]=*equiv[min_val];
					equiv[max_val] = equiv[min_val];
				}

			}
			
		}
	}

	//cout << endl << "hi" << endl;
	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++) {	
			pixel = pixellabel[r][c];
			if (pixel != -1) {

				pixellabel[r][c] = *equiv[pixel];
			}
		}
	}

	int objectsize[10000] = {0};
	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++) {
			if (pixellabel[r][c] != -1) {
				objectsize[pixellabel[r][c]]++;
			}
		}
	}
	
	int low_bound = 100;
	int upper_bound = 30000;
	std::vector<int> cur_labels;
	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++) {
			if (pixellabel[r][c] != -1) {
				if (objectsize[pixellabel[r][c]] < low_bound || objectsize[pixellabel[r][c]] > upper_bound) {
					pixellabel[r][c] = -1;
				}
				else {
					if(std::find(cur_labels.begin(), cur_labels.end(), pixellabel[r][c]) == cur_labels.end())
					cur_labels.push_back(pixellabel[r][c]);
				}
			}
		}
	}
	int num_objects = cur_labels.size();
	//cout << "Number of Objects: " << num_objects << endl;
	int old_label;
	for (int i=0; i<num_objects; i++) {
		old_label = cur_labels[i];
		for (int r=0; r<height; r++) {
			for (int c=0; c<width; c++) {
				if (pixellabel[r][c] == old_label) {
					pixellabel[r][c] = i+1;
				}
			}
		}
	}

	float m00[num_objects];
	float m01[num_objects];
	float m10[num_objects];
	float rbar[num_objects];
	float cbar[num_objects];
	for (int k=0; k<num_objects; k++) {
		m00[k] = 0.0;
		m01[k] = 0.0;
		m10[k] = 0.0;
		rbar[k] = 0.0;
		cbar[k] = 0.0;
	}

	for (int r=0; r<height; r++) {
		for (int c=0; c<width; c++) {
			int k = pixellabel[r][c]-1;
			m10[k] += r;
			m01[k] += c;
			m00[k] += 1;
		}
	}


	for (int k=0; k<num_objects; k++) {
		rbar[k] = m10[k]/m00[k];
		cbar[k] = m01[k]/m00[k];
		//cout << "Object "<< k+1 << "is at " << rbar[k] << " and " << cbar[k] << endl;
	}

	// assign UNIQUE color to each object
	
	Vec3b color;
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			switch (  pixellabel[row][col] )
			{
				
				case 0:
					red    = 255; // you can change color of each objects here
					green = 255;
					blue   = 255;
					break;
				case 1:
					red    = 255; // you can change color of each objects here
					green  = 0;
					blue   = 0;
					break;
				case 2:
					red    = 0;
					green  = 255;
					blue   = 0;
					break;
				case 3:
					red    = 0;
					green  = 0;
					blue   = 255;
					break;
				case 4:
					red    = 255;
					green  = 255;
					blue   = 0;
					break;
				case 5:
					red    = 255;
					green  = 0;
					blue   = 255;
					break;
				case 6:
					red    = 0;
					green  = 255;
					blue   = 255;
					break;
                case 7:
                    red    = 128;
                    green  = 128;
                    blue   = 0;
                    break;
                case 8:
                    red    = 128;
                    green  = 0;
                    blue   = 128;
                    break;
                case 9:
                    red    = 0;
                    green  = 128;
                    blue   = 128;
                 	break;
				default:
					red    = 0;
					green = 0;
					blue   = 0;
					break;					
			}

			color[0] = blue;
			color[1] = green;
			color[2] = red;
			associate_img.at<Vec3b>(Point(col,row)) = color;
		}
	}
	for (int k=0; k<num_objects; k++) {
		int r_cen = (int) rbar[k]; int c_cen = (int) cbar[k];
		if (r_cen + 5 < 240*2 && c_cen + 5 < 320*2 && r_cen - 5 > 0 && c_cen - 5 > 0) {
			color[0] = 255; color[1] = 255; color[2] = 255;
			//cout << r_cen << " , " << c_cen << endl;
			for (int i=-5; i<6; i++) {
				associate_img.at<Vec3b>(Point(c_cen+i, r_cen)) = color;
				associate_img.at<Vec3b>(Point(c_cen, r_cen+i)) = color;
			}
		}
	}	
	//out << "hi" << endl;
	/*// Calculate Beta
	float d = 12.0/100; //distance in m in world
	cout << rbar[0] << "," << cbar[0] << "," << rbar[1] << "," << cbar[1] << endl;
	float pixel_d = sqrt(pow(rbar[1] - rbar[0], 2) + pow(cbar[1]-cbar[0], 2));
	cout << "pixel_d: " << pixel_d << "; d: " << d << "," << pixel_d/d << endl;
	float beta = pixel_d/d;
	cout << "Beta: " << beta << endl;

	//Calculate Theta
	float theta = atan2(rbar[1]-rbar[0], cbar[1] - cbar[0]);
	cout << "Theta: " << theta <<endl;

	//Calculate Tx Ty
	float xw_1 = 17.5/100;
	float yw_1 = 0;
	float T_x = (rbar[1] - 0.5*height)/beta - cos(theta)*xw_1 + sin(theta)*yw_1;
	float T_y = (cbar[1] - 0.5*width)/beta - sin(theta)*xw_1 - cos(theta)*yw_1;
	cout << "T_x: " << T_x << " , T_y: " << T_y << " o_r: " << 0.5*height << " o_c: " << 0.5*width << endl;*/
	return associate_img;
}

/*****************************************************
	*Function for Lab 6
 * **************************************************/
 //This is a call back function of mouse click, it will be called when there's a click on the video window.
 //You will write your coordinate transformation in onClick function.
 //By calling onClick, you can use the variables calculated in the class function directly and use publisher
 //initialized in constructor to control the robot.
 //lab4 and lab3 functions can be used since it is included in the "lab4.h" 
void onMouse(int event, int x, int y, int flags, void* userdata)
{
		ic_ptr->onClick(event,x,y,flags,userdata);

}
void ImageConverter::onClick(int event,int x, int y, int flags, void* userdata)
{
	float beta = 800.056; //986.594;
	float T_x = -0.100297; //-0.0616644;
	float T_y = 0.0913649; //0.0781791;
	float theta = -0.00038072;
	float o_r = 240;
	float o_c = 320;
	float z_down = 0.017;
	float z_up = 0.25;


	float x_c = -(y-o_r)/beta-0.00662261;//+1.63236/100;
	float y_c = -(x-o_c)/beta+0.17; //+15.3191/100;
	float x_w = cos(theta)*(x_c - T_x) + sin(theta)*(y_c - T_y);// + (20-16.8)/100;
	float y_w = -sin(theta)*(x_c - T_x) + cos(theta)*(y_c - T_y);
	
	// For use with Lab 6
	// If the robot is holding a block, place it at the designated row and column. 
	if  ( event == EVENT_LBUTTONDOWN ) //if left click, do nothing other than printing the clicked point
	{  	
		if (leftclickdone == 1) {
			leftclickdone = 0;  // code started
			ROS_INFO_STREAM("left click:  (" << x << ", " << y << ")");  //the point you clicked
			if (SuctionValue < 1.5) {
				cout << x_w*100 << "," << y_w*100 << endl;
				driver_msg.destination=lab_invk(x_w, y_w, z_up, 45);

				//publish the point to the robot
			    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
				int spincount = 0;
				driver_msg.duration = 3.0;
				pub_command.publish(driver_msg);  // publish command, but note that is possible that
													  // the subscriber will not receive this message.
				spincount = 0;
				while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
					ros::spinOnce();  // Allow other ROS functionallity to run
					loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
					if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
						pub_command.publish(driver_msg);
						ROS_INFO_STREAM("Just Published again driver_msg");
						spincount = 0;
					}
					spincount++;  // keep track of loop count
				}
				ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
				
				while(!isReady)
				{
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO_STREAM("Ready for new point");

				driver_msg.destination=lab_invk(x_w, y_w, z_down, 45);

				//publish the point to the robot
				pub_command.publish(driver_msg);  // publish command, but note that is possible that
													  // the subscriber will not receive this message.
				spincount = 0;
				while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
					ros::spinOnce();  // Allow other ROS functionallity to run
					loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
					if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
						pub_command.publish(driver_msg);
						ROS_INFO_STREAM("Just Published again driver_msg");
						spincount = 0;
					}
					spincount++;  // keep track of loop count
				}
				ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
				
				while(!isReady)
				{
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO_STREAM("Ready for new point");

				srv.request.fun = 1;
				srv.request.pin = 0;  //Digital Output 0
				srv.request.state = 1.0; //Set DO0 on
				if (srv_SetIO.call(srv)) {
					ROS_INFO("True: Switched Suction ON");
					SuctionValue = 2;
				} else {
					ROS_INFO("False");
				}
				driver_msg.destination=lab_invk(x_w, y_w, z_up, 45);

				//publish the point to the robot
				pub_command.publish(driver_msg);  // publish command, but note that is possible that
													  // the subscriber will not receive this message.
				spincount = 0;
				while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
					ros::spinOnce();  // Allow other ROS functionallity to run
					loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
					if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
						pub_command.publish(driver_msg);
						ROS_INFO_STREAM("Just Published again driver_msg");
						spincount = 0;
					}
					spincount++;  // keep track of loop count
				}
				ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
				
				while(!isReady)
				{
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO_STREAM("Ready for new point");
			leftclickdone = 1; // code finished
		}

		} else {
			ROS_INFO_STREAM("Previous Left Click not finshed, IGNORING this Click"); 
		}
	}
	else if  ( event == EVENT_RBUTTONDOWN )//if right click, find nearest centroid,
	{
		if (rightclickdone == 1) {  // if previous right click not finished ignore
			rightclickdone = 0;  // starting code
			ROS_INFO_STREAM("right click:  (" << x << ", " << y << ")");  //the point you clicked
			// put your right click code here
				if (SuctionValue > 1.5) {
				cout << x_w*100 << "," << y_w*100 << endl;
				driver_msg.destination=lab_invk(x_w, y_w, z_up, 45);

				//publish the point to the robot
			    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
				int spincount = 0;
				driver_msg.duration = 3.0;
				pub_command.publish(driver_msg);  // publish command, but note that is possible that
													  // the subscriber will not receive this message.
				spincount = 0;
				while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
					ros::spinOnce();  // Allow other ROS functionallity to run
					loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
					if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
						pub_command.publish(driver_msg);
						ROS_INFO_STREAM("Just Published again driver_msg");
						spincount = 0;
					}
					spincount++;  // keep track of loop count
				}
				ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
				
				while(!isReady)
				{
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO_STREAM("Ready for new point");

				driver_msg.destination=lab_invk(x_w, y_w, z_down, 45);

				//publish the point to the robot
				pub_command.publish(driver_msg);  // publish command, but note that is possible that
													  // the subscriber will not receive this message.
				spincount = 0;
				while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
					ros::spinOnce();  // Allow other ROS functionallity to run
					loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
					if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
						pub_command.publish(driver_msg);
						ROS_INFO_STREAM("Just Published again driver_msg");
						spincount = 0;
					}
					spincount++;  // keep track of loop count
				}
				ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
				
				while(!isReady)
				{
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO_STREAM("Ready for new point");

				srv.request.fun = 1;
				srv.request.pin = 0;  //Digital Output 0
				srv.request.state = 0.0; //Set DO0 on
				if (srv_SetIO.call(srv)) {
					ROS_INFO("True: Switched Suction OFF");
					SuctionValue = 1;
				} else {
					ROS_INFO("False");
				}
				driver_msg.destination=lab_invk(x_w, y_w, z_up, 45);

				//publish the point to the robot
				pub_command.publish(driver_msg);  // publish command, but note that is possible that
													  // the subscriber will not receive this message.
				spincount = 0;
				while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
					ros::spinOnce();  // Allow other ROS functionallity to run
					loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
					if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
						pub_command.publish(driver_msg);
						ROS_INFO_STREAM("Just Published again driver_msg");
						spincount = 0;
					}
					spincount++;  // keep track of loop count
				}
				ROS_INFO_STREAM("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
				
				while(!isReady)
				{
					ros::spinOnce();
					loop_rate.sleep();
				}
				ROS_INFO_STREAM("Ready for new point");
			rightclickdone = 1; // code finished
		}
		} else {
			ROS_INFO_STREAM("Previous Right Click not finshed, IGNORING this Click"); 
		}
	}
}

