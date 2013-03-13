/*
* Author: James Morris
* Group: 33 Three-Dimensional-Thinking
* 
* Description: PC application that connects via USB RawHID to 3D scanner and initiates
*	a scan and displays the resulting data.
*	Also saves and can load previous scans
*
* Can be opened without a connection to the scanner to load previous scans.
*
*/
#include "3D-Thinking-PC.h"

//Global variables
hid_device *handle; //hid connection handle

//Step mode is 1 for full step
//2 for half step
//4 for quarter step
//8 for eighth step
//16 for sixteenth step
unsigned int stepMode = 2;

//USB transmission buffers, RawHID supports a max size of 64 bytes
//bufOut must have an extra byte that is used as a control byte for USB RawHID
//in most cases that value must be 0 or else the communication will fail
//The device will never see this first byte and will only see a packet of size BUFSIZE
unsigned char bufOut[BUFSIZE+1] = {0};
unsigned char bufIn[BUFSIZE] = {0};

/*
* Saves the point cloud in the native file format timestamped to the current time
*/
void savePCD(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data){
	char times[40] = "scan-";
	//Save it to a file
	pcl::io::savePCDFileASCII (timeStamp(times), cloud_data);
}

/*
* Exports the given point cloud as a mesh after calculating a mesh to fit the points
* TODO, this is very experimental and not supported
*/
/*
void exportVTK(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_post (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_post = cloud_data;
	
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_post);
	n.setInputCloud (cloud_post);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud_post, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (3); //0.025

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	//save the file as a .vtk
	pcl::io::saveVTKFile ("mesh.vtk", triangles);
}
*/
/*
* Converts the given scan data into rectangular coordinates and adds it to the point cloud
*/

/*
*
*/
void addCloudData(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int hor, int vert, float distance, int curPoint, int red, int green, int blue){
	//hor ranges 0 -> 179 and is a 90 degree swing so 45 left and right of center
	//vert ranges from 0 -> 479 and is a 90 degree swing
	//distance is in mm or 10 for each 1 unit here
	float rho = distance;
	float phi = vert * 34.5 / 720.0 + 72.75;
	float theta = hor * 90.0 / 200.0;

	cloud.points[curPoint].z = rho * sin(phi*M_PI/180) * cos(theta*M_PI/180);
	cloud.points[curPoint].x = rho * sin(phi*M_PI/180) * sin(theta*M_PI/180);
	cloud.points[curPoint].y = rho * cos(phi*M_PI/180);
	cloud.points[curPoint].r = red;
	cloud.points[curPoint].g = green;
	cloud.points[curPoint].b = blue;

	//printf("hor %d vert %d distance %f rho %f phi %f theta %f X %f Y %f Z %f \n",hor, vert, distance, rho, phi, theta, cloud.points[curPoint].x, cloud.points[curPoint].y, cloud.points[curPoint].z);

	//printf("Added point R:%d G:%d B:%d\n", red, green, blue);
}

/*
* Configures the cloud data size to match the incoming data set
*/
void setupCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data){
	cloud_data.width    = (stepMode * STEPPER_BASE_STEPS);
	cloud_data.height   = CAMERA_HEIGHT;
	cloud_data.is_dense = false;
	cloud_data.points.resize ((stepMode * STEPPER_BASE_STEPS) * cloud_data.height);
}

/*
* Prepends the given char array to a timestamp in the form
* of mmdd_hhmmss and appends .pcd where hours is 24 hour time
* Example prefix0210_133721.pcd
* This is Feb 10th at 1:37:21 PM (13 for 24 hour time)
*/
char* timeStamp(char* txt){
	char* rc = "";
	char timestamp[16];
	//get the current time
	time_t rawtime = time(0);
	//format the time to a local timezone
	tm *now = localtime(&rawtime);
	//Use a time string formater to set timestamp string
	if(rawtime != -1) {
		strftime(timestamp,16,"%m%d_%H%M%S",now);
		//add the prefix string
		rc = strcat(txt,timestamp);
		//append .pcd
		rc = strcat(rc,".pcd");
	}
	return(rc);
}

/*
* Displays the given point cloud data
* Blocks until the user closes the point cloud display window
*/
void displayCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_post (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_post = cloud_data;
	//Show the visualizer of the cloud

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_post);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud_post, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (10.0);
	viewer->initCameraParameters ();

	viewer->setCameraPosition(-50,0,-50,1,1,1);

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	}
}

void startScan(){
	//Current stepper motor step location
	unsigned int currentStep = stepMode / 2 * STEPPER_BASE_STEPS;
	//Counter for the point cloud point
	int currentPoint = 0;
	//To access as 2D array one needs to address like the following scanData[i*4+j]
	float *scanData = new float[720*4];
	
	//Create the point cloud object
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	//init to default size
	setupCloud(*cloud);
	//step_mode (full, half, etc)
	step_mode(stepMode);
	//Wake the stepper motor
	step_wake();
	//rotate stepper left till currentStep == 0 (scan start position)
	//printf("Rotating left 45 degrees to start scan\n");
	while(currentStep > 0){
		step_left();
		currentStep--;
		Sleep(STEP_DELAY_FAST);
	}
	//create dummy data for the depth and colors
		for(int j=0;j<720;j++){
			//scanData[j*4+0] = 100 / cos( (i - 100) / 200.0 * M_PI/180.0 * 90.0 ) / cos((j-360) * 34.5 / 720.0 *M_PI/180);

			scanData[j*4+1] = 255; //RED
			scanData[j*4+2] = 255; //GREEN
			scanData[j*4+3] = 255; //BLUE
		}
	//printf("Begin loop\n");

	//Prep for scan start
	laser_on();
	Sleep(STEP_DELAY_SLOW);
	//
	//Loop over every step in the current mode
	//
	for(int i=0;i<stepMode * STEPPER_BASE_STEPS; i++){
		
		//printf("value %f where i is %d \n",  ( cos( (i - 100) / 200.0 * M_PI/180.0 * 90.0 ) ), i );
		//Take a picture (with laser)
		takePictureLaser();
		if(i==0){
			takePictureLaser();
		}
		//turn the laser off
		laser_off();
		Sleep(STEP_DELAY_SLOW);
		//Take a picture (base image)
		
		takePictureBase();
		if(i==0){
			takePictureBase();
		}
		//turn laser on
		laser_on();
		//Rotate stepper motor
		step_right();
		currentStep++;
		//Send pictures to analysis and get back the depth and color data
		//printf("hand off data");
		analyzeData(scanData);
		//printf("image processing complete");
		//Add the data to the cloud
		//scanData[i*4+j]
		for(int j=0;j<CAMERA_HEIGHT;j++){
			addCloudData(*cloud, currentStep, j, scanData[j*4+0], currentPoint, scanData[j*4+1], scanData[j*4+2], scanData[j*4+3]);
			currentPoint++;
		}
		//Sleep(STEP_DELAY_SLOW);
	//	printf("loop %d\n", i);
	}

	//laser off
	laser_off();
	//printf("Rotating left 45 degrees to origin\n");
	while(currentStep > stepMode / 2 * STEPPER_BASE_STEPS){
		step_left();
		currentStep--;
		Sleep(STEP_DELAY_FAST);
	}
	//send step sleep command
	step_sleep();
	//Save the scan to a file and then display it to the user
	savePCD(*cloud);
	//blocks until the user closes the window
	displayCloud(*cloud);
	//cleanup
	delete [] scanData;
//	printf("scan complete\n");
}

/*
* Sends the given command value and waits for an ack
* from the uC
* returns true if ack and false if NACK or timeout
*/
bool sendCommand(int cmd){
	
	bufOut[0] = 0;
	bufOut[1] = cmd;
	//Command byte to start scan
	if(hid_write(handle, bufOut, sizeof(bufOut)) <= 0){
		printf("Scanner disconnected. . . Plug it in and restart the scan.\n");
		system("pause");
		return false;
	}
	
	return true;
}

bool step_left(){
	return sendCommand(PC_STEP_LEFT);
}

bool step_right(){
	return sendCommand(PC_STEP_RIGHT);
}

bool step_sleep(){
	return sendCommand(PC_STEP_SLEEP);
}

bool step_wake(){
	return sendCommand(PC_STEP_WAKE);
}

bool step_mode(unsigned int mode){
	if(mode == 1){
		return sendCommand(PC_STEP_MODE_FULL);
	}else if(mode == 2){
		return sendCommand(PC_STEP_MODE_HALF);
	}else if(mode == 4){
		return sendCommand(PC_STEP_MODE_QUARTER);
	}else if(mode == 8){
		return sendCommand(PC_STEP_MODE_EIGHTH);
	}else if(mode == 16){
		return sendCommand(PC_STEP_MODE_SIXTEENTH);
	}
	return false;
}

bool laser_on(){
	return sendCommand(PC_LASER_ON);
}

bool laser_off(){
	return sendCommand(PC_LASER_OFF);
}

bool send_ACK(){
	return sendCommand(PC_ACK);
}

bool send_NACK(){
	return sendCommand(PC_NACK);
}

/*
* Displays an open file console dialog
* User enters a filename on the command line to display
*/
void openFile(){
	//Create the point cloud object
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	std::string filename;
	//Wait for the user to enter a valid filename
	//reprompt if they enter an invalid name
	while(true){
		std::cout << "\n\nEnter the filename\n" ;
		std::cin >> filename ;
		//attempt to load the given file
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1)
		{
			//If opening failed print error and restart loop
			std::cout << "ERROR: file could not be opened\n" ;
		} else {
			//Display the cloud then break from the loop
			//after the user closes the display window
			std::cout << "Opening file...\n" ;

			// Create a KD-Tree
		  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

		  // Output has the PointNormal type in order to store the normals calculated by MLS
		  pcl::PointCloud<pcl::PointNormal> mls_points;

		  // Init object (second point type is for the normals, even if unused)
		  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
 
		  mls.setComputeNormals (true);

		  // Set parameters
		  mls.setInputCloud (cloud);
		  mls.setPolynomialFit (true);
		  mls.setSearchMethod (tree);
		  mls.setSearchRadius (0.03);

		  // Reconstruct
		  mls.process (mls_points);

		  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
		  /*

		  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		  viewer->setBackgroundColor (0, 0, 0);

			//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointNormal>(mls_points);
		//	
			viewer->addPointCloud<pcl::PointNormal> (mls_points, "sample cloud");
			
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
			viewer->addCoordinateSystem (10.0);
			viewer->initCameraParameters ();

			viewer->setCameraPosition(-50,0,-50,1,1,1);

			while (!viewer->wasStopped ())
			{
				viewer->spinOnce (100);
			}


			*/


			//displayCloud(mls_points);
			break;
		}
	}
}

/*
* Connects to the hardcoded USB RawHID device
*/
int connectRawHid(int pause){
	// Open the device using the VID, PID,
	// and optionally the Serial number (currently ignored).
	handle = hid_open(0x16C0, 0x0486, NULL);
	if (!handle) {
		printf("Unable to open device, Cannot make a new scan until the device is plugged in\n");
		if(pause == 1){
			system("pause");
		}
 		return -1;
	}
	return 0;
}

/*
* Prints out the device details to the console
*/
void displayHIDDetails(){
	#define MAX_STR 255
	wchar_t wstr[MAX_STR];
	int res;

	// Read the Manufacturer String
	wstr[0] = 0x0000;
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	if (res < 0)
		printf("Unable to read manufacturer string\n");
	printf("Manufacturer String: %ls\n", wstr);

	// Read the Product String
	wstr[0] = 0x0000;
	res = hid_get_product_string(handle, wstr, MAX_STR);
	if (res < 0)
		printf("Unable to read product string\n");
	printf("Product String: %ls\n", wstr);

	// Read the Serial Number String
	wstr[0] = 0x0000;
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	if (res < 0)
		printf("Unable to read serial number string\n");
	printf("Serial Number String: (%d) %ls", wstr[0], wstr);
	printf("\n");
}

/*
* Disconnects from the currently connected USB RawHID device
*/
void disconnectRawHid(){
	hid_close(handle);
	// Free static HIDAPI objects. 
	hid_exit();
}

/*
* Main application entry point
* Console based user prompts to start a new scan, load a file from a previous scan
* or exit the application
*/
int main(int argc, char* argv[])
{
	//Connects to the scanner, if unable to connect it opens with the ability to open previous scans
	//A warning message is displayed
	if(connectRawHid(0) >= 0){
		displayHIDDetails();
		disconnectRawHid();
	}
	cameraInit();
	while(true){
		std::cout << "\n\nThree Dimensional Thinking\n\n1: New Scan \n2: Open file\n3: Close Program\n" ;
		std::size_t Number ;
		std::cin >> Number ;
		if ( Number == 1 ) {
			//Starts a new scan
			if(connectRawHid(1) >= 0){
				std::cout << "Scanning\n" ;
				std::cout << "Please wait for the scan to complete\n" ;
				startScan();
				disconnectRawHid();
			}
		} else if ( Number == 2) {
			//Goes to the OpenFile dialog
			openFile();
		} else if ( Number == 3) {
			//Closes the application
			break;
		}else if ( Number == 4) {
			connectRawHid(0);
			laser_on();
			disconnectRawHid();
		}else if ( Number == 5) {
			connectRawHid(0);
			laser_off();
			disconnectRawHid();
		}else if ( Number == 6) {
			/*
			connectCamera();
			unsigned char * frameBuffer = new unsigned char[cameraSize];
			cv::Mat frame;
			frame = take_picture(frameBuffer);
			IplImage ipl_img = frame;
			cvSaveImage("./imageTest.jpg", &ipl_img);
			disconnectCamera()
			*/
		}	
	}
	return 0;
}