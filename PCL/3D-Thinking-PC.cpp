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

//Point Cloud Library includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

//Standard includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <math.h>

//hidapi for USB communication
#include "hidapi.h"

//Application defines
#define BUFSIZE 64			//number of bytes in a USB packet
#define PACKETSIZE 2		//number of bytes for each application level data packet
#define MAXHEIGHT 480		//max height of a scan
#define MAXROTATION 180		//max rotational steps of a scan

#define CMDSTART 64			//b'0100 0000
#define CMDEND 128			//b'1000 0000
#define CMDDATA 0			//b'0000 0000
#define CMDBLANK 192		//b'1100 0000
#define CMDMASK 192			//b'1100 0000

#define STATE_SCAN_START 0	//
#define STATE_SCAN_RUN 1	//
#define STATE_SCAN_END 2	//

//Global variables
hid_device *handle; //hid connection handle

//USB transmission buffers, RawHID supports a max size of 64 bytes
//bufOut must have an extra byte that is used as a control byte for USB RawHID
//in most cases that value must be 0 or else the communication will fail
//The device will never see this first byte and will only see a packet of size BUFSIZE
unsigned char bufOut[BUFSIZE+1] = {0};
unsigned char bufIn[BUFSIZE] = {0};

//Function definitions
void savePCD(pcl::PointCloud<pcl::PointXYZ>& cloud_data);
void exportVTK(pcl::PointCloud<pcl::PointXYZ>& cloud_data);
void addCloudData(pcl::PointCloud<pcl::PointXYZ>& cloud, int hor, int vert, int distance, int curPoint);
void setupCloud(pcl::PointCloud<pcl::PointXYZ>& cloud_data);
char* timeStamp(char* txt);
void displayCloud(pcl::PointCloud<pcl::PointXYZ>& cloud_data);
void startScan();
void openFile();
int connectRawHid(int pause);
void displayHIDDetails();
void disconnectRawHid();

/*
* Saves the point cloud in the native file format timestamped to the current time
*/
void savePCD(pcl::PointCloud<pcl::PointXYZ>& cloud_data){
	char times[40] = "scan-";
	//Save it to a file
	pcl::io::savePCDFileASCII (timeStamp(times), cloud_data);
}

/*
* Exports the given point cloud as a mesh after calculating a mesh to fit the points
* TODO, this is very experimental and not supported
*/
void exportVTK(pcl::PointCloud<pcl::PointXYZ>& cloud_data){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_post (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_post = cloud_data;
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
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

/*
* Converts the given scan data into rectangular coordinates and adds it to the point cloud
*/
void addCloudData(pcl::PointCloud<pcl::PointXYZ>& cloud, int hor, int vert, int distance, int curPoint){
	//hor ranges 0 -> 179 and is a 90 degree swing so 45 left and right of center
	//vert ranges from 0 -> 479 and is a 90 degree swing
	//distance is in mm or 10 for each 1 unit here
	float rho = distance / 10; //TODO distance needs to be calibrated based on output from FPGA
	float phi = vert / 5.3 + 45;
	float theta = hor / 2.0;

	cloud.points[curPoint].z = rho * sin(phi*M_PI/180) * cos(theta*M_PI/180);
	cloud.points[curPoint].x = rho * sin(phi*M_PI/180) * sin(theta*M_PI/180);
	cloud.points[curPoint].y = rho * cos(phi*M_PI/180);
}

/*
* Configures the cloud data size to match the incoming data set
*/
void setupCloud(pcl::PointCloud<pcl::PointXYZ>& cloud_data){
	cloud_data.width    = 180;
	cloud_data.height   = 480;
	cloud_data.is_dense = false;
	cloud_data.points.resize (cloud_data.width * cloud_data.height);
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
void displayCloud(pcl::PointCloud<pcl::PointXYZ>& cloud_data){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_post (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_post = cloud_data;
	//Show the visualizer of the cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Three Dimensional Thinking"));
	//viewer->addCoordinateSystem (10.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud<pcl::PointXYZ> (cloud_post, "Scan Data");
	//Block the program while the scan window is open
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	}
}

/*
* Initiates and completes a scan
* Saves the scan data to a file and displays it to the user
* Blocks until the user closes the viewer window
*/
void startScan(){
	//Create the point cloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//init to default size
	setupCloud(*cloud);
    //reset scan varibles
	unsigned int state = STATE_SCAN_START;
	unsigned int rotation = 0;
	unsigned int height = 0;
	unsigned char cmd;
	unsigned int range;
	unsigned int curPoint = 0;
	int i;
	int res;

	//continue looping until the scan is complete
	while(state != STATE_SCAN_END){
		if(state == STATE_SCAN_START){
			//The first byte out must be 0, see the declaration of bufOut for more detail
			bufOut[0] = 0;
			bufOut[1] = CMDSTART;
			//Command byte to start scan
			if(hid_write(handle, bufOut, sizeof(bufOut)) <= 0){
				printf("Scanner disconnected. . . Plug it in and restart the scan.\n");
				system("pause");
				return;
			}
			state = STATE_SCAN_RUN;
		}else if (state == STATE_SCAN_RUN){
			//Reads an input from the uC, will block until a packet is recieved
			if(hid_read(handle, bufIn, sizeof(bufIn)) <= 0){
				printf("Scanner disconnected. . . Plug it in and restart the scan.\n");
				system("pause");
				return;
			}
			//check the upper 2 bits of the first byte for a command
			if((bufIn[0] & CMDMASK) == CMDSTART){
				//do nothing this is expected as the first byte
				//There is currently no reset ability built in so this is ignored
			}else if((bufIn[0] & CMDMASK) == CMDEND){
				//Ends the scan
				state = STATE_SCAN_END;
			}else if ((bufIn[0] & CMDMASK) == CMDDATA){
				//Start and End commands are always in the first block of a HID packet
				//The only check needed when looping over a HID packet is to see if they have data or are blank
				//HID packet
				//Loop over all the data packets in the USB packet
				for(i=0;i<BUFSIZE/PACKETSIZE;i++){
					if((bufIn[0+i*PACKETSIZE] & CMDMASK) == CMDDATA){
						//Take the range from the 2 data packet bytes
						range = bufIn[1+i*PACKETSIZE] + ((bufIn[0+i*PACKETSIZE] & 15) << 8);
						//Add the cloud data point
						addCloudData(*cloud, rotation, height, range, curPoint);
						curPoint++;
						//Walk through all the height and rotational limits
						//This assumes data will be recieved in a specific order.
						//Height from 0-MAXHEIGHT at the far left rotation and then repeat
						//stepping the rotation to the right and walking the height from 0-MAXHEIGHT
						height++;
						if(height == MAXHEIGHT){
							rotation++;
							height = 0;
						}
					}
				}//for
			}//cmd check
		}//state check
	}//while

	//Save the scan to a file and then display it to the user
	savePCD(*cloud);
	//blocks until the user closes the window
	displayCloud(*cloud);
}

/*
* Displays an open file console dialog
* User enters a filename on the command line to display
*/
void openFile(){
	//Create the point cloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	std::string filename;
	//Wait for the user to enter a valid filename
	//reprompt if they enter an invalid name
	while(true){
		std::cout << "\n\nEnter the filename\n" ;
		std::cin >> filename ;
		//attempt to load the given file
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1)
		{
			//If opening failed print error and restart loop
			std::cout << "ERROR: file could not be opened\n" ;
		} else {
			//Display the cloud then break from the loop
			//after the user closes the display window
			std::cout << "Opening file...\n" ;
			displayCloud(*cloud);
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
		}
	}
	return 0;
}