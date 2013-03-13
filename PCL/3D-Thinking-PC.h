//Point Cloud Library includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>



//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/io/vtk_io.h>

//Standard includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <math.h>

//hidapi for USB communication
#include "hidapi.h"
#include "Triangulation.h"

//Application defines
#define BUFSIZE 64			//number of bytes in a USB packet

//PC to uC controls
#define PC_STEP_LEFT 1
#define PC_STEP_RIGHT 2
#define PC_STEP_SLEEP 3
#define PC_STEP_WAKE 4
#define PC_STEP_MODE_FULL 5
#define PC_STEP_MODE_HALF 6
#define PC_STEP_MODE_QUARTER 7
#define PC_STEP_MODE_EIGHTH 8
#define PC_STEP_MODE_SIXTEENTH 9
#define PC_LASER_ON 10
#define PC_LASER_OFF 11
#define PC_ACK 254
#define PC_NACK 253

#define STEPPER_BASE_STEPS 100

#define STEP_DELAY_FAST 20
#define STEP_DELAY_SLOW 10


#define CAMERA_HEIGHT 720
#define CAMERA_WIDTH 1280

//Function definitions
void savePCD(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data);
void exportVTK(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data);
void addCloudData(pcl::PointCloud<pcl::PointXYZRGB>& cloud, int hor, int vert, int distance, int curPoint, int red, int green, int blue);
void setupCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data);
void displayCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud_data);
void openFile();
char* timeStamp(char* txt);
void startScan();
bool sendCommand(int cmd);
bool step_left();
bool step_right();
bool step_sleep();
bool step_wake();
bool step_mode(unsigned int mode);
bool laser_on();
bool laser_off();
bool send_ACK();
bool send_NACK();
int connectRawHid(int pause);
void displayHIDDetails();
void disconnectRawHid();