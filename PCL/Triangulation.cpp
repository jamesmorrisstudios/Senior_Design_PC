#include "Triangulation.h"

cv::VideoCapture cap(0); 
cv::Mat laserFrame;
cv::Mat baseFrame;


void cameraInit(){
	/*
		cv::Mat frame;
		cap >> frame; // get a new frame from camera
		//cvtColor(frame, edges, CV_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		//Canny(edges, edges, 0, 30, 3);
		imshow("edges", frame);
		cv::waitKey(30);
	}
	*/
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
	cap.set(CV_CAP_PROP_FPS, 30);
}

void takePictureLaser(){
	//printf("picture laser\n");
	cv::Mat image(720, 1280, CV_8UC3);
	cap.read(image);
	image.copyTo(laserFrame);
	//cap.read(image);
	//image.copyTo(laserFrame);
}

void takePictureBase(){
	//printf("picture base\n");
	cv::Mat image2(720, 1280, CV_8UC3);
	cap.read(image2);
	image2.copyTo(baseFrame);
	//cap.read(image2);
	//image2.copyTo(baseFrame);
}

void analyzeData(float * scanData){
	//printf("what the balls");
	int threshold = 0;
	float l;
	float hyp;
	float theta;
	float delta;
	int *hold =new int[3];
	hold[0]=0;
	hold[1]=0;
	hold[2]=0;

	int *holdy1 =new int[3];
	holdy1[0]=0;
	holdy1[1]=0;
	holdy1[2]=0;

	int *holdy2 =new int[3];
	holdy2[0]=0;
	holdy2[1]=0;
	holdy2[2]=0;

	cv::Mat image(720, 1280, CV_8UC3);
	
	for(int i=0;i<720;i++)
	{//printf("what the balls");
	for(int j=1620; j<3200; j+=3)
		{
		//printf("what the balls");
		image.at<uchar>(i,j)= abs(laserFrame.at<uchar>(i,j) - baseFrame.at<uchar>(i,j));
		image.at<uchar>(i,j+1)= abs(laserFrame.at<uchar>(i,j+1) - baseFrame.at<uchar>(i,j+1));
		image.at<uchar>(i,j+2)= abs(laserFrame.at<uchar>(i,j+2) - baseFrame.at<uchar>(i,j+2));
		//printf("what the balls");
		//printf("%d,",image.at<uchar>(i,j));
		//printf("%d ",laserFrame->at<uchar>(i,j));
		//printf("%d ",baseFrame->at<uchar>(i,j));
			if((image.at<uchar>(i,j)+image.at<uchar>(i,j+1)+image.at<uchar>(i,j+2))>hold[0])
			{
				hold[0] = (image.at<uchar>(i,j)+image.at<uchar>(i,j+1)+image.at<uchar>(i,j+2));
				hold[1] = i;
				hold[2] =j/3;
			//printf("%d,%d,%d,",hold[0],hold[1],hold[2]);
			}
	  }
	//printf("\n");
	
	/*
		if(i%2==0)
	{
		holdy1[0]=hold[0];
		holdy1[1]=hold[1];
		holdy1[2]=hold[2];
		if(abs(holdy1[2]-holdy2[2])<20)
		{
			scanData[i*4]=holdy1[0];
			scanData[i*4+1]=holdy1[1];
			scanData[i*4+2]=holdy1[2];
			if(i!=0)
				
			{
			scanData[i*4-4]=holdy2[0];
			scanData[i*4-3]=holdy2[1];
			scanData[i*4-2]=holdy2[2];
			}
		}
		else
		{
			scanData[i*4]=0;
			scanData[i*4+1]=0;
			scanData[i*4+2]=0;
		}
	}
	if(i%2==1)
	{
		holdy2[0]=hold[0];
		holdy2[1]=hold[1];
		holdy2[2]=hold[2];
		if(abs(holdy1[2]-holdy2[2])<20)
		{
			scanData[i*4]=holdy2[0];
			scanData[i*4+1]=holdy2[1];
			scanData[i*4+2]=holdy2[2];
			scanData[i*4-4]=holdy1[0];
			scanData[i*4-3]=holdy1[1];
			scanData[i*4-2]=holdy1[2];
		}
		else
		{
			scanData[i*4-4]=0;
			scanData[i*4-3]=0;
			scanData[i*4-2]=0;
		}
		*/

	scanData[i*4] = hold[0];

	if ((hold[2]>850) || (hold[2]<640))
	{
		scanData[i*4] = 0;
		hold[1] = 0;
		hold[2] = 0;
	}
	if(scanData[i*4] > 3)
	{
	theta =((90-(hold[2]-588)*0.04531)*3.14159)/180;
	l=tan(theta)*7.625;
	delta =((460-i)*0.04792*3.14159)/180;
	hyp=(1/cos(delta))*l;
	scanData[i*4]=hyp;
	}
	//printf("%f,%f,%f,\n",scanData[i*4], hold[1], hold[2]);
	hold[0]=0;
	hold[1]=0;
	hold[2]=0;
    
	}
	/*
	for(int i =0; i<720;i++)
	{
		printf("%d,%d,%d,\n",scanData[i*4],scanData[i*4+1],scanData[i*4+2]);
	}
	*/
	/*
	imshow("test1", baseFrame); 
	imshow("test2", laserFrame);
	imshow("test3", image);
    if(cvWaitKey(200)==27);
	system("pause");
	*/
}