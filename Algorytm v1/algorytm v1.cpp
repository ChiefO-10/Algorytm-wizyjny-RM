#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <vector>
#include "opencv2/aruco/dictionary.hpp"
#include <opencv2/aruco.hpp>

#define _USE_MATH_DEFINES
#include <cmath>
#include<iostream>
#include<conio.h>  
#include<math.h>  

#include <windows.h>

using namespace std;

HANDLE hCOM;

int calibration();
int prog();
bool setupUART();
void video_capture(cv::VideoCapture& videocapture, cv::Mat& frame);
void video_capture_fast(cv::VideoCapture& videocapture, cv::Mat& frame);
static void saveCameraParams(string& filename, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
static void readCameraParams(string& filename, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
static void ClickCoordinates(int event, int x, int y, int flags, void* );
vector <double> calculate_markerVectors(vector<vector<cv::Point2f>> corners, cv::Point2d clickPt);
bool Send_UartComm(char* data, int dat_len);
bool Read_UartComm();
void stop();

string control(vector <double> position);
cv::Point2d pt(1,1);
bool new_Point = false;

int main() 
{
	
	for (int opt = 0; opt == 0;) {

		bool condition = false;
		while (condition == false)
		{
		
		int option;
		cout << "Chose action:" << endl << "1. Calibrate" << endl << "2. Run" << endl << "3. Close" << endl;
		cin >> option;

			switch (option)
			{
			case 1:
				calibration();
				break;
			case 2:
				setupUART();
				prog();
				break;
			case 3:
				opt++;
				condition = true;
				CloseHandle(hCOM); //close the handle
				break;
			default:
				cout << "Wrong action" << endl;
			}
		}
	}
	
	return(0);
}



int calibration()
{

	cv::VideoCapture capCalib;
	cv::Mat frameCalib;

	bool proceed = false;

	capCalib.open("http://192.168.0.20:8080/video");				//pobranie obrazu z IPwebcam

	if (!capCalib.isOpened()) {                                  // if unable to open image
		cout << "error: video stream unavailable";     // show error message on command line
		_getch();                                               // may have to modify this line if not using Windows
		return(0);                                              // and exit program
	}

	double dWidth = capCalib.get(CV_CAP_PROP_FRAME_WIDTH);					//szerokosc obrazu
	double dHeight = capCalib.get(CV_CAP_PROP_FRAME_HEIGHT);				//wysokosc obrazu
	cout << "Frame resolution: " << dWidth << "x" << dHeight << endl;	//wartosc rozdziellczosci obrazu wejsciowego
	cv::Size imageSize(dWidth, dHeight);

	bool foundCorners;
	vector<cv::Point2f> corners_temp;
	vector<vector<cv::Point2f>> detePoints;

	cv::Size boardSize(9, 6);
	//cv::TermCriteria criteria  (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);
	bool calpass = false;
	while (capCalib.isOpened()) {

		char Y_N_final = 0;


		if (!calpass) {
			for (int n = 0; n <= 9;) {
				char Y_N_single = 0;
				cout << "Press any key to take picture of calibration cheesboard..." << endl;
				_getch();
				video_capture(capCalib, frameCalib);

				foundCorners = cv::findChessboardCorners(frameCalib, boardSize, corners_temp,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
				cv::drawChessboardCorners(frameCalib, boardSize, cv::Mat(corners_temp), foundCorners);

				cv::namedWindow("Calibration image", CV_WINDOW_AUTOSIZE);		//okno wyswietlania
				cv::imshow("Calibration image", frameCalib);						//wyswietlanie obrazu 
				cv::waitKey(30);


				if (!foundCorners) {
					cout << "Pattern not found. Taking next frame..." << endl;
				}
				if (foundCorners) {
					cout << "Pattern  found! " << n + 1 << "/10" << endl;
					cout << "Accept image? y/n?" << endl;
					cin >> Y_N_single;
				}

				while (Y_N_single == 'y') {

					//cornerSubPix(frameCalib, corners_temp, boardSize,cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
					detePoints.push_back(corners_temp);
					n++;

					if (n == 10) {
						cout << "10 sample images taken" << endl;

						calpass = true;
						
					}
					break;
				}

			}

		}
		if (calpass)
		{
			cout << "Take pictures again? y/n ?" << endl;
			cin >> Y_N_final;
			switch (Y_N_final)
			{
			case 'y':
				calpass = 0;
				break;
			case'n':
				proceed = true;
				break;
			}
			
		}

	

		vector<cv::Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;
		cv::Mat cameraMatrix, distCoeffs;
		//calibration
		while (proceed == true) {
			int a = 26;										//bok kwadratu szachownicy w mm
			vector<vector<cv::Point3f> > arrayObjectPoints;
			vector<cv::Point3f> objectPoints;
			for (int y = 0; y < 6; ++y) {
				for (int x = 0; x < 9; ++x)
					objectPoints.push_back(cv::Point3f(x*a, y*a, 0));
			}
			for (int n = 0; n <= 9; ++n)
				arrayObjectPoints.push_back(objectPoints);

			//objectPoints.resize(detePoints.size(), objectPoints[0]);

			cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
			double rms = calibrateCamera(arrayObjectPoints, detePoints, imageSize, cameraMatrix,
				distCoeffs, rvecs, tvecs);

			cv::Mat imageUndistorted;
			undistort(frameCalib, imageUndistorted, cameraMatrix, distCoeffs);
			cv::namedWindow("undist", CV_WINDOW_AUTOSIZE);		//okno wyswietlania
			cv::imshow("undist", imageUndistorted);

			string filename = "Parameters.xml";
			saveCameraParams(filename, imageSize, cameraMatrix, distCoeffs);
			cv::waitKey(10);

			bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
			if (ok) {
				cout << "Camera parameters computed"<<endl;
			}
			else {
				cout << "Camera parameters computnig fail"<<endl;
			}
			break;
		}
		break;
	}
	
	_getch();
	return(0);
}

int prog()
{
	cv::Mat frame;
	cv::Mat frameGS;
	
	

	string filename = "Parameters.xml";
	cv::Mat cameraMatrix, distCoeffs;
	cv::Size imageSize;
	readCameraParams(filename, imageSize, cameraMatrix, distCoeffs);
	vector<cv::Vec3d> temp_rvecs, temp_tvecs;
	cv::Mat rot_matrix= cv::Mat::zeros(3,3,CV_64FC1);	
	const cv::Ptr<cv::aruco::Dictionary> markDict = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);  //stworzenie obiektu dictionary bedacego markerem gotowego zestawu?
	bool move = false;
	int n = 0;

	cv::VideoCapture capIP;
	capIP.open("http://192.168.0.20:8080/video");				//pobranie obrazu z IPwebcam

	if (!capIP.isOpened()) {                                  // if unable to open image
		cout << "error: video stream unavailable"<<endl;     // show error message on command line
		_getch();                                               
		return(0);                                              
	}

	double iWidth = imageSize.width;
	double iHeight = imageSize.height;
	cout << "Frame resolution: " << iWidth << "x" << iHeight << endl;	//wartosc rozdziellczosci obrazu wejsciowego


	char charCheckForEscKey = 0;
	while (charCheckForEscKey != 27 && capIP.isOpened())								//petla przerywana przyciskiem esc
	{
		video_capture_fast(capIP, frame);
		//cv::cvtColor(frame, frameGS, CV_BGR2GRAY);					//zmiana klatki wejsciowej na odcienie szarosci

		vector<int> markerIds;
		vector<vector<cv::Point2f>> markerCorners;
		vector <double> pose;
		

		cv::aruco::detectMarkers(frame, markDict, markerCorners, markerIds);
		
		cv::setMouseCallback("Video stream", ClickCoordinates, NULL);
		if (new_Point == true) {
			cout << "pt =" << pt.x << "," << pt.y << endl;
			move = true;
			new_Point = false;
		}

		if (markerIds.size() > 0) {

			cv::aruco::estimatePoseSingleMarkers(markerCorners, 50, cameraMatrix, distCoeffs, temp_rvecs, temp_tvecs);
			//cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
			
			//cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, temp_rvecs[i], temp_tvecs[i], 2);
			
			cv::Point3f tvec_3f(temp_tvecs[0][0], temp_tvecs[0][1], temp_tvecs[0][2]);	
			
			if (move == true) {
				pose = calculate_markerVectors(markerCorners, pt);
				cout << "MARKER POSE " << pose[0] << "," << pose[1] << " theta:" << pose[2] <<endl;
				
				double delta_pose[2] = { pose[0] - pt.x, pose[1] - pt.y };

				if (abs(delta_pose[0]) > 20 || abs(delta_pose[1]) > 20) {
					
					n++;
					string msg = control(pose);
					if (n = 30) {
						int len = msg.length();
						char *UART_msg = new char[len + 1];
						strcpy_s(UART_msg, 11, msg.c_str());
						Send_UartComm(UART_msg, len);
						//Read_UartComm();
						n = 0;
						cout << "msg OUT " << msg << endl;
					}
				}
				else {
					move = false;
					stop();
					cout << "Target attained" << endl;
				}

				

			}
	
		}
		else if (move == true && markerIds.size() == 0)
		{
			
			move = false;
			stop();
			cout << "Marker lost" << endl;
		}
		

		cv::namedWindow("Video stream", CV_WINDOW_AUTOSIZE);		//okno wyswietlania
		cv::imshow("Video stream", frame);						//wyswietlanie obrazu
		
		
		charCheckForEscKey = cv::waitKey(1);
		
	}
	CloseHandle(hCOM); //close the handle
}
void video_capture(cv::VideoCapture& videocapture, cv::Mat& frame)
{
	int i;
	for (i = 0; i < 30; i++) {
		bool bFrameread_1;
		bool bFrameRead_1 = videocapture.grab();				//wczytanie nowej klatki

		if (!bFrameRead_1) {
			cout << " error: Frame read failed" << endl;
		}
		
		
	}
	videocapture.retrieve(frame);
}
void video_capture_fast(cv::VideoCapture& videocapture, cv::Mat& frame)
{
	//int i;
	//for (i = 0; i < 8; i++) {
		bool bFrameread_1;
		bool bFrameRead_1 = videocapture.grab();				//wczytanie nowej klatki
		if (!bFrameRead_1) {
			cout << " error: Frame read failed" << endl;
		//}
	}
	videocapture.retrieve(frame);
}
//zapis do pliku
class Data
{
	public:
		Data() : imageSize(0,0),cameraMatrix(),distCoeffs(),rvecs(0),tvecs(0) {}

	public:
		cv::Size imageSize;
		cv::Mat cameraMatrix;
		cv::Mat distCoeffs;
		vector<cv::Mat> rvecs;
		vector<cv::Mat> tvecs;

		void write(cv::FileStorage& fs) const                        //Write serialization for this class
		{
			fs <<"{" 
			<<"image_Width" << imageSize.width 
			<< "image_Height" << imageSize.height
			<< "Camera_Matrix" << cameraMatrix
			<< "Distortion_Coefficients" << distCoeffs
			<< "rvecs" << rvecs 
			<< "tvecs" << tvecs 
			<< "}";
		}

		void read(const cv::FileNode& node)							//Read serialization for this class
		{
			node["image.Width"] >> imageSize.width;
			node["image.Height"] >> imageSize.height;
			node["Camera_Matrix"] >> cameraMatrix;
			node["Distortion_Coefficients"] >> distCoeffs;
			node["rvecs"] >> rvecs;
			node["tvecs"] >> tvecs;
		}
};
void write(cv::FileStorage& fs, const std::string&, const Data& x)
{
	x.write(fs);
}
void read(const cv::FileNode& node, Data& x, const Data& default_value = Data())
{
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}

static void saveCameraParams(string& filename, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs << "image_Width" << imageSize.width;					//cv::Size
	fs << "image_Height" << imageSize.height;
	fs << "Camera_Matrix" << cameraMatrix;                  // cv::Mat
	fs << "Distortion_Coefficients" << distCoeffs;
	
	
	fs.release();
}
static void readCameraParams(string& filename, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
cv::FileStorage fs;
fs.open(filename, cv::FileStorage::READ);

fs["image_Width"] >> imageSize.width;
fs["image_Height"] >> imageSize.height;
fs["Camera_Matrix"] >> cameraMatrix;
fs["Distortion_Coefficients"] >> distCoeffs;

}
static void ClickCoordinates(int event, int x, int y, int flags, void* )
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		
		cout << "LMB clicked: position (" << x << ", " << y << ")" << endl;
		int cor[1];
		pt.x = x;
		pt.y = y;
		new_Point = true;
		
	}
}
bool setupUART()
{
	DCB dcbParam;
		hCOM = CreateFile("COM5",
		GENERIC_WRITE | GENERIC_READ | GENERIC_EXECUTE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	if (!GetCommState(hCOM, &dcbParam)) cout<<"NO VAIABLE COM PORT"<<endl;
	dcbParam.BaudRate = CBR_9600; //9600 Baud
	dcbParam.ByteSize = 8; //8 data bits
	dcbParam.Parity = NOPARITY; //no parity
	dcbParam.StopBits = ONESTOPBIT; //1 stop
	if (!SetCommState(hCOM, &dcbParam))
		cout << "COM PORT ERROR" << endl;

	COMMTIMEOUTS timeout = { 0 };
	timeout.ReadIntervalTimeout = 50;
	timeout.ReadTotalTimeoutConstant = 50;
	timeout.ReadTotalTimeoutMultiplier = 50;
	timeout.WriteTotalTimeoutConstant = 50;
	timeout.WriteTotalTimeoutMultiplier = 10;

	SetCommTimeouts(hCOM, &timeout);

	return true;
}
bool Send_UartComm(char* data, int dat_len)
{
		DWORD byteswritten;
		bool retVal = WriteFile(hCOM, data, dat_len, &byteswritten, NULL);
	
	return retVal;
}
bool Read_UartComm()
{
	char data[18];
	DWORD bytesread;
	bool retVal = ReadFile(hCOM, data, 18, &bytesread, NULL);
	
	cout << "msg IN: ";
		for (int i = 0; i < 18; i++) cout << data[i];
	cout << endl;
	return retVal;
}
vector <double> calculate_markerVectors(vector<vector<cv::Point2f>> corners, cv::Point2d clickPt)
{

	vector <double> vec_AB(2);
	vec_AB[0] = corners[0][1].x - corners[0][0].x;		// vector AB [Bx-Ax,By-Ay]
	vec_AB[1] = corners[0][1].y - corners[0][0].y;
	vector <double> vec_BC(2);
	vec_BC[0] = corners[0][2].x - corners[0][1].x;		// vector BC [Cx-Bx,Cy-By]
	vec_BC[1] = corners[0][2].y - corners[0][1].y;
	vector <double> vec_AC(2);
	vec_AC[0] = corners[0][2].x - corners[0][0].x;		// vector AC [Cx-Ax,Cy-Ay]
	vec_AC[1] = corners[0][2].y - corners[0][0].y;

	cv::Point2f F;
	F.x = corners[0][1].x - (vec_AB[0] / 2);				//  F = B - (AB/2)
	F.y = corners[0][1].y - (vec_AB[1] / 2);
	cv::Point2f G;
	G.x = corners[0][2].x - (vec_BC[0] / 2);				//  G = C - (BC/2)
	G.y = corners[0][2].y - (vec_BC[1] / 2);
	cv::Point2f S;
	S.x = corners[0][2].x - (vec_AC[0] / 2);				//  S = C -(AC/2)
	S.y = corners[0][2].y - (vec_AC[1] / 2);

	vector <double> vec_SF(2);
	vec_SF[0] = F.x - S.x;
	vec_SF[1] = F.y - S.y;
	vector <double> vec_SG(2);
	vec_SG[0] = G.x - S.x;
	vec_SG[1] = G.y - S.y;
	vector <double> vec_ST(2);							// vector to target point
	vec_ST[0] = clickPt.x - S.x;
	vec_ST[1] = clickPt.y - S.y;

	double dot_prod_SF = vec_SF[0] * vec_ST[0] + vec_SF[1] * vec_ST[1];
	double dot_prod_SG = vec_SG[0] * vec_ST[0] + vec_SG[1] * vec_ST[1];
	double scalarSF = sqrt(vec_SF[0] * vec_SF[0] + vec_SF[1] * vec_SF[1]);
	double scalarSG = sqrt(vec_SG[0] * vec_SG[0] + vec_SG[1] * vec_SG[1]);
	double scalarST = sqrt(vec_ST[0] * vec_ST[0] + vec_ST[1] * vec_ST[1]);
	double alfa = (acos(dot_prod_SF / (scalarSF*scalarST)))* 180.0 / M_PI; // angle between vector pointing front of vehicle and vector to destination
	double beta = (acos(dot_prod_SG / (scalarSG*scalarST)))* 180.0 / M_PI; // auxiliary angle to determinate in which quarter point is placed in respect to vehicle front  
	double theta;

	if (alfa < 90 && beta < 90) theta = alfa;
	if (alfa < 90 && beta > 90) theta = -alfa;
	if (alfa > 90 && beta < 90) theta = alfa;
	if (alfa > 90 && beta > 90) theta = -alfa;
	vector <double> pos_img;
	pos_img.push_back(S.x);
	pos_img.push_back(S.y);
	pos_img.push_back(theta);
	
	return pos_img;			// [x center, y center, theta]
}
string control(vector <double> position)
{
	int v_valL = 90;
	int v_valR = 90;
	char control_msg[11];
	int deg = 10; // deegree dead zone

	if (position[2] >= deg || position[2] <= -deg) {

		if (position[2] > deg) {
			int ret = sprintf_s(control_msg, sizeof(control_msg), "L-%dR%de", v_valL, v_valR);		//turn left ; uart message to robot ; length 8  ex.(L50R-50e)
		}
		else if (position[2] < -deg) {
			int ret = sprintf_s(control_msg, sizeof(control_msg), "L%dR-%de", v_valL, v_valR);		//turn right ; uart message to robot ;  length 8  (L-50R50e)
		}
	}
	else {
		for (int n=0; n > 10; n++)stop();
		
		int ret = sprintf_s(control_msg, sizeof(control_msg), "L%dR%deX", v_valL , v_valR);		// length 7+1  (X)
	}
	
	string control_srt(control_msg);

	return control_srt;
}
void stop(){
	string msg = "L00R00eX";
	int len = msg.length();
	char *UART_msg = new char[len + 1];
	strcpy_s(UART_msg, 11, msg.c_str());
	Send_UartComm(UART_msg, len);
	cout << "msg OUT " << msg << endl;
}