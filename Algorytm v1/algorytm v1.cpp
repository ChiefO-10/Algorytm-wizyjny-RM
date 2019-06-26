

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


int calibration();
int prog();
void video_capture(cv::VideoCapture& videocapture, cv::Mat& frame);
void video_capture_fast(cv::VideoCapture& videocapture, cv::Mat& frame);
static void saveCameraParams(string& filename, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
static void readCameraParams(string& filename, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
void getEulerAngles(cv::Mat &rotCamerMatrix, cv::Vec3d &eulerAngles);
static void ClickCoordinates(int event, int x, int y, int flags, void* );
void worldPoint(cv::Mat rotationMatrix, cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat intrinsics, cv::Point3d realPoint, cv::Point3d worldPoint);
cv::Point3d pt(-1,1,1);
cv::Point3d wPoint;
bool new_Point = false;

int main() 
{
	
	for (int opt = 0; opt == 0;) {

		int option;
		cout << "Chose action:" << endl << "1. Calibrate" << endl << "2. Run" << endl << "3. Close" << endl;
		cin >> option;

		switch (option)
		{
		case 1:
			calibration();
			break;
		case 2:
			prog();
			break;
		case 3:
			opt++;
			break;
		default:
			cout << "Wrong action" << endl;
		}
	}
	
	return(0);
}



int calibration()
{

	cv::VideoCapture capCalib;
	cv::Mat frameCalib;

	bool proceed = false;

	capCalib.open("http://192.168.43.1:8080/video");				//pobranie obrazu z IPwebcam

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
	
	char out_msg[10];							//uart message to robot; max lenght 11  (L - 100R - 100e)
	
	const cv::Ptr<cv::aruco::Dictionary> markDict = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);  //stworzenie obiektu dictionary bedacego markerem gotowego zestawu?

	cv::VideoCapture capIP;
	capIP.open("http://192.168.43.1:8080//video");				//pobranie obrazu z IPwebcam

	if (!capIP.isOpened()) {                                  // if unable to open image
		cout << "error: video stream unavailable"<<endl;     // show error message on command line
		_getch();                                               // may have to modify this line if not using Windows
		return(0);                                              // and exit program
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
		vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
		//const cv::Ptr<cv::aruco::DetectorParameters> DetePara;

		
		cv::aruco::detectMarkers(frame, markDict, markerCorners, markerIds);

		if (markerIds.size() > 0) {
			
			//bool cv::solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec,
				//bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE);
			
			cv::Scalar markObjPoints;
			cv::aruco::estimatePoseSingleMarkers(markerCorners, 50, cameraMatrix, distCoeffs, temp_rvecs, temp_tvecs);
			cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
			for (int i = 0; i < markerIds.size(); i++)
			cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, temp_rvecs[i], temp_tvecs[i], 2);

						
			cv::Vec3d rodri_rvec = temp_rvecs[0];

			cv::Mat rodri_tvec = cv::Mat (temp_tvecs[0]);

			cv::Rodrigues(rodri_rvec, rot_matrix);

			cv::Mat rot_matrix_t = rot_matrix.t();
			
			//cv::Mat rodri_tvec2 = -rot_matrix_t * rodri_tvec.t();			// X,Y,Z coordinates of marker in camera coordianate system
			
			double * tmp = rot_matrix_t.ptr<double>(0);
			cv::Point3f camWorld(tmp[0],tmp[1], tmp[2]);
			
			cv::Point3f tvec_3f(temp_tvecs[0][0], temp_tvecs[0][1], temp_tvecs[0][2]);
			


			cv::Vec3d tvec ((double*)rodri_tvec.data);
			// TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TESTTEST TEST TESTTEST TEST TESTTEST TEST TEST
			cv::Mat points2d;
			

			//cv::Vec3d eulerAngles;
			//getEulerAngles(rot_matrix, eulerAngles);
			//cv::Point2d yaw(eulerAngles[1]);			// point keeping marker vector/X,Y axis rotation @ Rodrigues			
			
			//worldPoint(rot_matrix, rodri_rvec, rodri_tvec, cameraMatrix, pt, wPoint);

			cout <<"POINTCAM" << pt.x << "," << pt.y << endl;
			
			cout << "MARKER POSE"<< temp_tvecs[0][0] << "," << temp_tvecs[0][0] << endl;
			//cout << eulerAngles[1] << endl;

			// TEST TEST TESTTEST TEST TESTTEST TEST TESTTEST TEST TESTTEST TEST TESTTEST TEST TEST
		}



		cv::namedWindow("Video stream", CV_WINDOW_AUTOSIZE);		//okno wyswietlania
		cv::imshow("Video stream", frame);						//wyswietlanie obrazu

		
		cv::setMouseCallback("Video stream", ClickCoordinates, NULL);
		if (new_Point == true) {
			cout << "pt =" << pt.x << "," << pt.y << endl;

			new_Point = false;
		}
		cv::waitKey(1);
		
	}

	
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
	int i;
	for (i = 0; i < 5; i++) {
		bool bFrameread_1;
		bool bFrameRead_1 = videocapture.grab();				//wczytanie nowej klatki

		if (!bFrameRead_1) {
			cout << " error: Frame read failed" << endl;
		}


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
bool Send_UartComm(char* data, int dat_len)
{
		DCB dcbParam;
		HANDLE hCOM = CreateFile(	"COM5",
									GENERIC_WRITE,
									0,
									NULL,
									OPEN_EXISTING,
									0,
									NULL	);

		if (!GetCommState(hCOM, &dcbParam))
			return false;
		dcbParam.BaudRate = CBR_9600; //9600 Baud
		dcbParam.ByteSize = 8; //8 data bits
		dcbParam.Parity = NOPARITY; //no parity
		dcbParam.StopBits = ONESTOPBIT; //1 stop
		if (!SetCommState(hCOM, &dcbParam))
			return false;	

		DWORD byteswritten;
		bool retVal = WriteFile(hCOM, data, dat_len, &byteswritten, NULL);
		CloseHandle(hCOM); //close the handle
			return retVal;
}
void calculate_prototype(cv::Point2f position, cv::Point destination, float& distance_x, float& distance_y, float& angle)
{
	distance_x = destination.x - position.x;
	distance_y = destination.x - position.x;
	angle = atan2(distance_x, distance_y)*180/ M_PI;

}
bool linear_control(cv::Point2f position, cv::Point destination,char* msg)
{	
	int v_val=50;

	char msg_buff[10];

	float anglR=0;                                          // robot facing vector angle
	float dist_x, dist_y, anglD, angl_diff;
	calculate_prototype(position, destination, dist_x, dist_y, anglD);
	angl_diff = anglD - anglR ;
	if (angl_diff >= 3 || angl_diff <= -3){
		
		if (angl_diff >= 3) {

			int ret = sprintf_s(msg_buff,sizeof(msg_buff), "L%dR-%de", v_val,v_val);		//turn left ; uart message to robot ; max lenght 11  ex.(L-100R-100e)
			msg = msg_buff;
		}
		if (angl_diff <= -3) {
			
			int ret = sprintf_s(msg_buff, sizeof(msg_buff), "L-%dR%de", v_val, v_val);				//turn right ; uart message to robot ; max lenght 11  (L-100R-100e)
			msg = msg_buff;
		}
	}
	else {
		sprintf_s(msg_buff, "L%dR%de", v_val, v_val);
		msg = msg_buff;
	}
	return true;
}

void getEulerAngles(cv::Mat &rotCamerMatrix, cv::Vec3d &eulerAngles) {

	cv::Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
	double* _r = rotCamerMatrix.ptr<double>();
	double projMatrix[12] = { _r[0],_r[1],_r[2],0,
						  _r[3],_r[4],_r[5],0,
						  _r[6],_r[7],_r[8],0 };

	decomposeProjectionMatrix(cv::Mat(3, 4, CV_64FC1, projMatrix),
		cameraMatrix,
		rotMatrix,
		transVect,
		rotMatrixX,
		rotMatrixY,
		rotMatrixZ,
		eulerAngles);
}
void worldPoint(cv::Mat rotationMatrix, cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat intrinsics, cv::Point3d imgPoint, cv::Point3d worldPoint)
{
	cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1

// image point
	uvPoint.at<double>(0, 0) = imgPoint.x; //got this point using mouse callback
	uvPoint.at<double>(1, 0) = imgPoint.y;
	
	cv::Mat tempMat, tempMat2;
	double s, zConst = 0;
	tempMat = rotationMatrix.inv() * intrinsics.inv() * uvPoint;
	tempMat2 = rotationMatrix.inv() * cv::Mat(tvec);
	s = zConst + tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);
	cv::Mat wcPoint = rotationMatrix.inv() * (s * intrinsics.inv() * uvPoint - tvec);

	cv::Point3f Point(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0)); // point in world coordinates
	
	worldPoint.x = wcPoint.at<double>(0, 0);
	worldPoint.y = wcPoint.at<double>(1, 0);
	worldPoint.z = wcPoint.at<double>(2, 0);
	cout << "POINTWRLD:"<< Point.x << "," << Point.y << endl;
}

//void pid(float Kp, float Ki, float Kd, float d_t, float destPoint, float actPoint, double max_I, double min_I) 
//{	
//	double integr;
//	double error_prev;
//
//	double error = destPoint - actPoint;
//	
//	double P = Kp * error;
//
//	integr += error * d_t;
//	double I = Ki * integr;
//
//	double deriv = (error - error_prev) / d_t;
//	double D = Kd * deriv;
//	
//	if (integr > max_I) integr = max_I;
//	else if (integr < min_I) integr = min_I;
//
//}
