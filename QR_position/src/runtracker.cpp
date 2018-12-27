#include <algorithm>
#include <fstream>  //file function
#include <iostream>
#include <sstream>  //istringstream
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "aruco.hpp"  //QRcode
#include "kcftracker.hpp"

#include <dirent.h>

#include <time.h>
#include <ctime>
//#define CLC_Begin    clock_t start,finish; double totaltime;
// start=clock();//record time begin #define CLC_Finish   finish=clock();
// totaltime=(double)(finish-start) / 1000; cout<<"   It cost
//"<<totaltime<<"ms！"<<endl; //record time finish #define cb CLC_Begin #define
// cf CLC_Finish

// ROS Publisher
#include "QR_position/Num.h"
#include "ros/ros.h"

using namespace std;
using namespace cv;

#define xsize 640
#define ysize 480

// talker
float xtarget, ytarget;
// bool  OneFrameFinish = false;

#define data_type float
data_type find_max(vector<data_type> &a) {
  data_type max;
  max = a[0];
  for (int i = 1; i < a.size(); i++) {
    if (max < a[i]) max = a[i];
  }

  return max;
}

data_type find_min(vector<data_type> &a) {
  data_type min;
  min = a[0];
  for (int i = 1; i < a.size(); i++) {
    if (min >= a[i]) min = a[i];
  }

  return min;
}

float Pitch = 0;  // pitch y
float Roll = 0;   // roll x
float Yaw = 0;    // yaw z
// void attitudeCallback(const dji_sdk::AttitudeQuaternion::ConstPtr& msg)
// {
//           float q0 = msg->q0 ;
// 	  float q1 = msg->q1;
// 	  float q2 = msg->q2;
// 	  float q3 = msg->q3;
// 		Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch y
// 		Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2*
// q2
// + 1)* 57.3;	// roll x 		Yaw   = atan2(2*(q1*q2 +
// q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw z
// 	//std::cout << "x: " << Pitch << endl;
// 	//std::cout << "y: " << Roll << endl;
// 	//std::cout << "z: " << Yaw << endl;
// }

// solvePnP attitude solution
double world_x = 0, world_y = 0, world_z = 0;
void getpositionPnP(vector<Point2f> &imgP) {
  // camera Parameters
  double camD[9] = {411.8370, -0.2385, 308.4093, 0, 411.0792,
                    225.1734, 0,       0,        1};
  // k1,k2,p1,p2,k3...(Radial Tangential)
  double distCoeffD[4] = {-0.3256, 0.1073, -0.000052212, -0.0013};
  Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);
  Mat distortion_coefficients = Mat(4, 1, CV_64FC1, distCoeffD);
  vector<Point3f> objP;
  objP.push_back(Point3f(0, 0, 0));  //三维坐标的单位是毫米  //small one
  objP.push_back(Point3f(50, 0, 0));
  objP.push_back(Point3f(50, 50, 0));
  objP.push_back(Point3f(0, 50, 0));
  Mat objPM;
  Mat(objP).convertTo(objPM, CV_32F);

  Mat rvec = Mat(3, 1, CV_64FC1);
  Mat rotM = Mat(3, 3, CV_64FC1);
  Mat tvec = Mat(3, 1, CV_64FC1);

  // camera coordinate system
  Rodrigues(rotM, rvec);
  solvePnP(objPM, Mat(imgP), camera_matrix, distortion_coefficients, rvec,
           tvec);
  Rodrigues(rvec, rotM);

  // cout <<"rotation matrix: " <<endl<<rotM<<endl;
  // cout <<"translation matrix: " <<endl<<tvec.at<double>(0,0)<<"
  // "<<tvec.at<double>(1,0)<<" "<<tvec.at<double>(2,0)<<endl;

  // convert to world coordinate
  Mat Rt = rotM.t();
  Mat InvRT = Mat(4, 4, CV_64FC1);
  InvRT.at<double>(0, 0) = Rt.at<double>(0, 0);
  InvRT.at<double>(0, 1) = Rt.at<double>(0, 1);
  InvRT.at<double>(0, 2) = Rt.at<double>(0, 2);
  InvRT.at<double>(1, 0) = Rt.at<double>(1, 0);
  InvRT.at<double>(1, 1) = Rt.at<double>(1, 1);
  InvRT.at<double>(1, 2) = Rt.at<double>(1, 2);
  InvRT.at<double>(2, 0) = Rt.at<double>(2, 0);
  InvRT.at<double>(2, 1) = Rt.at<double>(2, 1);
  InvRT.at<double>(2, 2) = Rt.at<double>(2, 2);

  Mat RtT = -Rt * tvec;
  InvRT.at<double>(0, 3) = RtT.at<double>(0, 0);
  InvRT.at<double>(1, 3) = RtT.at<double>(1, 0);
  InvRT.at<double>(2, 3) = RtT.at<double>(2, 0);

  InvRT.at<double>(3, 0) = 0;
  InvRT.at<double>(3, 1) = 0;
  InvRT.at<double>(3, 2) = 0;
  InvRT.at<double>(3, 3) = 1;

  Mat worldcoordinate = Mat(4, 1, CV_64FC1);
  Mat cameracoordinate = Mat(4, 1, CV_64FC1);
  cameracoordinate.at<double>(0, 0) = 0;
  cameracoordinate.at<double>(1, 0) = 0;
  cameracoordinate.at<double>(2, 0) = 0;
  cameracoordinate.at<double>(3, 0) = 1;

  worldcoordinate = InvRT * cameracoordinate;
  world_x = worldcoordinate.at<double>(0, 0);
  world_y = worldcoordinate.at<double>(1, 0);
  world_z = worldcoordinate.at<double>(2, 0);
  // cout<<"position: "<<"\n"<< world_x <<" " << world_y <<" "<< world_z <<endl;
}

int main(int argc, char *argv[]) {
  // if (argc > 5) return -1;

  bool HOG = true;
  bool FIXEDWINDOW = false;
  bool MULTISCALE = true;
  bool SILENT = false;
  bool LAB = false;

  ros::init(argc, argv, "QR_position");  // node name
  ros::NodeHandle n, get_attitude;

  ros::Publisher chatter_pub = n.advertise<QR_position::Num>("imgmsg", 1);
  QR_position::Num msg;

  // Create KCFTracker object
  KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

  // Frame readed
  Mat frame;
  // Tracker results
  Rect result;

  // Camera
  // Read groundtruth for the 1st frame
  // float xMin = 290, yMin = 220, xMax = 390, yMax = 247;
  float xMin, yMin, xMax, yMax;

  // Using min and max of X and Y for groundtruth rectangle
  float width;   // = xMax - xMin;
  float height;  // = yMax - yMin;

  // Frame counter
  int nFrames = 0;
  int lostcounter = 0;
  // double cameras
  VideoCapture capture(0);

  if (!capture.isOpened()) {
    std::cout << "Can not open the vedio!" << endl;
    return -1;
  }
  // set images size
  capture.set(CV_CAP_PROP_FRAME_WIDTH, xsize);   //
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, ysize);  //

  // drop out the first several frame because some camera need to initiate.
  for (int i = 0; i < 10; i++) {
    Mat img_temp;
    capture.read(img_temp);
    waitKey(100);
  }
  cout << "camera ok!" << endl;

#define useQRcode 1
#if useQRcode
  int dictionaryId = 12;
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  // Mat markerImg;
  // aruco::drawMarker(dictionary, 7, 1000, markerImg);
  // imshow("marker", markerImg); //create marke

  const Ptr<aruco::DetectorParameters> &p_params =
      aruco::DetectorParameters::create();

  p_params->adaptiveThreshWinSizeMin = 7;
  p_params->adaptiveThreshWinSizeMax = 110;
  p_params->adaptiveThreshWinSizeStep = 50;

#endif

#define record 1
#if record
  ofstream file("test.txt", ios::app);
#endif

#define save 0
#if save
  cv::VideoWriter out;
  {
    Mat img_temp;
    capture[0].read(img_temp);
    out.open(
        "my_video_looking_down.avi",  //输出文件名
                                      // CV_FOURCC('D','I','V','X'), // MPEG-4
                                      // 编码 CV_FOURCC('D', 'I', 'V', '3'),
        CV_FOURCC('M', 'J', 'P', 'G'),
        30.0,  // 帧率 (FPS)
        img_temp.size(),
        true  // 只输入彩色图
    );
  }
#endif

  bool videostop = false;
  bool TargetGet = false;
  int count = 0;
  ros::Rate loop_rate(100);
  while (!videostop && ros::ok()) {
    // Read frame from the stream
    int readfailuretimes = 0;
    int maxfailuretimes = 13;
    // image get
    Mat img_pre;
    for (readfailuretimes = 0; readfailuretimes < maxfailuretimes;
         readfailuretimes++) {
      if (capture.read(img_pre)) break;
    }
    if (readfailuretimes >= maxfailuretimes) {
      videostop = true;
      break;
    }
    // resize the picture
    if (img_pre.rows > 600 && img_pre.cols > 600 && 0) {
      resize(img_pre, frame, Size(img_pre.cols / 4, img_pre.rows / 4), 0, 0,
             INTER_LINEAR);
    } else {
      frame = img_pre;
    }

    // get the attitude of camera
    ros::spinOnce();  // if you were to add a subscription into this
                      // application, and did not have ros::spinOnce() here,
                      // your callbacks would never get called.

// Detection period
#define redetction \
  0  //25 \
													 //camera0
    if ((nFrames >= redetction) ||
        (!TargetGet))  // reinit kcf after ... frames.
    {
      vector<vector<Point2f> > corners;
      vector<int> ids;
      aruco::detectMarkers(frame, dictionary, corners, ids, p_params);

      //   aruco::drawDetectedMarkers(frame, corners);

      if (ids.size() > 0)  // find ids.size() QRcode Picture
      {
        nFrames = 0;

        // only use the first picture.
        float ax[4] = {corners[0][0].x, corners[0][1].x, corners[0][2].x,
                       corners[0][3].x};
        float ay[4] = {corners[0][0].y, corners[0][1].y, corners[0][2].y,
                       corners[0][3].y};
        vector<float> xarr(ax, ax + 4);
        vector<float> yarr(ay, ay + 4);

        xMin = find_min(xarr);
        yMin = find_min(yarr);
        xMax = find_max(xarr);
        yMax = find_max(yarr);

        width = xMax - xMin;
        height = yMax - yMin;

        vector<Point2f> points;
        points.push_back(Point2f(corners[0][0].x, corners[0][0].y));
        points.push_back(Point2f(corners[0][1].x, corners[0][1].y));
        points.push_back(Point2f(corners[0][2].x, corners[0][2].y));
        points.push_back(Point2f(corners[0][3].x, corners[0][3].y));
        getpositionPnP(points);

        TargetGet = true;
        lostcounter = 0;
      } else  // can not detect the target.  kcf use the former messgaes.
      {
        // TargetGet = false;
        nFrames = 1;
        lostcounter++;
      }
    }

    // if find, tracking
    if (TargetGet && (lostcounter < 30)) {
      // First frame, give the groundtruth to the tracker
      if (nFrames == 0) {
        tracker.init(Rect(xMin, yMin, width, height), frame);
        xtarget = xMin + 0.5 * width;
        ytarget = yMin + 0.5 * height;
      }
      // Update
      else {
        result = tracker.update(frame);
        if ((result.x < 0) || (result.y < 0) ||
            ((result.x + result.width) > xsize) ||
            ((result.y + result.height) > ysize)) {
          cout << "x or y is out of boundary of the image.";
          // break;
        }
        xtarget = result.x + 0.5 * result.width;
        ytarget = result.y + 0.5 * result.height;
      }
      if (!SILENT) {
        if (nFrames == 0) {
          rectangle(frame, Point(xMin, yMin),
                    Point(xMin + width, yMin + height), Scalar(0, 0, 255), 1,
                    8);
        } else {
          rectangle(frame, Point(result.x, result.y),
                    Point(result.x + result.width, result.y + result.height),
                    Scalar(0, 0, 255), 1, 8);
        }
      }

      // Publish out
      msg.camera_n = 0;
      msg.x = world_x;
      msg.y = world_y;
      msg.z = world_z;
      cout << "position: "
           << "\n"
           << world_x << " " << world_y << " " << world_z << endl;

      ++count;
      ROS_INFO("sent: camra%d [%f %f %f] %d ", msg.camera_n, msg.x, msg.y,
               msg.z, count);
      chatter_pub.publish(msg);
      ros::spinOnce();
      // loop_rate.sleep();

#define show 1
#if show  //show                                                           \
		 //  cout << "x:" << result.x << " " << "y:" << result.y << endl; \
		 //  cout << "width:" << result.width << " " << "height:" << result.height << endl;
#if record
      file << world_x << world_y << world_z << endl;
      file << endl;
#endif

#endif

      nFrames++;
    }

    // if not find at first. Assume that it won't lost the target
    // if ((!(TargetGet)) || lostcounter == 1000) {
    if (lostcounter >= 30) {
      msg.camera_n = -2;  // not find number
      msg.x = 0;
      msg.y = 0;
      msg.z = 0;
      ++count;
      ROS_INFO("sent: not find %d [%f %f %f] %d ", msg.camera_n, msg.x, msg.y,
               msg.z, count);
      chatter_pub.publish(
          msg);  // broadcast the message to anyone who is connected.
    }

#if show
    imshow("Image looking down", frame);
    // imshow("Image strbismus", frame1);
#if save
    out << frame;
    //  out1 << frame1;
#endif
    // waitKey(1);
    int key2 = waitKey(10);
    if (key2 == 'S' || key2 == 's') {
      while (1) {
        int key = waitKey(10);
        if (key == 'C' || key == 'c') {
          break;
        }
      }
    }
    if (key2 == 'Q' || key2 == 'q') {
      break;
    }
#endif
  }

#if record
  file.close();  //关闭文件
#endif

  // Publish out
  msg.camera_n = -1;
  msg.x = -1;
  msg.y = -1;
  msg.z = -1;

  ++count;
  ROS_INFO("sent: camra%d [%f %f %f] %d ", msg.camera_n, msg.x, msg.y, msg.z,
           count);
  chatter_pub.publish(msg);
  ros::spinOnce();
  sleep(1);

  return 0;
}
