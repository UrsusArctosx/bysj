/**
 * ocr.cpp:
 * Read the digits from a scratchcard. See the tutorial at
 * http://opencv-code.com/tutorials/how-to-read-the-digits-from-a-scratchcard
 *
 * Compile with:
 * g++ -I/usr/local/include -L/usr/local/lib ocr.cpp -o ocr \
 *       -lopencv_core -lopencv_imgproc -lopencv_highgui -ltesseract
 */
#include "get_speed.h"
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <assert.h>
using namespace std;
using namespace cv;

#define _DEBUG_ON 0

void getYellow(Mat& src, Mat& dst)
{
  for (int i=0; i<src.rows; i++)
	{
	  for (int j=0; j<src.cols; j++)
		{
		  Vec3b p = src.at<Vec3b>(i, j);
		  uchar r = p[2];
		  uchar g = p[1];
		  uchar b = p[0];
		  if (r>200 && g>200 && b<30)
			dst.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
		  else
			dst.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
		}
	}
}

int detectSpeed(Mat& src, char** speed)
{
  int x1 = 980;
  int y1 = 1010;
  int x2 = 1030;
  int y2 = 1060;
  int w = x2 - x1;
  int h = y2 - y1;
  Rect region_of_interest = Rect(x1, y1, w, h);
  Mat image_roi = src(region_of_interest);

  Mat im1(image_roi.size(), image_roi.type());
  getYellow(image_roi, im1);

  Mat im2;
  cvtColor(im1, im2, CV_BGR2GRAY);

  tesseract::TessBaseAPI tess;
  tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
  tess.SetVariable("tessedit_char_whitelist", "0123456789");
  tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
  tess.SetImage((uchar*)im2.data, im2.cols, im2.rows, 1, im2.cols);

  *speed = tess.GetUTF8Text();

  char str[255] = {0};
  int j = 0; 
  for (int i=0; i<strlen(*speed); i++)
	{
	  if ((*speed)[i]>='0' && (*speed)[i])
		str[j++] = (*speed)[i];
	}

#if _DEBUG_ON
  cout << "speed: " << str << endl;

  imshow("debug", im1);
  waitKey();
#endif

  // return atoi(*speed);
  return atoi(str);
}

// int main(int argc, char** argv)
// {
//   ifstream ifs("test_speed.dat");
//   if (ifs.bad())
// 	{
// 	  cerr << "Could not open the file" << endl;
// 	  exit(1);
// 	}
  
//   string file_name;
//   int standard_speed;
//   string dir = "/Volumes/chenzongxiong/bysj/test_speed/";
//   unsigned int right_count = 0;
//   int idx = 0;
//   while (!ifs.eof())
// 	{
// 	  ifs >> file_name >> standard_speed;
// 	  #if _DEBUG_ON
// 	  // if (file_name!="4517.png")
// 	  // continue;
// 	  if (argc < 2)
// 		{cerr << "The debug image not input" << endl; exit(1);}
// 	  file_name = argv[1];
// 	  file_name += ".png";
// 	  standard_speed = atoi(argv[2]);
// 	  #endif
// 	  string file_path = dir+file_name;
	  
// 	  Mat im0 = imread(file_path);
// 	  if (!im0.data)
// 	  	{
// 	  	  cerr << "Could not read the image " << file_path << endl;
// 	  	  exit(1);
// 	  	}
// 	  char* speed = NULL;
// 	  int predict_speed = detectSpeed(im0, &speed);
// 	  // assert(standard_speed==predict_speed);
// 	  if (predict_speed!=standard_speed)
// 		{
// 		  cout << "#" << file_name << " ";
// 		  cout << "predict_speed: " << predict_speed <<
// 			"<------> standard_speed: " << standard_speed;
	  
// 		  cout << "----> Wrong" << endl;
// 		  #if _DEBUG_ON
// 		  break;
// 		  #endif
// 		  // imshow("Wrong", im0);
// 		  // waitKey();
// 		}
// 	  else
// 	  	{
// 	  	  right_count ++;
// 		  #if _DEBUG_ON
// 		  break;
// 		  #endif
// 	  	}
// 	}  
//   cout << "Accuracy: ";
//   cout << ((float)right_count)/100 << endl;
//   return 0;
// }
