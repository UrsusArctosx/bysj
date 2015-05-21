#ifndef _MAT2PCD_H
#define _MAT2PCD_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
using namespace std;
using namespace cv;
void saveImageAsClouds(Mat& point_clouds, string file_name);
void saveVectorAsClouds(vector<Point3f>& v, string file_name);
void saveVectorAsClouds(vector<Point3f>& v, vector<Vec3b>& color, string file_name);

#endif
