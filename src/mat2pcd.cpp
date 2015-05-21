#include <iostream>
#include <fstream>
#include <string>
#include <assert.h>
// Opencv
#include <opencv2/opencv.hpp>
// Point cloud library
#include <pcl/point_types.h>

#include "mat2pcd.h"

using namespace std;
using namespace cv;

/*
 * ofs: the filestream you want to write
 * height: the HEIGHT of pcd file
 * width: the WIDTH of pcd file
 */
void writePCDFileHeader(ofstream& ofs, int width, int height, bool rgb=true)
{
  // print pcd file header
  ofs << "# .PCD v.7 - Point Cloud Data file format\n";
  ofs << "VERSION .7\n";
  ofs << "FIELDS x y z";
  if (rgb)	{ofs << " rgb\n";} else {ofs << std::endl;}
  ofs << "SIZE 4 4 4";
  if (rgb)	{ofs << " 4\n";} else {ofs << std::endl;}
  ofs << "TYPE F F F";
  if (rgb)	{ofs << " F\n";} else {ofs << std::endl;}
  ofs << "COUNT 1 1 1";
  if (rgb)	{ofs << " 1\n";} else {ofs << std::endl;}

  ofs << "WIDTH " << width << std::endl;
  ofs << "HEIGHT " << height << std::endl;
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << width*height << std::endl;
  ofs << "DATA ascii\n";
}
/*
 * point_clouds: 3-channels image
 * file_name: the name of the pcd file
 */
void saveImageAsClouds(cv::Mat& point_clouds, string file_name="img.pcd")
{
  ofstream ofs(file_name);
  if (ofs.bad())
	{
	  std::cerr << "Error: cannot open file" << file_name << std::endl;
	  exit(1);
	}

  writePCDFileHeader(ofs, point_clouds.cols, point_clouds.rows);
  // print pcd file content
  for (int i=0; i<point_clouds.rows; i++)
	{
	  for (int j=0; j<point_clouds.cols; j++)
		{
		  Vec3b p = point_clouds.at<Vec3b>(i, j);
		  uint8_t r = (uint8_t)p[2];
		  uint8_t g = (uint8_t)p[1];
		  uint8_t b = (uint8_t)p[0];
		  uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
						  static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		  float c = *reinterpret_cast<float*>(&rgb);
		  ofs << j << " " << i << " 0 " << c << std::endl;
		}
	}
  ofs.close();
  // std::cout << "Saving image into pcd successfully..." << std::endl;
}
void saveVectorAsClouds(vector<Point3f>& v, string file_name="vec.pcd")
{
  ofstream ofs(file_name);
  if (ofs.bad())
	{
	  std::cerr << "Error: cannot open file" << file_name << std::endl;
	  exit(1);
	}

  writePCDFileHeader(ofs, v.size(), 1, false);
  for (int i=0; i<v.size(); i++)
	{
	  ofs << v[i].x << " " << v[i].y << " " << v[i].z << std::endl;
	}
  ofs.close();
  // std::cout << "Saving vector into pcd successfully..." << std::endl;
}
void saveVectorAsClouds(vector<Point3f>& v, vector<Vec3b>& color, string file_name="vec_color.pcd")
{
  assert(v.size()==color.size());

  ofstream ofs(file_name);
  if (ofs.bad())
	{
	  std::cerr << "Error: cannot open file" << file_name << std::endl;
	  exit(1);
	}

  writePCDFileHeader(ofs, v.size(), 1, true);
  for (int i=0; i<v.size(); i++)
	{
	  ofs << v[i].x << " " << v[i].y << " " << v[i].z << " ";

	  uint8_t r = (uint8_t)color[i][2];
	  uint8_t g = (uint8_t)color[i][1];
	  uint8_t b = (uint8_t)color[i][0];

	  uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
					  static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	  float c = *reinterpret_cast<float*>(&rgb);
	  ofs << c << std::endl;
	}
  ofs.close();
  // std::cout << "Saving vector with color into pcd successfully..." << std::endl;
}

void usage(char** argv)
{
  std::cout << "Save the inputted image into pcd file" << std::endl;
  std::cout << argv[0] << "/path/to/image" << "/path/to/pcd_file" << std::endl;
  std::cout << "/path/to/pcd_file: default file name is mat2pcd.pcd" << std::endl;
  exit(1);
}
// int main(int argc, char** argv)
// {
//   string file_name = "mat2pcd.pcd";
//   if (argc < 2)
// 	{
// 	  usage(argv);
// 	}
//   if (argc == 3)
// 	{
// 	  file_name = string(argv[2]);
// 	}
//   Mat img = imread(argv[1]);
//   imshow("img", img);
//   saveImageAsClouds(img, file_name);
//   waitKey();
//   return 0;
// }
