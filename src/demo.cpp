/*
  Copyright 2012. All rights reserved.
  Institute of Measurement and Control Systems
  Karlsruhe Institute of Technology, Germany
  This file is part of libviso2.
  Authors: Andreas Geiger

  libviso2 is free software; you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation; either version 2 of the License, or any later version.

  libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
  PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with
  libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
  Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "viso_mono.h"
#include "reconstruction.h"
#include "mat2pcd.h"
#include "get_speed.h"
#include <libgen.h>
#include <dirent.h>
#include <cstdlib>
#include "elas.h"
#include <sstream>

using namespace std;
using namespace cv;

void save3dPointsAsPCD(vector<Reconstruction::point3d>& p, string filename) {
  vector<Point3f> pt3d;
  for (int i = 0; i < p.size(); i++) {
    Point3f pt;
    pt.x = p[i].x;
    pt.y = p[i].y;
    pt.z = p[i].z;
    pt3d.push_back(pt);
  }
  saveVectorAsClouds(pt3d, filename);
}
void drawMatched(vector<Matcher::p_match>& p_matched, Mat &img) {
  for (int i = 0; i < p_matched.size(); i++) {
    int u1 = p_matched[i].u1p;
    int v1 = p_matched[i].v1p;
    int i1 = p_matched[i].i1p;
    int u2 = p_matched[i].u1c;
    int v2 = p_matched[i].v1c;
    int i2 = p_matched[i].i1c;
    line(img, Point(u1, v1), Point(u2, v2), Scalar(255, 255, 255), 1);
    circle(img, Point(u1, v1), 1, Scalar(255, 0, 0), -1);
    circle(img, Point(u2, v2), 1, Scalar(0, 255, 0), -1);
  }
}
void clearNearbyOpticalflow(Mat* frame, int y)
{
  for (int i=y; i<frame->rows; i++)
	{
	  for (int j=0; j<frame->cols; j++)
		{
      if (frame->channels() == 3) {
        Vec3b rgb = Vec3b(0, 0, 0);
        frame->at<Vec3b>(i, j) = rgb;
      } else if (frame->channels() == 1) {
        frame->at<char>(i, j) = 0;
      }
		}
	}
}

void help () {
  cout << "./viso video_name pcd_saved_dir scale pitch start_frame" << endl;
}

// compute disparities of pgm image input pair file_1, file_2
void getDisparity(Mat& left_img, Mat& right_img, Mat& left_img_disp8, Mat& right_img_disp8)
{
  // cout << "Processing: " << file_1 << "\t" << file_2 << endl;
  // load images
  // image<uchar> *I1,*I2;
  // I1 = loadPGM(file_1);
  // I2 = loadPGM(file_2);
  // // check for correct size
  // if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
  //     I1->width()!=I2->width() || I1->height()!=I2->height()) {
  //   cout << "ERROR: Images must be of same size, but" << endl;
  //   cout << "       I1: " << I1->width() <<  " x " << I1->height() <<
  //                ", I2: " << I2->width() <<  " x " << I2->height() << endl;
  //   delete I1;
  //   delete I2;
  //   return;
  // }
  // get image width and height
  // int32_t width  = I1->width();
  // int32_t height = I1->height();
  int width = left_img.cols;
  int height = right_img.rows;

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(left_img.data,right_img.data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  // float disp_max = 0;
  // for (int32_t i=0; i<width*height; i++) {
  //   if (D1_data[i]>disp_max) disp_max = D1_data[i];
  //   if (D2_data[i]>disp_max) disp_max = D2_data[i];
  // }

  // copy float to uchar
  // image<uchar> *D1 = new image<uchar>(width,height);
  // image<uchar> *D2 = new image<uchar>(width,height);
  // for (int32_t i=0; i<width*height; i++) {
  //   D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
  //   D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
  // }
  Mat left_img_disp(height, width, CV_32FC1, D1_data);
  Mat right_img_disp(height, width, CV_32FC1, D2_data);
  // float max_val, min_val;
  // Mat left_img_disp8, right_img_disp8;
  // minMaxLoc(left_img_disp, &min_val, &max_val);
  normalize(left_img_disp, left_img_disp8, 0, 255, CV_MINMAX, CV_8U);
  normalize(right_img_disp, right_img_disp8, 0, 255, CV_MINMAX, CV_8U);

  // save disparity images
  // char output_1[1024];
  // char output_2[1024];

  // strncpy(output_1,file_1,strlen(file_1)-4);
  // strncpy(output_2,file_2,strlen(file_2)-4);
  // output_1[strlen(file_1)-4] = '\0';
  // output_2[strlen(file_2)-4] = '\0';
  // strcat(output_1,"_disp.png");
  // strcat(output_2,"_disp.png");
  // imwrite(output_1, left_img_disp);
  // imwrite(output_2, right_img_disp);
  // savePGM(D1,output_1);
  // savePGM(D2,output_2);

  // free memory
  // delete I1;
  // delete I2;
  // delete D1;
  // delete D2;
  free(D1_data);
  free(D2_data);
}

int main (int argc, char** argv) {

  // Mat img1 = imread(argv[1], 0);
  // Mat img2 = imread(argv[2], 0);
  // resize(img1, img1, Size(0.5*img1.cols, 0.5*img1.rows));
  // resize(img2, img2, Size(0.5*img2.cols, 0.5*img2.rows));
  // clearNearbyOpticalflow(&img1, 420);
  // clearNearbyOpticalflow(&img2, 420);
  // cout << "hello world" << endl;
  // Mat img1_disp, img2_disp;
  // getDisparity(img1, img2, img1_disp, img2_disp);
  // imshow ("img1", img1);
  // imshow("img1 disp", img1_disp);
  // imshow("img2 disp", img2_disp);
  // string im1_disp = argv[1];
  // im1_disp = im1_disp.substr(0, im1_disp.length()-4)+ "_disp.png";
  // imwrite(im1_disp, img1_disp);

  // waitKey(0);
  // return 0;

  help();
  ofstream ofs("log.txt");
  ofstream ofs_rt;
  ofstream ofs_x;
  ofstream ofs_y;
  ofstream ofs_z;

  if (ofs.bad()) {
    cerr << "file cannot open" << endl;
    exit(1);
  }
  // sequence directory
  bool save_flag = false;
  if (argc>=3) {
    save_flag = true;
  }
  VideoCapture cap;
  char* in_filepath = argv[1];
  char* bn = basename(in_filepath);

  cap.open(in_filepath);
  if (!cap.isOpened()) {
    cout << "Could not initialize capturing...\n";
    exit(1);
  }

  float scale = 1.0;
  if (argc >= 4) {
    scale = atof(argv[3]);
  }
  float pitch = -0.08;
  if (argc >= 5) {
    pitch = atof(argv[4]);
  }
  int start_frame = 1;
  if (argc >= 6) {
    start_frame = atoi(argv[5]);

  }
  string base_dir;
  if (save_flag) {
    base_dir = argv[2];
    if (base_dir[base_dir.length()-1]=='/')
      base_dir += bn;
    else
      base_dir = base_dir+'/'+bn;
    stringstream ss;
    ss << "_" << scale << "_" << pitch;
    base_dir = base_dir.substr(0, base_dir.length()-4)+ss.str();
    cout << base_dir << endl;

    string cmd = "mkdir -p " + base_dir;
    system(cmd.c_str());

    ofs_x.open(base_dir+"/"+"x.txt");
    ofs_y.open(base_dir+"/"+"y.txt");
    ofs_z.open(base_dir+"/"+"z.txt");
    ofs_rt.open(base_dir+"/"+"rt.txt");
    base_dir += "/frame_";
  }

  char numstr[256] = {0};
  int frame_index = 0;
  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_mono.h
  VisualOdometryMono::parameters param;

  param.calib.f = 733.2/scale;
  param.calib.cu = 444.261*scale;
  param.calib.cv = 277.314*scale;
  param.height = 1.6;
  // param.pitch  = -0.08;

  param.pitch = pitch;
  param.bucket.max_features = 1000;
  int32_t first_frame = 0;
  int32_t last_frame = 8800;
  int32_t step = 3;
  // init visual odometry
  VisualOdometryMono viso(param);

  Reconstruction re3d;
  re3d.setCalibration(param.calib.f, param.calib.cu, param.calib.cv);

  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);

  // loop through all frames
  // for (int32_t i=first_frame; i < last_frame; i+=step) {
  Mat gray_frame, optical_flow;
  Mat left_img, right_img;
  Mat left_img_disp8, right_img_disp8;

  const int k_factor = 23;
  int SKIPPED_FRAMES = 2;
  int velocity_world;
  cap.set(CV_CAP_PROP_POS_FRAMES, start_frame);
  // intrinsic param
  // Mat camera_matrix, dist_coeffs, remap_x, remap_y;

  // FileStorage fs_in;
  // fs_in.open("calibration.yml", FileStorage::READ);
  // fs_in["camera matrix"] >> camera_matrix;
  // fs_in["distorted coeffs"] >> dist_coeffs;
  // cout << camera_matrix << endl;
  // cout << dist_coeffs << endl;
  cout << scale << "\t" << pitch << "\t" << start_frame << endl;

  Mat r1, r2, t1, t2;
  Mat prev_r(3, 1, CV_64FC1);
  Mat prev_prev_r(3, 1, CV_64FC1);
  Mat prev_t(3, 1, CV_64FC1);
  Mat prev_prev_t(3, 1, CV_64FC1);
  double kkk = 0.7;
  int counter = 0;
  Mat frame;
  for ( ;; ) {
    cap >> frame;
    if (frame.empty())
      return 0;
    char* speed = NULL;
    velocity_world = detectSpeed(frame, &speed) / 3.6f; // m/s
    if (velocity_world>0) {
      SKIPPED_FRAMES = k_factor/velocity_world;
      if (SKIPPED_FRAMES < 2)
        SKIPPED_FRAMES = 2;
      else if (SKIPPED_FRAMES > 15)
        SKIPPED_FRAMES = 15;
    }
    else
      SKIPPED_FRAMES = 15;

    for (int i = 1; i < SKIPPED_FRAMES; i++) {
      cap >> frame;
      if (frame.empty())
        return 0;
    }

    cout << "speed: " << velocity_world << endl;
    unsigned int curr_frame = (unsigned int)cap.get(CV_CAP_PROP_POS_FRAMES);
    // status
    cout << "Processing: Frame: " << curr_frame << endl;// cap.get(CV_CAP_PROP_POS_FRAMES) << endl;

    // catch image read/write errors here
    resize(frame, frame, Size(0.5*frame.cols, 0.5*frame.rows));
    // undistorted image

    clearNearbyOpticalflow(&frame, 420);
    cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
    int width = gray_frame.cols;
    int height = gray_frame.rows;
    // convert input images to uint8_t buffer
    uint8_t* gray_frame_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    int32_t k=0;
    for (int32_t v=0; v<height; v++) {
      for (int32_t u=0; u<width; u++) {
        gray_frame_data[k++] = gray_frame.at<char>(v, u);
      }
    }

    vector<Matcher::p_match> p_matched = viso.getMatches();
    // compute visual odometry
    int32_t dims[] = {width,height,width};
    if (viso.process(gray_frame_data,dims)) {
      // on success, update current pose
      // Matrix pose_tmp = Matrix::eye(4);
      // pose_tmp = pose * Matrix::inv(viso.getMotion());
      // float fl_pre[16] = {0};
      // float fl_cur[16] = {0};
      // for (int i = 0; i < 4; i ++ )
      //   for (int j = 0; j < 4; j ++) {
      //     fl_pre[i*4+j] = pose.val[i][j];
      //     fl_cur[i*4+j] = pose_tmp.val[i][j];
      //   }
      // Mat Rt1 = Mat(4, 4, CV_32FC1, &fl_pre);
      // Mat Rt2 = Mat(4, 4, CV_32FC1, &fl_cur);
      // Mat R1 = Mat(Rt1, Range(0, 3), Range(0, 3));
      // Mat R2 = Mat(Rt2, Range(0, 3), Range(0, 3));
      // Mat Rx1, Rxx1, Rx2, Rxx2;
      // Rodrigues(R1, Rx1);
      // Rodrigues(R2, Rx2);
      // if (fabs(Rx1.at<float>(0, 0) - Rx2.at<float>(0, 0)) > 0.02) {
      //   Rx2.at<float>(0, 0) = param.pitch;
      // }
      // Rodrigues(Rx2, Rxx2);
      // double dl[16] = {0};
      // for (int i=0; i<3; i ++) {
      //   for (int j=0; j<3; j ++) {
      //     dl[i*4+j] = Rxx2.at<float>(i, j);
      //   }
      // }
      // dl[3] = fl_cur[3];      dl[7] = fl_cur[7];
      // dl[11] = fl_cur[11];      dl[15] = 1;
      // dl[12] = dl[13] = dl[14] = 0;
      // Matrix refine_RT = Matrix(4, 4, dl);
      // pose = refine_RT;
      counter ++;
      if (counter > 3) {
        Mat r_predict(3, 1, CV_64FC1);
        r_predict.at<double>(0, 0) = prev_r.at<double>(0, 0)*2-prev_prev_r.at<double>(0, 0);
        r_predict.at<double>(1, 0) = prev_r.at<double>(1, 0)*2-prev_prev_r.at<double>(1, 0);
        r_predict.at<double>(2, 0) = prev_r.at<double>(2, 0)*2-prev_prev_r.at<double>(2, 0);
        cout << "r_predict: \n" << r_predict << endl;

        Matrix pose_calc = pose * Matrix::inv(viso.getMotion());
        double dl[16] = {0};
        for (int i=0; i<4; ++ i)
          for (int j=0; j<4; ++ j) {
            dl[4*i+j] = pose_calc.val[i][j];
          }
        Mat pose_calc_mat(4, 4, CV_64FC1, dl);
        cout << "pose: \n" << pose << endl;
        cout << "pose_calc_mat: \n" << pose_calc_mat << endl;

        Mat r_calc = Mat(pose_calc_mat, Range(0, 3), Range(0, 3));
        Mat r_degree(3, 1, CV_64FC1);
        Rodrigues(r_calc, r_degree);
        cout << "r_calc: \n" << r_degree << endl;
        r_degree.at<double>(0, 0) = kkk*r_degree.at<double>(0, 0) + (1.0-kkk)*r_predict.at<double>(0, 0);
        r_degree.at<double>(1, 0) = kkk*r_degree.at<double>(1, 0) + (1-kkk)*r_predict.at<double>(1, 0);
        r_degree.at<double>(2, 0) = kkk*r_degree.at<double>(2, 0) + (1.0-kkk)*r_predict.at<double>(2, 0);
        cout << "r update: \n" << r_degree << endl;
        cout << "pose: \n" << pose << endl;
        cout << "pose_calc: \n" << pose_calc << endl;
        Rodrigues(r_degree, r_calc);

        // Mat t_predict(3, 1, CV_64FC1);
        Mat t_calc = Mat(pose_calc_mat, Range(0, 3), Range(3, 4));
        // cout << "prev_t: \n" << prev_t << "\nprev_prev_t: \n" << prev_prev_t << endl;
        // t_predict.at<double>(0, 0) = prev_t.at<double>(0, 0)*2-prev_prev_t.at<double>(0, 0);
        // t_predict.at<double>(1, 0) = prev_t.at<double>(1, 0)*2-prev_prev_t.at<double>(1, 0);
        // t_predict.at<double>(2, 0) = prev_t.at<double>(2, 0)*2-prev_prev_t.at<double>(2, 0);
        // cout << "t_predict: \n" << t_predict << endl;

        // cout << "t_calc: \n" << t_calc << endl;
        // t_calc.at<double>(0, 0) = kkk*t_calc.at<double>(0, 0) + (1-kkk)*t_predict.at<double>(0, 0);
        // t_calc.at<double>(1, 0) = kkk*t_calc.at<double>(1, 0) + (1-kkk)*t_predict.at<double>(1, 0);
        // t_calc.at<double>(2, 0) = kkk*t_calc.at<double>(2, 0) + (1-kkk)*t_predict.at<double>(2, 0);
        // cout << "t update: \n" << t_calc << endl;

        for (int i = 0; i < 3; i ++ ) {
          for (int j = 0; j < 3; j ++) {
            pose.val[i][j] = r_calc.at<double>(i, j);
          }
          pose.val[i][3] = t_calc.at<double>(i, 0);
        }
        cout << "pose update: \n" << pose << endl;
        // cout << "r_calc: \n" << r_calc << endl;
        // cout << "t_calc: \n" << t_calc << endl;
        // cout << "r_degree: \n" << r_degree << endl;

        prev_prev_r = prev_r;
        // prev_prev_t = prev_t;
        prev_r = r_degree;
        // prev_t = t_calc;
      } else {
        pose = pose*Matrix::inv(viso.getMotion());
        prev_prev_r = prev_r;
        prev_prev_t = prev_t;
        double dl[16] = {0};
        for (int i=0; i<4; ++ i)
          for (int j=0; j<4; ++ j) {
            dl[4*i+j] = pose.val[i][j];
          }
        Mat tmp(4, 4, CV_64FC1, dl);
        Mat tmp_r(tmp, Range(0, 3), Range(0, 3));
        // Mat tmp_t(tmp, Range(0, 3), Range(3, 4));
        Rodrigues(tmp_r, prev_r);
        // prev_t = tmp_t;
      }

      //  right_img = gray_frame;
      right_img = gray_frame.clone();
      if (!left_img.empty()) {
        getDisparity(left_img, right_img, left_img_disp8, right_img_disp8);
        imshow("left disp8", left_img_disp8);
        getDisparity(right_img, left_img, left_img_disp8, right_img_disp8);
        imshow("right disp8", right_img_disp8);
      }
      // left_img = right_img;
      cv::swap(left_img, right_img);

      // output some statistics
      double num_matches = viso.getNumberOfMatches();
      double num_inliers = viso.getNumberOfInliers();
      // reconstruction from 3d
      re3d.update(p_matched, viso.getMotion(), 1, 2, 30, 3);
      vector<Reconstruction::point3d> p = re3d.getPoints();
      if (save_flag) {
        sprintf(numstr, "%d", frame_index ++ );
        string filename = base_dir + numstr + ".pcd";
        save3dPointsAsPCD(p, filename);
      }
      // struct triangulateio in, out;
      // cout << ", Matches: " << num_matches << endl;
      // cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << endl;
      // cout << "p_matched.size(): " << p_matched.size() << endl;
      // cout << "p.size(): " << p.size() << endl;
      // cout << pose << endl << endl;

      // ofs << ", Matches: " << num_matches;
      // ofs << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
      ofs_rt << pose << endl << endl;
      ofs_x << pose.val[0][3] << endl;
      ofs_y << pose.val[1][3] << endl;
      ofs_z << pose.val[2][3] << endl;

    } else {
      cout << " ... failed!" << endl;
      // ofs << " ... failed!" << endl;
    }
    imshow("frame", gray_frame);
    optical_flow = Mat::zeros(gray_frame.rows, gray_frame.cols, CV_8UC3);
    drawMatched(p_matched, optical_flow);
    imshow("matched", optical_flow);
    // prev_image
    // left_img = gray_frame;
    // release uint8_t buffers
    free(gray_frame_data);
    if (waitKey(10)==27 || waitKey(10)==0x20)
      break;

    cout << "-----------------------------" << endl;
  }
  // output
  cout << "Demo complete! Exiting ..." << endl;

  // exit

  return 0;
}
