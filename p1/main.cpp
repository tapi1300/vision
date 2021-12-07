/**
 * @author David Tapiador 
**/

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <math.h> 

using namespace cv;
using namespace std;

int max_color_space = 4;
int color_space = 0;
Mat original;

void ChangeColorSpace(int space, void* max_color)
{
  vector<Mat> three_channels;
  split(original, three_channels);

  switch(space) {
    case 1: //CMY
      for ( int i=0; i<original.rows; i++ ) {
        for ( int j=0; j<original.cols; j++ ) {
          three_channels[0].at<uchar>(i,j) = 255 - three_channels[0].at<uchar>(i,j);
          three_channels[1].at<uchar>(i,j) = 255 - three_channels[1].at<uchar>(i,j);
          three_channels[2].at<uchar>(i,j) = 255 - three_channels[2].at<uchar>(i,j);
        }
      }
      break;
    case 2: //HSI
      for(int i = 0; i < original.rows; i++) {
        for(int j = 0; j < original.cols; j++) {
          double R = (double)three_channels[2].at<uchar>(i,j) /255;
          double G = (double)three_channels[1].at<uchar>(i,j) /255;
          double B = (double)three_channels[0].at<uchar>(i,j) /255;

          double H = acos( ( 0.5*((R-G)+(R-B)))/(sqrt(pow(R-B,2)+(R-B)*(G-B))));
          double S = 1. - (3./(R+G+B))*min(min(R,G),B);
          double I = (R+G+B)/3.;

          H = H * 255;
          if(B > G) { H = 360 - H;}
          S = S * 255;
          I = I * 255;
          three_channels[0].at<uchar>(i,j) = H;
          three_channels[1].at<uchar>(i,j) = S;
          three_channels[2].at<uchar>(i,j) = I;
        }
      }
      break;
    case 3: //HSV
      for(int i = 0; i < original.rows; i++) {
        for(int j = 0; j < original.cols; j++) {
          double R = (double)three_channels[2].at<uchar>(i,j) /255;
          double G = (double)three_channels[1].at<uchar>(i,j) /255;
          double B = (double)three_channels[0].at<uchar>(i,j) /255;

          double H = acos( ( 0.5*((R-G)+(R-B)))/(sqrt(pow(R-B,2)+(R-B)*(G-B))));
          double S = 1. - (3./(R+G+B))*min(min(R,G),B);
          double V = max(max(R,G),B);

          H = H * 255;
          if(B > G) { H = 360 - H;}
          S = S * 255;
          V = V * 255;
          three_channels[0].at<uchar>(i,j) = H;
          three_channels[1].at<uchar>(i,j) = S;
          three_channels[2].at<uchar>(i,j) = V;
        }
      }
      break;
    case 4:
      Mat HSV_opencv;
      cvtColor(original, HSV_opencv, COLOR_RGB2HSV);
      split(HSV_opencv, three_channels);
  }

  vector<Mat> channels;
  channels.push_back(three_channels[0]);
  channels.push_back(three_channels[1]);
  channels.push_back(three_channels[2]);

  Mat new_image;
  merge(channels, new_image);
  imshow("Practise 1", new_image);
}

int main( int argc, char** argv ) {
  // Load an image
  original = imread( "RGB.jpg", IMREAD_COLOR );
  if ( original.empty() ) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }

  // Show image
  // namedWindow( "Practise 1", WINDOW_AUTOSIZE );
  imshow("Practise 1", original);

  // Create Trackbar
  createTrackbar( "Element:\n 0: RGB \n 1: CMY \n 2: HSI \n 3: HSV \n 4: HSV OpenCV", "Practise 1",
    &color_space, max_color_space,
    ChangeColorSpace );
  ChangeColorSpace(0, 0);

  waitKey(0);
  return 0;
}