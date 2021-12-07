/**
 * @author David Tapiador de Vera
 */

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;


int max_color_space = 4;
int color_space = 0;
int radio = 50;
Mat original;


void lowpassFilter(const cv::Mat &dft_Filter, int distance)
{
  Mat tmp = Mat(dft_Filter.rows, dft_Filter.cols, CV_32F);
  tmp = 0;
  circle(tmp, Point(dft_Filter.rows/2, dft_Filter.cols/2), 50, Scalar(255,255,255), -1);

  Mat tmp2[] = {tmp, tmp};
  merge(tmp2, 2, dft_Filter);
}

void highpassFilter(const cv::Mat &dft_Filter, int distance)
{
  Mat tmp = Mat(dft_Filter.rows, dft_Filter.cols, CV_32F);
  tmp = 255;
  circle(tmp, Point(dft_Filter.rows/2, dft_Filter.cols/2), 50, Scalar(0,0,0), -1);

  Mat tmp2[] = {tmp, tmp};
  merge(tmp2, 2, dft_Filter);
}


Mat get_complexI(Mat I)
{ 
  Mat padded;
  int m = getOptimalDFTSize( I.rows );
  int n = getOptimalDFTSize( I.cols ); // on the border add zero values
  copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
  
  // 2. Make place for both the complex and the real values
  Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
  Mat complexI;
  merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

  // 3. Make the Discrete Fourier Transform
  dft(complexI, complexI);            // this way the result may fit in the source matrix
  return complexI;
}

void rearrange(Mat magI) {
  // rearrange the quadrants of Fourier image  so that the origin is at the image center
  int cx = magI.cols/2;
  int cy = magI.rows/2;
  Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
  Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
  Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
  Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right
  Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);
  q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
  q2.copyTo(q1);
  tmp.copyTo(q2);
}

Mat get_highpass_img()
{
  Mat img, complexI, filter, filterOutput, lowpass, planes[2];

  complexI = get_complexI(original);
  filter = complexI.clone();

  highpassFilter(filter, radio); // create an ideal low pass filter

  rearrange(complexI); // rearrage quadrants
  mulSpectrums(complexI, filter, complexI, 0); // multiply 2 spectrums
  rearrange(complexI); // rearrage quadrants

  idft(complexI, complexI, DFT_INVERSE|DFT_REAL_OUTPUT); // compute inverse

  split(complexI, planes);
  normalize(planes[0], lowpass, 0, 1, NORM_MINMAX);
  return lowpass;
}

Mat get_lowpass_img()
{
  Mat img, complexI, filter, filterOutput, highpass, planes[2];

  complexI = get_complexI(original);
  filter = complexI.clone();

  lowpassFilter(filter, radio); // create an ideal low pass filter

  rearrange(complexI); // rearrage quadrants
  mulSpectrums(complexI, filter, complexI, 0); // multiply 2 spectrums
  rearrange(complexI); // rearrage quadrants

  idft(complexI, complexI, DFT_INVERSE|DFT_REAL_OUTPUT); // compute inverse

  split(complexI, planes);
  normalize(planes[0], highpass, 0, 1, NORM_MINMAX);
  return highpass;
}


void ChangeColorSpace(int space, void* max_color)
{
  Mat img;
  switch(space) {
    case 1: //Fourier
      {
      Mat fourier, complexI, planes[2];

      complexI = get_complexI(original);

      split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
      magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
      img = planes[0];
      img += Scalar::all(1);                    // switch to logarithmic scale
      log(img, img);
      img = img(Rect(0, 0, img.cols & -2, img.rows & -2));

      rearrange(img); // rearrage quadrants
      normalize(img, img, 0, 1, NORM_MINMAX); // Transform the matrix with float values into a
                                              // viewable image form (float between values 0 and 1).
      imshow("Practise 2", img);
      break;
    }
    case 2://Highpass
      img = get_highpass_img();
      imshow("Practise 2", img);

      break;

    case 3: //Lowpass
      img = get_lowpass_img();
      imshow("Practise 2", img);

      break;

    case 4: //AND
    {
      Mat highpass = get_highpass_img();
      Mat lowpass = get_lowpass_img();

      Mat dest_highpass(highpass.rows, highpass.cols, 0);
      Mat dest_lowpass(lowpass.rows, lowpass.cols, 0);
      Mat AND(original.rows, original.cols, 0);

      //Highpass umbral
      float threshold_high = 0.4;
      // Read pixel values
      for ( int i=0; i<highpass.rows; i++ ) {
        for ( int j=0; j<highpass.cols; j++ ) {
          // You can now access the pixel value and calculate the new value
          float value = highpass.at<float>(i,j);
          if (value > threshold_high) 
            dest_highpass.at<uchar>(i,j) = (float)255;
          else 
            dest_highpass.at<uchar>(i,j) = (float)0;
        }
      }

      //Lowpass umbral
      float threshold_low = 0.6;
      // Read pixel values
      for ( int i=0; i<lowpass.rows; i++ ) {
        for ( int j=0; j<lowpass.cols; j++ ) {
          // You can now access the pixel value and calculate the new value
          float value = lowpass.at<float>(i,j);
          if (value > threshold_low) 
            dest_lowpass.at<uchar>(i,j) = (float)255;
          else 
            dest_lowpass.at<uchar>(i,j) = (float)0;
        }
      }

      //AND
      bitwise_and(dest_lowpass, dest_highpass, AND);
      imshow("Practise 2", AND);
      break;
    }
    default: //Original
      imshow("Practise 2", original);
      break;
  }
}

int main(int argc, char ** argv)
{
  original = imread("lenna.jpg", 0);
  if( original.empty()){
      cout << "Error opening image" << endl;
      return EXIT_FAILURE;
  }
  resize(original, original, Size(512, 512));

  imshow("Practise 2", original);

  // Create Trackbar
  createTrackbar( "Element:\n 0: Original \n 1: Fourier \n 2: Highpass Filter \n 3: Lowpass Filter \n 4: AND",
    "Practise 2", &color_space, max_color_space, ChangeColorSpace );
  waitKey();

  return EXIT_SUCCESS;
}
