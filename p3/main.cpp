/**
 * @author David Tapiador de Vera
**/

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;


int max_shrink = 255;
int min_val = 0, max_val = 30;
int radio = 50;
Scalar red = Scalar(0,0,255), blue = Scalar(255, 0, 0);
string main_name = "Practise 3";
string shrink_name = "Shrink";
string stretch_name = "Stretch";
string substract_name = "Substract original-shrink";
string equalized_name = "Equalized";
string trackbar_label_min = "Shrink value min: 0-255";
string trackbar_label_max = "Shrink value max: 0-255";
Mat original;

// ERRORS FUNCTIONS
void bad_trackbars()
{
	/*This function is called when the "minimum" trackbar is higher
		than the "maximum" trackbar.
	Sends an error to the shell and resets both trackbars.
	*/
	min_val = 50;
	max_val = 150;
	fprintf(stderr, "The minimum value cannot be lower than the maximum value!\n");
	setTrackbarPos(trackbar_label_min, main_name, min_val);
	setTrackbarPos(trackbar_label_max, main_name, max_val);
}

// LOWPASS FUNCTIONS
void lowpassFilter(const cv::Mat &dft_Filter, int distance)
{
	/*This function saves in dft_Filter the lowpass filter
	(black image with a blue circle in the middle)
	*/
	Mat tmp = Mat(dft_Filter.rows, dft_Filter.cols, CV_32F);
	tmp = 0;
	circle(tmp, Point(dft_Filter.rows/2, dft_Filter.cols/2), 50, Scalar(255,255,255), -1);

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
	merge(planes, 2, complexI);		// Add to the expanded another plane with zeros

	// 3. Make the Discrete Fourier Transform
	dft(complexI, complexI);	// this way the result may fit in the source matrix
	return complexI;
}

void rearrange(Mat magI) {
	/*Rearrange the quadrants of Fourier image so that the
	origin is at the image center.
	*/
	int cx = magI.cols/2;
	int cy = magI.rows/2;
	Mat q0(magI, Rect(0, 0, cx, cy));	// Top-Left - Create a ROI per quadrant
	Mat q1(magI, Rect(cx, 0, cx, cy));	// Top-Right
	Mat q2(magI, Rect(0, cy, cx, cy));	// Bottom-Left
	Mat q3(magI, Rect(cx, cy, cx, cy));	// Bottom-Right
	Mat tmp;							// swap quadrants (Top-Left with Bottom-Right)
	
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);
	q1.copyTo(tmp);						// swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);
}

Mat get_lowpass_img()
{
	/*This function returns the lowpass filtered image of the global
	variable "Mat original".
	*/
	Mat img, complexI, filter, filterOutput, lowpass, planes[2];

	complexI = get_complexI(original);
	filter = complexI.clone();

	lowpassFilter(filter, radio); // create an ideal low pass filter

	rearrange(complexI); // rearrage quadrants
	mulSpectrums(complexI, filter, complexI, 0); // multiply 2 spectrums
	rearrange(complexI); // rearrage quadrants

	idft(complexI, complexI, DFT_INVERSE|DFT_REAL_OUTPUT); // compute inverse

	split(complexI, planes);
	normalize(planes[0], lowpass, 0, 1, NORM_MINMAX);
	return lowpass;
}

// HISTOGRAMS FUNCTIONS
Mat get_histogram(Mat image)
{
	/*This function receives an B&W image and returns its
		histogram without normalizing.
	*/
	int histSize = 256;
	float range[] = {0, 255}; //the upper boundary is exclusive
	const float* histRange = { range };
	bool uniform = true, accumulate = false;
	Mat hist;

	calcHist( &image, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
	return hist;
}

Mat get_histImage(Mat hist_mod, Mat hist_orig)
{
	/*This function receives two histograms, normalizes both and creates
		a single image of 521x400 pixels that can be ploted with "imshow".
	*/
	int histSize = 256;
	int hist_w = 512, hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );
	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar(0) );

	normalize(hist_mod, hist_mod, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	normalize(hist_orig, hist_orig, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

	for( int i = 1; i < histSize; i++ )
	{
		line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist_mod.at<float>(i-1)) ),
			Point( bin_w*(i), hist_h - cvRound(hist_mod.at<float>(i)) ),
			blue, 2, 8, 0  );
		line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist_orig.at<float>(i-1)) ),
			Point( bin_w*(i), hist_h - cvRound(hist_orig.at<float>(i)) ),
			red, 2, 8, 0  );
	}
	return histImage;
}


// IMAGE FILTERING FUNCTIONS
void get_shrink(Mat src, float _min, float _max, Mat& dest, Mat& hist_dest)
{
	/*This function receives an image and two floats (min & max for shrinking).
	It saves the shrinked image in "dest" and its histogram in "hist_dest".
	*/
	float Rmax, Rmin, Cmax, Cmin;
	Mat shrink(src.rows, src.cols, 0);
	Cmax = _max;
	Cmin = _min;

	// Get min and max of histogram
	for ( int i=0; i<src.rows; i++ ) {
		for ( int j=0; j<src.cols; j++ ) {
			float value = src.at<float>(i,j);
			if(i == 0 && j == 0)
			{
				Rmax = value;
				Rmin = value;
				continue;
			}
			Rmax = max(Rmax,value);
			Rmin = min(Rmin,value); 
		}
	}
	// Shrink the source image with the values from the params
	for (int i=0; i<src.rows; i++ ) {
		for ( int j=0; j<src.cols; j++ ) {
			float value = src.at<float>(i,j);
			shrink.at<uchar>(i,j) = ((Cmax-Cmin)/(Rmax-Rmin))*(value-Rmin)+Cmin;
		}
	}
	dest = shrink;
	hist_dest = get_histogram(dest);
}

void get_substract(Mat src1, Mat src2, Mat& dest, Mat& hist_dest)
{
	/*This function makes a pixel by pixel substraction of src1-src2,
		saves the image in "dest" and its histogram in "hist_dest". 
	*/
	Mat sub(src1.rows, src1.cols, 0);
	for (int i=0; i<src1.rows; i++ ) {
		for ( int j=0; j<src1.cols; j++ ) {
			float value1 = (float)src1.at<uchar>(i,j);
			float value2 = (float)src2.at<uchar>(i,j);
			sub.at<uchar>(i,j) = value1-value2;
		}
	}

	dest = sub;
	hist_dest = get_histogram(sub);
}

void get_stretch(Mat src, Mat& dest, Mat& hist_dest)
{
	/*This function receives an image. It saves the streched
		image in "dest" and its histogram in "hist_dest".
	*/
	float InMax, InMin, MAX, MIN;
	Mat stretch(src.rows, src.cols, 0);
	MAX = 255;
	MIN = 0;

	// Get min and max of histogram
	for ( int i=0; i<src.rows; i++ ) {
		for ( int j=0; j<src.cols; j++ ) {
			float value = (float)src.at<uchar>(i,j);
			if(i == 0 && j == 0)
			{
				InMax = value;
				InMin = value;
				continue;
			}
			InMax = max(InMax,value);
			InMin = min(InMin,value); 
		}
	}

	// Stretch the source image with the values from the params
	for (int i=0; i<src.rows; i++ ) {
		for ( int j=0; j<src.cols; j++ ) {
			float value = src.at<uchar>(i,j);
			stretch.at<uchar>(i,j) = (((value-InMin)/(InMax-InMin))*(MAX-MIN)+MIN);
		}
	}
	dest = stretch;
	hist_dest = get_histogram(dest);
}

void get_equalized(Mat src, Mat& dest, Mat& hist_dest)
{
	/*This function receives an image and saves de equalized image
		in "dest" and its histogram at "hist_dest".
	*/
	equalizeHist(src, dest);
	hist_dest = get_histogram(dest);
}

// CALLBACK FUNCTIONS
void update_images()
{
	/*This function updates the images shown with the new values of
		the globals "min_val" and "max_val".
	*/
	if(max_val <= min_val){bad_trackbars();}

	// Variable initialization for images and histograms
	Mat lowpass, shrink_img, substract_img, stretch_img, equalized_img;
	Mat original_histImage, shrink_hist, substract_hist, stretch_hist, equalized_hist;
	Mat shrink_hist_plot, substract_hist_plot, stretch_hist_plot, equalized_hist_plot;
	
	lowpass = get_lowpass_img();
	original_histImage = get_histogram(original);

	// Apply the hist transforms
	get_shrink(lowpass, min_val, max_val, shrink_img, shrink_hist);
	get_substract(original, shrink_img, substract_img, substract_hist);
	get_stretch(substract_img, stretch_img, stretch_hist);
	get_equalized(stretch_img, equalized_img, equalized_hist);

	// Get the different image 2D of the histograms
	shrink_hist_plot = get_histImage(shrink_hist, original_histImage);
	substract_hist_plot = get_histImage(substract_hist, original_histImage);
	stretch_hist_plot = get_histImage(stretch_hist, original_histImage);
	equalized_hist_plot = get_histImage(equalized_hist, original_histImage);

	// Write the comparison between the different histograms and the original image histogram
	putText(shrink_hist_plot, to_string(compareHist(original_histImage,shrink_hist, HISTCMP_CORREL)),
		Point(shrink_hist_plot.rows-20, 15),FONT_HERSHEY_SIMPLEX, 0.5, red);
	putText(substract_hist_plot, to_string(compareHist(original_histImage,substract_hist, HISTCMP_CORREL)),
		Point(substract_hist_plot.rows-20, 15),FONT_HERSHEY_SIMPLEX, 0.5, red);
	putText(stretch_hist_plot, to_string(compareHist(original_histImage,stretch_hist, HISTCMP_CORREL)),
		Point(stretch_hist_plot.rows-20, 15),FONT_HERSHEY_SIMPLEX, 0.5, red);
	putText(equalized_hist_plot, to_string(compareHist(original_histImage,equalized_hist, HISTCMP_CORREL)),
		Point(equalized_hist_plot.rows-20, 15),FONT_HERSHEY_SIMPLEX, 0.5, red);

	// Show all images on their windows
	imshow(shrink_name, shrink_hist_plot);
	imshow(substract_name, substract_hist_plot);
	imshow(stretch_name, stretch_hist_plot);
	imshow(equalized_name, equalized_hist_plot);
	imshow(main_name, equalized_img);
}

void ChangeVal(int val, void* max_or_min)
{
	/*This function changes the global variable depending on the trackbar used:
		max_or_min:
			0 = min trackbar
			1 = max trackbar
	*/
	int param = *((int *)&max_or_min);
	if(param == 0)
	{
		min_val = val;
	} else {
		max_val = val;
	}
	update_images();
}

int main(int argc, char** argv)
{
	original = imread("lenna.jpg", 0);
	if( original.empty()){
			fprintf(stderr, "Error opening image");
			return EXIT_FAILURE;
	}
	resize(original, original, Size(512, 512));

	update_images();

	// Create Trackbars
	createTrackbar(trackbar_label_min, main_name, &min_val, max_shrink, ChangeVal, (void*)0);
	createTrackbar(trackbar_label_max, main_name, &max_val, max_shrink, ChangeVal, (void*)1);
	waitKey();

	return EXIT_SUCCESS;
}
