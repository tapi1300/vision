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

// Trackbar values
int max_selector = 2, max_Canny = 255, max_accumulator = 300, max_radius = 50, max_aspect_ratio = 4;
int selector_val = 0, Canny_val = 100, accumulator_val = 200, radius_val = 30, aspect_ratio_val = 1;

// Colors
Scalar blue = Scalar(200,0,0), green = Scalar(0,200, 0), red = Scalar(0,0,200);
Scalar black = Scalar(0,0,0), white = Scalar(255,255,255);

// Window names
string main_name = "Practise 4";

// Trackbar names
string selector_trackbar = "0: Hough - 1: Contours - 2: Centroids";
string canny_trackbar = "Canny thresh [0-255]:";
string accumulator_trackbar = "Hough lines accumulator [0-300]:";
string radius_trackbar = "Hough radius value max [0-50]:";
string aspect_ratio_trackbar = "Aspect ratio value*0.01 [0-4]:";

// Global Image
Mat original;	/* Global for the original image
					**DO NOT CHANGE ITS VALUE**/


Mat get_Hough(Mat canny)
{
	/*This function receives a Mat with the image already filtered with Canny, and returns
		the original image with the hough lines and circles drawn.
	*/
    Mat ret_img, gray, bgr_canny;
    ret_img = original.clone();

    // Standard Hough Line Transform
    vector<Vec2f> lines; // will hold the results of the detection (rho, theta)
    HoughLines( canny, lines, 1, CV_PI/180, accumulator_val, 0, 0 ); // runs the actual detection

    vector<Vec3f> circles;
	float val;
	if(Canny_val == 0) {val = 0.1;}
	else { val = Canny_val;}

    HoughCircles(canny, circles, HOUGH_GRADIENT, 1,
                 canny.rows/16,  // change this value to detect circles with different distances to each other
                 val, 30, 0, radius_val // change the last two parameters (min_radius & max_radius) to detect larger circles
    );

    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ ) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*( a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*( a));
        line( ret_img, pt1, pt2, green, 3, LINE_AA );
	}

    for( size_t i = 0; i < circles.size(); i++ ) {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        int radius = c[2];
        // circle center
        circle( ret_img, center, 1, black, 3, LINE_AA);
        // circle outline
        circle( ret_img, center, radius, black, 3, LINE_AA);
    }
	return ret_img;
}

Mat get_Contours(Mat canny)
{
	/*This function receives a Mat with the image already filtered with Canny, and returns
		the original image with the contours drawn.
	*/

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

    // Drawing contours
    Mat ret_img = original.clone();

	for( int i = 0; i < contours.size(); i++ )
	{
		Rect rect = boundingRect(contours[i]);
		float aspect_ratio = (float)rect.width/(float)rect.height;
		if((float)abs(1. - aspect_ratio) < (0.01*aspect_ratio_val))
		{
			// Pongo el color hasta 200 para que no salgan colores cercanos al blanco (algunos no se aprecian bien)
			Scalar color = Scalar( rand()%200, rand()%200, rand()%200 );
			drawContours( ret_img, contours, (int)i, color, 2, LINE_8, hierarchy, 1);	
		}
	}
	return ret_img;
}

Mat get_centroids(Mat canny)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Vec3f> circles;

	// Obtención de centroides
	findContours( canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<Moments> m(contours.size() );
	for( size_t i = 0; i < contours.size(); i++ ) {
        m[i] = moments( contours[i] );
    }

	// Obtención de centros de Hough
	float val;
	if(Canny_val == 0) {val = 0.1;}
	else { val = Canny_val;}
    HoughCircles(canny, circles, HOUGH_GRADIENT, 1,
                 canny.rows/16,  // change this value to detect circles with different distances to each other
                 val, 30, 0, radius_val // change the last two parameters (min_radius & max_radius) to detect larger circles
    );

	/***DIBUJAR***/
	// El orden de dibujo va por el tamaño, dibujando primero los circulos más
	//		grandes para que no tapen a los pequeños.
	Mat drawing = original.clone();

	// Dibujo de puntos a menos de 4 de distancia respecto a algún centroide
    for( size_t i = 0; i < circles.size(); i++ ) {
        Point center_hough(circles[i][0], circles[i][1]);
		for( size_t j = 0; j < m.size(); j++ ) {
			Point center_centroide(m[j].m10/m[j].m00, m[j].m01/m[j].m00);
			float distancia = sqrt(pow(center_hough.x-center_centroide.x,2) + pow(center_hough.y-center_centroide.y,2));
			if(distancia < 4.)
			{
				circle(drawing, center_hough, 10, green, -1, LINE_AA);
			}
		}
	}

	// Dibujo de centros de hough
    for( size_t i = 0; i < circles.size(); i++ ) {
        Point center(circles[i][0], circles[i][1]);
        circle( drawing, center, 6, blue, -1, LINE_AA);
	}

    // Dibujo de centroides
    for( size_t i = 0; i < m.size(); i++ ) {
		// Pongo el color hasta 200 para que no salgan colores cercanos al blanco (algunos no se aprecian bien)
		Scalar color = Scalar( rand()%200, rand()%200, rand()%200 );
		Point p(m[i].m10/m[i].m00, m[i].m01/m[i].m00);
		circle(drawing, p, 4, color, -1, LINE_AA);
    }

	return drawing;
}

// CALLBACK FUNCTIONS
void update_images()
{
	/*This function updates the images shown with the new values of
		the globals "min_val" and "max_val".
	*/
	// Show all images on their windows
    Mat canny_img, final_img, temp_img, temp_img2;
	cvtColor(original, temp_img, COLOR_BGR2GRAY);
	Canny(temp_img, canny_img, Canny_val, Canny_val*2, 3);
	switch (selector_val)
	{
	case 0:
		final_img = get_Hough(canny_img);
		break;

	case 1:
		final_img = get_Contours(canny_img);
		break;

	case 2:
		final_img = get_centroids(canny_img);
		break;
	}
	imshow(main_name, final_img);
}

void ChangeVal(int val, void* trackbar_num)
{
	/*This function changes the global variable depending on the trackbar used:
		trackbar_num:
			0 = selector_val
			1 = Canny_val
			1 = accumulator_val
			3 = radius_val
			4 = aspect_ratio_val
	After updating the global variable, also updates the image shown.
	*/
	int param = *((int *)&trackbar_num);

	switch (param) {
	case 0:
		selector_val = val;
		break;
	case 1:
		Canny_val = val;
		break;
	case 2:
		accumulator_val = val;
		break;
	case 3:
		radius_val = val;
		break;
	case 4:
		aspect_ratio_val = val;
		break;
	}
	update_images();
}

int main(int argc, char** argv)
{
	original = imread("damas.jpg");
	if( original.empty()){
			fprintf(stderr, "Error opening image");
			return EXIT_FAILURE;
	}
	resize(original, original, Size(512, 512));

	update_images();

	// Create Trackbars
	createTrackbar(selector_trackbar, main_name, &selector_val, max_selector, ChangeVal, (void *)0);
	createTrackbar(canny_trackbar, main_name, &Canny_val, max_Canny, ChangeVal, (void *)1);
	createTrackbar(accumulator_trackbar, main_name, &accumulator_val, max_accumulator, ChangeVal, (void *)2);
	createTrackbar(radius_trackbar, main_name, &radius_val, max_radius, ChangeVal, (void *)3);
	createTrackbar(aspect_ratio_trackbar, main_name, &aspect_ratio_val, max_aspect_ratio, ChangeVal, (void *)4);
	waitKey();

	return EXIT_SUCCESS;
}
