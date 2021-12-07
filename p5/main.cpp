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
int max_Iterator = 100, max_Element = 2, max_Kernel = 7;
int Iterator_val = 60, Element_val = 0, Kernel_val = 1;

// Colors
Vec3b red = Vec3b(0, 0, 255);

// Window names
string main_name = "Skeleton Demo";

// Trackbar names
string iterator_trackbar = "Iterations:\n0-100";
string element_trackbar = "Element:\n0: Rect - 1: Cross - 2 Ellipse";
string kernel_trackbar = "Kernel size:\n2n + 1";

// Global Image
Mat original;	/* Global for the original image
					**DO NOT CHANGE ITS VALUE**/


Mat open_img(Mat src)
{
	/* Hace una apertura (transformación morfológica) a la matriz
		src y la devuelve.
	*/
	int operation = 2;
	Mat ret, element;
	element = getStructuringElement( Element_val, Size( 2*Kernel_val + 1, 2*Kernel_val+1 ), Point( Kernel_val, Kernel_val ) );
	morphologyEx( src, ret, operation, element );
	return ret;
}

Mat erode( Mat src) {
	/* Erosiona la matriz src y se devuelte la matriz erosionada.
	*/
	Mat ret, element;
	element = getStructuringElement( Element_val, Size( 2*Kernel_val + 1, 2*Kernel_val+1 ), Point( Kernel_val, Kernel_val ) );
	erode( src, ret, element );
	return ret;
}

// CALLBACK FUNCTIONS
void update_images(int, void*)
{
	/*This function updates the images shown with the new values of
		the globals "min_val" and "max_val".
	*/
    if( Iterator_val == 0)
	{
		return;
	}

	Mat open, temp, eroded, final, esqueleto, orig_copy;

	esqueleto = Mat::zeros(original.size(), original.type());
	orig_copy = original.clone();

	for( int i = 0; i < Iterator_val; i++ )
	{
		// Abrir la imagen
		open = open_img(orig_copy);
		// Restar
		temp = orig_copy - open;
		// Erosionar
		orig_copy = erode(orig_copy);
		// Unir
		bitwise_or(esqueleto, temp, esqueleto);

		imshow(main_name, esqueleto);
		waitKey(50);
	}

	// Poner el esqueleto en rojo en la imagen original y mostrarla 1s
	final = original.clone();
	cvtColor(final, final, COLOR_GRAY2BGR);
	threshold(esqueleto,esqueleto, 50, 255, THRESH_BINARY);
	// normalize(esqueleto, esqueleto, 0, 255, NORM_MINMAX, -1, Mat() );
	for (int i = 0; i < final.cols; i++) {
		for (int j = 0; j < final.rows; j++) {
			Scalar intensity = esqueleto.at<uchar>(j, i);
			if (intensity.val[0] == 255) {
				final.at<Vec3b>(j, i) = red;
			}
		}
	}

	imshow(main_name, final);
	waitKey(1000);
}

int main(int argc, char** argv)
{
	original = imread("model.png", IMREAD_GRAYSCALE);
	if( original.empty()){
			fprintf(stderr, "Error opening image");
			return EXIT_FAILURE;
	}
	resize(original, original, Size(512, 512));

	// Create Trackbars
	update_images(0, (void*)0);

	createTrackbar(iterator_trackbar, main_name, &Iterator_val, max_Iterator, update_images);
	createTrackbar(element_trackbar, main_name, &Element_val, max_Element, update_images);
	createTrackbar(kernel_trackbar, main_name, &Kernel_val, max_Kernel, update_images);


	waitKey();

	return EXIT_SUCCESS;
}
