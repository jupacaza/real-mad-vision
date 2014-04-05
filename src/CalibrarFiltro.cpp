/*
 * Title: Etapa 01
 * Class: Vision para Robot
 * Instructor: Dr. Jose Luis Gordillo (http://robvis.mty.itesm.mx/~gordillo/)
 * Code: 
 * Institution: Tec de Monterrey, Campus Monterrey
 * Date: 1.April.2014
 * Description: Parrot and Segmentation Algorithm.
 * This programs uses OpenCV http://www.opencv.org/
 */

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <iostream>
#include <math.h>
#include <queue>
#include <fstream>
#include "CHeli.h"

#define XIMAGE 0
#define YIMAGE 20
#define XVIDEO 1250
#define XYIQ 0
#define YYIQ 570
#define XHSV 950
#define YHSV 570

#define RGB 0
#define YIQ 1
#define HSV 2


#define maxColors 10

//////////////////// TURN PARROT ON (1) or OFF (0)
#define PARROT 0

using namespace cv;
using namespace std;

// Here we will store points
vector<Point> points;
Point orig, dest;

/* Create images where captured and transformed frames are going to be stored */
Mat currentImage;
Mat storedImage, backupImage;
Mat filteredImage;
Mat blackAndWhiteImage;
Mat binaryImage;
Mat grayScaleImage;
Mat YIQImage;
Mat HSVImage;
Mat closing;
//for parrot use
CHeli *heli;
CRawImage *image;

//other globals
bool greyScalePointReady;
Point greyScalePoint;

void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param);
void rawToMat( Mat &destImage, CRawImage* sourceImage);

void generaimagenFiltradaBinaria (const Mat &origin, Mat &destination);
void generateHSV (const Mat &origin,  Mat &destination);

ofstream outFile;

int main(int argc, char *argv[])
{
/* First, open camera device */
    // Abre webcam
    VideoCapture camera;
    Mat currentImage;
    if (!PARROT) {
     
     camera.open(0);
     
    }
/* Open parrot camera instead */
    if (PARROT) {
    //establishing connection with the quadcopter
    heli = new CHeli();
    
    //this class holds the image from the drone 
    image = new CRawImage(320,240);
    
    // init currentImage
    currentImage = Mat(240, 320, CV_8UC3);
    }


    /* Create main OpenCV window to attach callbacks */
    namedWindow("Video");

    unsigned char key = 0;

/*To use timers
// **Start timer **
clock_t start;

assert((start = clock())!=-1);
*/   


    while (key != 27)
	{
        /* 1 Obtain a new frame from camera web */
        if (!PARROT) {
		camera >> currentImage;
        }
        /* 2 Obtain image from Parrot instead */
        else {
        //image is captured
        heli->renewImage(image);
        // Copy to OpenCV Mat
        rawToMat(currentImage, image);
        }        

		if (currentImage.data) 
		{
            /* Show image */
            imshow("Video", currentImage);
            cvMoveWindow("Video", XVIDEO, YIMAGE);
		}
		else
		{
			cout << "No image data.. " << endl;
		}

        if (key == 'c') { //capture snapshot
            storedImage.release();
            points.clear(); //remove all stored points on the vector
            storedImage = currentImage.clone();
            backupImage = currentImage.clone();
            namedWindow("Image");
            cvMoveWindow("Image", XIMAGE, YIMAGE);
            setMouseCallback("Image", mouseCoordinatesExampleCallback);
            imshow("Image", storedImage);


        }
        /***********************************
            Generate all processing
        ************************************/
        
        if(key == 'r' && storedImage.data)
        {
            imshow("Image",backupImage);
            storedImage = backupImage;
        }

        if(key == 'h' && storedImage.data && (orig.x != dest.x))
        {
            Mat dilateFilter; //Declarar la variable aqui para que tambien se pueda usar la funcion del enter

            generaimagenFiltradaBinaria(storedImage, filteredImage);
            dilate(filteredImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            imshow("Image", closing);
        }


        /*************************************/
        key = waitKey(5);
	}
} //main

void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param)
{
    static bool clicked=false;
    //static char wname[5] = {'B', 0, 'G', 0, 'R'};

    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            cout << "  Mouse X, Y: " << x << ", " << y << 
                " color: R " << (int)storedImage.at<Vec3b>(y, x)[2] <<
                " G " << (int)storedImage.at<Vec3b>(y, x)[1] <<
                " B " << (int)storedImage.at<Vec3b>(y, x)[0];
            cout << endl;

            clicked=true;
            orig = Point(x,y);
            /*  Draw a point */
            points.push_back(Point(x, y));

            imshow("Image", storedImage);
         //   showHistogram(storedImage, wname, XIMAGE, YIMAGE, storedImage.at<Vec3b>(y, x));

            break;

        case CV_EVENT_MOUSEMOVE:
            if (clicked)
                {
                    Mat tmpImage;
                    tmpImage=storedImage.clone();
                    dest = Point(x,y);
                    {
                        rectangle(tmpImage, orig, dest,Scalar( 0, 255, 0));
                         imshow("Image", tmpImage);
                    }
                }
            break;
        case CV_EVENT_LBUTTONUP:
            
            clicked=false;
            break;
    }
}//mouseCoordinatesExampleCallback

// Convert CRawImage to Mat *for parrot use*
void rawToMat( Mat &destImage, CRawImage* sourceImage)
{   
    uchar *pointerImage = destImage.ptr(0);
    
    for (int i = 0; i < 240*320; i++)
    {
        pointerImage[3*i] = sourceImage->data[3*i+2];
        pointerImage[3*i+1] = sourceImage->data[3*i+1];
        pointerImage[3*i+2] = sourceImage->data[3*i];
    }
} //rawToMat

void generaimagenFiltradaBinaria (const Mat &sourceImage, Mat &destinationImage)
{
    Vec3b maxVec, minVec;
    bool gray;
    int tolerance[3], maxValue, minValue;

    int startY, endY, startX, endX;

    int channels = sourceImage.channels();

    //transformar a YIQ o HSV si es necesario
    Mat transformedImage;

    transformedImage = HSVImage;
    generateHSV (sourceImage,transformedImage);

    maxValue = 255;
    minValue = 0;
    tolerance[0] = 5; //H
    tolerance[1] = 10; //S
    tolerance[2] = 3; //V
    
    minVec = transformedImage.at<Vec3b>(orig.y, orig.x);
    maxVec = minVec;

    if(dest.y >= orig.y)
    {
        startY = orig.y;
        endY = dest.y;
    }
    else
    {
        startY = dest.y;
        endY = orig.y;
    }

    if(dest.x >= orig.x)
    {
        startX = orig.x;
        endX = dest.x;
    }
    else
    {
        startX = dest.x;
        endX = orig.x;
    }

    for (int y = startY; y <= endY; y++) 
    {
        uchar* rowPointer = (uchar*) transformedImage.ptr<uchar>(y);

        for (int x = startX; x <= endX; x++)
            for (int i = 0; i < channels; ++i)
            {
                if(rowPointer[x * channels + i] > maxVec[i])
                {
                    maxVec[i] = rowPointer[x * channels + i];
                }

                if(rowPointer[x * channels + i] < minVec[i])
                {
                    minVec[i] = rowPointer[x * channels + i];
                }
            }
    }
    
   for (int i = 0; i < channels; ++i)
    {

        if ((minVec[i] - tolerance[i]) < minValue)
            minVec[i] = minValue;
        else
            minVec[i] -= tolerance[i];
        
        if ((maxVec[i] + tolerance[i]) > maxValue)
            maxVec[i] = maxValue;
        else
            maxVec[i] += tolerance[i];
    }


    // ya tenemos los valores filtrados, guardamos en un archivo:
    /* OUTPUT FILE */
    outFile.open("../src/main/data/limits.txt");
    if (channels == 3) {    //solo nos importa guardar para imagenes de 3 canales
        outFile << minVec[0] << ' ' << minVec[1] << ' ' << minVec[2] << '\n';
        outFile << maxVec[0] << ' ' << maxVec[1] << ' ' << maxVec[2] << '\n';

        cout << (int)minVec[0] << ' ' << (int)minVec[1] << ' ' << (int)minVec[2] << '\n';
        cout << (int)maxVec[0] << ' ' << (int)maxVec[1] << ' ' << (int)maxVec[2] << '\n';
    }
    outFile.close();
    ////////////////////////////////////////////////////////////
    
    if (destinationImage.empty())
        destinationImage = Mat(transformedImage.rows, transformedImage.cols, CV_8UC1);
    
    for (int y = 0; y < sourceImage.rows; ++y) 
    {   
      //  uchar* sourceRowPointer = (uchar*) sourceImage.ptr<uchar>(y);
        uchar* transformedRowPointer = (uchar*) transformedImage.ptr<uchar>(y);
        uchar* destinationRowPointer = (uchar*) destinationImage.ptr<uchar>(y);
        for (int x = 0; x < sourceImage.cols; ++x)
        {
            gray = false;
            for (int i = 0; i < channels; ++i)
            {
                if(transformedRowPointer[x * channels + i] > maxVec[i] || transformedRowPointer[x * channels + i] < minVec[i])
                {
                    destinationRowPointer[x] = 0;
                    gray = true;
                } else
                {
                    if(!gray)
                        destinationRowPointer[x] = 255;
                }
            }
        }
    }
} //generaimagenFiltrada

void generateHSV (const Mat &origin,  Mat &destination)
{
    double blue, red, green;
    double hv, sv, vv;
    double min, max, delta;

    if (destination.empty())
        destination= Mat(origin.rows, origin.cols, origin.type());

    int channels = origin.channels();

    for (int y = 0; y < origin.rows; ++y) 
    {
        uchar* sourceRowPointer = (uchar*) origin.ptr<uchar>(y);
        uchar* destinationRowPointer = (uchar*) destination.ptr<uchar>(y);

        for (int x = 0; x < origin.cols; ++x)
        {
            blue = sourceRowPointer[x * channels]/255.0;
            green = sourceRowPointer[x * channels + 1]/255.0;
            red = sourceRowPointer[x * channels + 2]/255.0;

            min = MIN(red,MIN(green,blue));
            max = MAX(red,MAX(green,blue));
            vv=max; //Es V
            delta = max - min;
            if( delta != 0 ) {
                sv = delta / max;       // s
            }
            else {
                // r = g = b = 0        // si s=0 entonces v es indefinido
                sv = 0;
                //hv = -1;
                //return;
            }

            if(red == max) {
                hv = ( green - blue ) / delta;     // between yellow & magenta
            }
            else if( green == max ) {
                hv = 2 +( blue - red ) / delta; // between cyan & yellow
            }
            else {
                hv = 4 +( red - green ) / delta; // between magenta & cyan
            }

            hv *= 60;               // degrees

            if(hv < 0) {
                hv += 360;
            } else if (hv > 360) {
                hv -= 360;
            }

            /*
                hv is a value between 0 and 360
                sv is a value between 0 and 1
                vv is a value between 0 and 1
            */
            destinationRowPointer[x * channels] = hv * 255 / 360;
            destinationRowPointer[x * channels + 1] = sv * 255;
            destinationRowPointer[x * channels + 2] = vv * 255;
        }
    }
} //generateHSV