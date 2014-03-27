/*
 * Title: Etapa 01
 * Class: Vision para Robot
 * Instructor: Dr. Jose Luis Gordillo (http://robvis.mty.itesm.mx/~gordillo/)
 * Code: 
 * Institution: Tec de Monterrey, Campus Monterrey
 * Date: 12 February 2013
 * Description: Demonstration of first phase of the project.
 * This programs uses OpenCV http://www.opencv.org/
 */

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <iostream>
#include <math.h>
#include <queue>
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
//CHeli *heli;
//CRawImage *image;

//other globals
bool greyScalePointReady;
Point greyScalePoint;



const Vec3b ColorMat[maxColors] = { //BGR
    Vec3b(255, 0, 0 ),      //blue
    Vec3b(0, 255, 0 ),      //green
    Vec3b(0, 0, 255 ),      //red
    Vec3b(255, 255, 0 ),    //cyan
    Vec3b(0, 255, 255 ),    //yellow
    Vec3b(255, 0, 255 ),    //magenta
    Vec3b(0, 154, 255 ),    //orange
    Vec3b(0, 128, 128 ),    //olive
    Vec3b(0, 100, 0 ),      //darkgreen
    Vec3b(128, 0, 0 )       //navy
};



void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param);
void mouseCallbackForBinaryImage(int event, int x, int y, int flags, void* param);
void generateBlackWhite(const Mat &origin, Mat &destination);
void generateBlackWhite1C(const Mat &origin, Mat &destination) ; //Hace la imagen para un solo canal
void generateBinary(const Mat &origin, const unsigned char limit, Mat &destination); //limit va de 0 a 255
void generateGrayScale(Mat &Matrix);
void rawToMat( Mat &destImage, CRawImage* sourceImage);
void showHistogram(Mat& img, const char* wname, const int sourceX, const int sourceY);
void showHistogram(Mat& img, const char* wname, const int sourceX, const int sourceY, const Vec3b components);

Vec3b getK(int &x);

void OilDrop(const Mat &dst, Mat &colorDst);

void generaimagenFiltradaBinaria (const Mat &origin, Mat &destination,int type);
void generateYIQ(const Mat &origin, Mat &destination);
void generateHSV(const Mat &origin, Mat &destination);


int main(int argc, char *argv[])
{
	/* First, open camera device */
    // Abre webcam
    	
 VideoCapture camera;
 camera.open(0);
 Mat currentImage;

    /* Open parrot camera instead */
    
    //establishing connection with the quadcopter
    // heli = new CHeli();
    
    //this class holds the image from the drone 
    //  image = new CRawImage(320,240);
    

  // init currentImage
    //Mat currentImage = Mat(240, 320, CV_8UC3);



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
		camera >> currentImage;

        /* 2 Obtain image from Parrot instead */
        //image is captured
     //  heli->renewImage(image);
        // Copy to OpenCV Mat
     // rawToMat(currentImage, image);
        

		if (currentImage.data) 
		{
            /* Show image */
            imshow("Video", currentImage);
            
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
        
        if (key == 10 && storedImage.rows > 0) 
        { //executes on enter
            generateBlackWhite1C(storedImage, blackAndWhiteImage);
           // greyScalePointReady = true;  
            namedWindow("Gray Scale Image");
            cvMoveWindow("Gray Scale Image", XIMAGE+200, YIMAGE+200);
            imshow("Gray Scale Image", blackAndWhiteImage);

            generateGrayScale(grayScaleImage);
            namedWindow("Grey Scale");
            cvMoveWindow("Grey Scale",XIMAGE,YIMAGE+100);
            imshow("Grey Scale", grayScaleImage);
            setMouseCallback("Grey Scale", mouseCallbackForBinaryImage);

            //initialization
            greyScalePoint.x = 200; //catches a 0.5 level in grey scale
            greyScalePoint.y = 0;
            greyScalePointReady = true;
            orig.x = dest.x = 0;
            }
               

        if (greyScalePointReady && blackAndWhiteImage.data) 
        {   //executes on click on greyscale window
            //in grayScaleImage all channels are equal, pick only one channel
            unsigned char value = grayScaleImage.at<Vec3b>(greyScalePoint.y, greyScalePoint.x)[0];
            cout << "Gray Scale value: " << (int)value << endl;
            generateBinary(blackAndWhiteImage, value, binaryImage);
            greyScalePointReady = false;  

            //Closing is obtained by the dilation of an image followed by an erosion. 
            Mat dilateFilter;
            dilate(binaryImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            imshow("closing", closing);   
        

        }
        
        if(key == 'r' && storedImage.data)
        {
            imshow("Image",backupImage);
            storedImage = backupImage;
        }

        if(key == 'h' && storedImage.data && (orig.x != dest.x))
        {
            Mat dilateFilter; //Declarar la variable aqui para que tambien se pueda usar la funcion del enter

            generaimagenFiltradaBinaria(storedImage, filteredImage,HSV);
            dilate(filteredImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            imshow("Image", closing);   

        /* NOTA: PROBAR CON FILTROS DE PASO BAJO SI MEJORA EL FILTRADO

            //medianBlur
            Mat medianBlurImage;
            medianBlur(storedImage, medianBlurImage, 5);
            generaimagenFiltradaBinaria(medianBlurImage, filteredImage,HSV);
            dilate(filteredImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            imshow("Filtrado mediana", closing);   

            //AverageBlur
            Mat AverageBlurImage;
            blur(storedImage, AverageBlurImage, Size(5,5), Point(-1,-1));
            generaimagenFiltradaBinaria(AverageBlurImage, filteredImage,HSV);
            dilate(filteredImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            imshow("Filtrado average", closing);

            // GaussianBlur
            Mat GaussianImage;
            GaussianBlur(storedImage, GaussianImage, Size(5,5), 0, 0);
            generaimagenFiltradaBinaria(GaussianImage, filteredImage,HSV);
            dilate(filteredImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            imshow("Filtrado Gaussian", closing);   
        */

        }


        if (key=='s' && closing.rows > 0)
        {
        Mat oil;
        OilDrop(closing, oil);
        imshow("oildrop", oil);
        }


        /*************************************/
        key = waitKey(5);
	}
} //main

void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param)
{
    static bool clicked=false;
    static char wname[5] = {'B', 0, 'G', 0, 'R'};

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

void mouseCallbackForBinaryImage(int event, int x, int y, int flags, void* param) 
{
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            greyScalePoint.x = x;
            greyScalePoint.y = y;
            greyScalePointReady = true;
            break;
        default:
            break;
    }
} //mouseCallbackForBinaryImage

void generateBlackWhite(const Mat &origin, Mat &destination) 
{
    int average;

    if (destination.empty())
        destination = Mat(origin.rows, origin.cols, origin.type());

    int channels = origin.channels();
    
    for (int y = 0; y < origin.rows; ++y) 
    {
        uchar* sourceRowPointer = (uchar*) origin.ptr<uchar>(y);
        uchar* destinationRowPointer = (uchar*) destination.ptr<uchar>(y);
        for (int x = 0; x < origin.cols; ++x) {
            average = ( sourceRowPointer[x * channels] + sourceRowPointer[x * channels + 1] + sourceRowPointer[x * channels + 2] ) / 3;
            destinationRowPointer[x * channels    ] = (unsigned char)average;
            destinationRowPointer[x * channels + 1] = (unsigned char)average;
            destinationRowPointer[x * channels + 2] = (unsigned char)average;
        }
    }

} //generateBlackWhite

void generateBlackWhite1C(const Mat &origin, Mat &destination) 
{
    int average;

    if (destination.empty())
        destination = Mat(origin.rows, origin.cols, CV_8UC1);

    int channels = origin.channels();
    
    for (int y = 0; y < origin.rows; ++y) 
    {
        uchar* sourceRowPointer = (uchar*) origin.ptr<uchar>(y);
        uchar* destinationRowPointer = (uchar*) destination.ptr<uchar>(y);
        for (int x = 0; x < origin.cols; ++x) {
            average = ( sourceRowPointer[x * channels] + sourceRowPointer[x * channels + 1] + sourceRowPointer[x * channels + 2] ) / 3;
            destinationRowPointer[x] = (unsigned char)average; 
            //Solamente escribe el promedio en el unico canal de la imagen
        }
    }
}//generate blackwhite

void generateBinary(const Mat &origin, const unsigned char limit, Mat &destination) 
{
    if (destination.empty())
        destination = Mat(origin.rows, origin.cols, origin.type());

    int channels = origin.channels();
    
    for (int y = 0; y < origin.rows; ++y) 
    {
        uchar* sourceRowPointer = (uchar*) origin.ptr<uchar>(y);
        uchar* destinationRowPointer = (uchar*) destination.ptr<uchar>(y);
        for (int x = 0; x < origin.cols; ++x) {
            for (int i = 0; i < channels; ++i) {
                if (sourceRowPointer[x * channels + i] < limit )
                    destinationRowPointer[x * channels + i] = 0;
                else
                    destinationRowPointer[x * channels + i] = 255;
            }
        }
    }
} //generateBinary



void generateGrayScale(Mat &Matrix) 
{
    Matrix.create(25,255,CV_8UC3);

    int channels = Matrix.channels();
    
    for (int y = 0; y < Matrix.rows; ++y) 
    {
        uchar* rowPointer = (uchar*) Matrix.ptr<uchar>(y);
        for (int x = 0; x < Matrix.cols; ++x)
            for (int i = 0; i < channels; ++i)
            {
                rowPointer[x * channels + i] = x;
            }
    }
} //generateGrayScale

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

Vec3b getK(int &x) { 

    Vec3b vector = ColorMat[x];
    x++;
    if (x >= maxColors)
        x = 0;
    return vector;
}

//Imagen binaria, Imagen destino
void OilDrop(const Mat &dst, Mat &colorDst) {
    Vec3b k;
    uchar aux;
    int colorIndex=0;
    Vec3b negro=Vec3b(0, 0, 0); //Use to compare with black color
    
    int state=1;  //Variable for state machine for exploring
    int factor; // Variable to explore in spiral
    int delta;

    #define maxAreas 10
    int m00[maxAreas]; //Variable to get the size of the blobs
    long m10[maxAreas];
    long m01[maxAreas];
    long m20[maxAreas];
    long m02[maxAreas];
    long m11[maxAreas];
    double u11[maxAreas];
    double u20[maxAreas];
    double u02[maxAreas];
    double angle[maxAreas];
    double xtest[maxAreas];
    double ytest[maxAreas];
    Point centroid[maxAreas];


    if (colorDst.empty())
        colorDst = Mat(dst.rows, dst.cols,  CV_8UC3);
    colorDst=Scalar::all(0);  //Start with the empty image
    
    for (int nareas = 0; nareas < 8; nareas++) { //color 5 areas max
        int seedCol = dst.cols/2;
        int seedRow = dst.rows/2;
        Point seed = Point(seedCol, seedRow);   //first seed, on the middle of the image

        int c = 30;
        factor = dst.cols / 20;
        delta = factor;

        do {

            switch(state)
            {   
                case 1:
                seedRow += delta;  //move right
                delta += factor;   //increase delta
                state = 2;
                break;
                
                case 2:
                seedCol -= delta;  //move up
                delta += factor;   //increase delta
                state = 3;
                break;
                
                case 3:
                seedRow -= delta;  //move left
                delta += factor;   //increase delta
                state = 4;
                break;
                
                case 4:
                seedCol += delta;  //move down
                delta += factor;   //increase delta
                state = 1;
                break;
                
                case 5:
                seedCol = rand() % dst.cols;    //random
                seedRow = rand() % dst.rows;
                c--;
                break;
            }

            if(seedRow >= dst.rows || seedRow < 0)
            {
                seedRow = dst.rows/2;
                state = 5;  //Change to random
            }

            if(seedCol >= dst.cols || seedCol < 0)
            {
                seedCol = dst.cols/2;
                state = 5;  //Change to random
            }

            seed = Point(seedCol, seedRow); //Generate seed

      
        } while ((dst.at<uchar>(seed) != 255 || (colorDst.at<Vec3b>(seed) != negro) ) && c > 0); //find a white starting point, if it doesn't find one in c chances, skip

        if (c > 0) {
        
            k = getK(colorIndex);
            queue<Point> Fifo;
            Fifo.push(seed);

            colorDst.at<Vec3b>(seed) = k;   // color the seed pixel
            m00[colorIndex-1] = 1;
            m10[colorIndex-1] = seed.x;
            m01[colorIndex-1] = seed.y;
            m20[colorIndex-1] = seed.x * seed.x;
            m02[colorIndex-1] = seed.y * seed.y;
            m11[colorIndex-1] = seed.x * seed.y;


            Point coord;
            
            while (!Fifo.empty()) {
                for (int state = 0; state < 4; state++) {   //check all 4 sides
                    switch (state) {
                    case 0: //north
                        coord.x = Fifo.front().x;
                        coord.y = Fifo.front().y - 1;
                        break;
                    case 1: //west
                        coord.x = Fifo.front().x - 1;
                        coord.y = Fifo.front().y;
                        break;
                    case 2: //south
                        coord.x = Fifo.front().x;
                        coord.y = Fifo.front().y + 1;
                        break;
                    case 3: //east
                        coord.x = Fifo.front().x + 1;
                        coord.y = Fifo.front().y;
                        break;
                    }

                    if (coord.y >= 0 && coord.y < dst.rows && coord.x >= 0 && coord.x < dst.cols) { //is in range
                        Vec3b temp = colorDst.at<Vec3b>(coord);
                        if (temp != k) {                            //AND is not colored with k on the destination yet
                         //   aux = dst.at<uchar>(coord);
                            if (dst.at<uchar>(coord) != 0) {        //AND is not 0 in the binary image
                                colorDst.at<Vec3b>(coord) = k;      //color the destination
                                Fifo.push(Point(coord));            //enqueue this point for analysis
                                m00[colorIndex-1] += 1;
                                m10[colorIndex-1] += coord.x;
                                m01[colorIndex-1] += coord.y;
                                m20[colorIndex-1] += coord.x * coord.x;
                                m02[colorIndex-1] += coord.y * coord.y;
                                m11[colorIndex-1] += coord.x * coord.y;
                            }
                        }
                    }
                }

                Fifo.pop();
            }

        }
    }
        
    for (int i=0;i<colorIndex;i++) {
        xtest[i]=(double) m10[i]/m00[i];
        ytest[i]=(double) m01[i]/m00[i];
        u20[i]=m20[i]-m10[i]*xtest[i];
        u02[i]=m02[i]-m01[i]*ytest[i];
        u11[i]=m11[i]-m00[i]*xtest[i]*ytest[i];
        angle[i]=(1.0/2.0)*(atan2(2*u11[i], u20[i]-u02[i]));

        centroid[i]=Point(xtest[i], ytest[i]);

        cout<<"Region"<< i+1 << "\tValor:"<<endl;
        cout << "m00" << "\t" << m00[i] << endl;
        cout << "m10" << "\t" << m10[i] << endl;
        cout << "m01" << "\t" << m01[i] << endl;
        cout << "m20" << "\t" << m20[i] << endl;
        cout << "m02" << "\t" << m02[i] << endl;
        cout << "m11" << "\t" << m11[i] << endl;
        cout<< "xtest" << "\t" << xtest[i] << endl;
        cout<< "ytest" << "\t" << ytest[i] << endl;
        cout << "u20" << "\t" << u20[i] << endl;
        cout << "u02" << "\t" << u02[i] << endl;
        cout << "u11" << "\t" << u11[i] << endl;
        cout << "angle" << "\t" << angle[i] << endl;

        cout << endl;
        }

} //End Oil Drop

void generaimagenFiltradaBinaria (const Mat &sourceImage, Mat &destinationImage, int type)
{
    Vec3b maxVec, minVec;
    bool gray;
    int tolerance, maxValue, minValue;

    int startY, endY, startX, endX;

    int channels = sourceImage.channels();
    


    //transformar a YIQ o HSV si es necesario
    Mat transformedImage;
    switch(type)
    {
                    
        case YIQ:
                    transformedImage = YIQImage;
                    generateYIQ(sourceImage,transformedImage);
                    break;
                
        case HSV:
                    transformedImage = HSVImage;
                    generateHSV (sourceImage,transformedImage);
                    break;
        default:
                    transformedImage=sourceImage;
    }

    maxValue = 255;
    minValue = 0;
    tolerance = 3;
    
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

        if ((minVec[i] - tolerance) < minValue)
            minVec[i] = minValue;
        else
            minVec[i] -= tolerance;
        
        if ((maxVec[i] + tolerance) > maxValue)
            maxVec[i] = maxValue;
        else
            maxVec[i] += tolerance;
    }


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

void generateYIQ (const Mat &origin, Mat &destination)   
{
    double blue, red, green;
    float yq, iq, qq;

    if (destination.empty())
        destination= Mat(origin.rows, origin.cols, origin.type());

    int channels = origin.channels();
    
    for (int y = 0; y < origin.rows; ++y) 
    {
        uchar* sourceRowPointer = (uchar*) origin.ptr<uchar>(y);
        uchar* destinationRowPointer = (uchar*) destination.ptr<uchar>(y);
        for (int x = 0; x < origin.cols; ++x)
        {
            blue= sourceRowPointer[x * channels]/255.0;
            green = sourceRowPointer[x * channels + 1]/255.0;
            red = sourceRowPointer[x * channels + 2]/255.0;
            
            yq = 0.299*red + 0.587*green + 0.114*blue;
            iq = 0.596*red - 0.275*green - 0.321*blue;
            qq = 0.212*red - 0.523*green + 0.311*blue;

            /*
                y is a value from 0 to 1
                i is a value from -0.596 to 0.596, difference = 1.192
                q is a value from -0.523 to 0.523, difference = 1.046

                adjustment must be made
            */

            iq = (iq + 0.596) / 1.192;
            qq = (qq + 0.523) / 1.046;

            destinationRowPointer[x * channels    ] = yq * 255;
            destinationRowPointer[x * channels + 1] = iq * 255;
            destinationRowPointer[x * channels + 2] = qq * 255;
        }
    }
} //generateYIQ

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

void showHistogram(Mat& img, const char* wname, const int sourceX, const int sourceY)
{
    int bins = 256;             // number of bins
    int nc = img.channels();    // number of channels

    vector<Mat> hist(nc);       // histogram arrays

    // Initalize histogram arrays
    for (unsigned int i = 0; i < hist.size(); i++)
        hist[i] = Mat::zeros(1, bins, CV_32SC1);

    // Calculate the histogram of the image
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            for (int k = 0; k < nc; k++)
            {
                uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
                hist[k].at<int>(val) += 1;
            }
        }
    }

    // For each histogram arrays, obtain the maximum (peak) value
    // Needed to normalize the display later
    int hmax[3] = {0,0,0};
    int hgeneralmax = 0;
    for (int i = 0; i < nc; i++)
    {
        for (int j = 0; j < bins-1; j++) {
            hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
            hgeneralmax = hist[i].at<int>(j) > hgeneralmax ? hist[i].at<int>(j) : hgeneralmax;
        }
    }
    /*    //adding the following FOR makes the height proportional to the absolute max of all channels 
    for (int i = 0; i < nc; i++)
    {   
        hmax[i] = hgeneralmax;
    }
    */

    //const char* wname[3] = { "Channel0", "Channel1", "Channel2" };
    Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };

    vector<Mat> canvas(nc);

    // Display each histogram in a canvas
    for (int i = 0; i < nc; i++)
    {
        canvas[i] = Mat::ones(125, bins, CV_8UC3);

        for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
        {
            line(
                canvas[i], 
                Point(j, rows), 
                Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])), 
                nc == 1 ? Scalar(200,200,200) : colors[i], 
                1, 8, 0
            );
        }

        imshow(nc == 1 ? "value" : &wname[i*2], canvas[i]);
        cvMoveWindow(nc == 1 ? "value" : &wname[i*2], sourceX+650,sourceY+i*155);
    }
} //showHistogram

void showHistogram(Mat& img, const char* wname, const int sourceX, const int sourceY, const Vec3b components)
{
    int bins = 256;             // number of bins
    int nc = img.channels();    // number of channels

    vector<Mat> hist(nc);       // histogram arrays

    // Initalize histogram arrays
    for (unsigned int i = 0; i < hist.size(); i++)
        hist[i] = Mat::zeros(1, bins, CV_32SC1);

    // Calculate the histogram of the image
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            for (int k = 0; k < nc; k++)
            {
                uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
                hist[k].at<int>(val) += 1;
            }
        }
    }

    // For each histogram arrays, obtain the maximum (peak) value
    // Needed to normalize the display later
    int hmax[3] = {0,0,0};
    int hgeneralmax = 0;
    for (int i = 0; i < nc; i++)
    {
        for (int j = 0; j < bins-1; j++) {
            hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
            hgeneralmax = hist[i].at<int>(j) > hgeneralmax ? hist[i].at<int>(j) : hgeneralmax;
        }
    }

    //const char* wname[3] = { "Channel0", "Channel1", "Channel2" };
    Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };

    vector<Mat> canvas(nc);

    // Display each histogram in a canvas
    for (int i = 0; i < nc; i++)
    {
        canvas[i] = Mat::ones(125, bins, CV_8UC3);

        for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
        {
            line(
                canvas[i], 
                Point(j, rows), 
                Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])), j == components[i] ? Scalar(255,255,255) : colors[i], 
                1, 8, 0
            );
        }

        imshow(nc == 1 ? "value" : &wname[i*2], canvas[i]);
        cvMoveWindow(nc == 1 ? "value" : &wname[i*2], sourceX+650,sourceY+i*155);
    }
} //showHistogram

