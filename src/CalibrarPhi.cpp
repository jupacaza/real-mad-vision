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
#include <string>
#include "CHeli.h"

#define XIMAGE 0
#define YIMAGE 20
#define XVIDEO 1250

#define maxColors 10
#define maxAreas 10

#define PARROT 0    // TURN PARROT ON (1) or OFF (0)

using namespace cv;
using namespace std;

// Here we will store points
vector<Point> points;
Point orig, dest;

/* Create images where captured and transformed frames are going to be stored */
Mat currentImage;
Mat storedImage, backupImage;
Mat filteredImage;
Mat HSVImage;
Mat closing;
//for parrot use
CHeli *heli;
CRawImage *image;

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


void rawToMat( Mat &destImage, CRawImage* sourceImage);

Vec3b getK(int &x);

void OilDrop(const Mat &dst, Mat &colorDst);

void generaBinariaDeArchivo (const Mat &origin, Mat &destination);
void generateHSV(const Mat &origin, Mat &destination);
void graphPhis (double PhisArray[][2], int savedPhis);

ofstream outFile, phiListFile;
ifstream inFile;
Vec3b maxVec, minVec;
double phi1[maxAreas], phi2[maxAreas];
double refPhis[4][4];
//test
//Moments cvHuMoments;
//double cvPhis[7];

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

    // read color limits from file
    inFile.open("../src/main/data/limits.txt");
    inFile >> minVec[0];
    inFile >> minVec[1];
    inFile >> minVec[2];
    inFile >> maxVec[0];
    inFile >> maxVec[1];
    inFile >> maxVec[2];
    inFile.close();

    cout << "Read:\n";
    cout << (int)minVec[0] << ' ' << (int)minVec [1] << ' ' << (int)minVec [2] << '\n';
    cout << (int)maxVec[0] << ' ' << (int)maxVec [1] << ' ' << (int)maxVec [2] << '\n';

    int savedPhis = 0, lastSavedPhis = 0;
    double PhisArray[15][2];
    phi1[0] = -1;

    inFile.open("../src/main/data/phi.txt");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            inFile >> refPhis[i][j];
        }
    }
    inFile.close();

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
            //namedWindow("Image");
            //cvMoveWindow("Image", XIMAGE, YIMAGE);
            //imshow("Image", storedImage);

            Mat dilateFilter; //Declarar la variable aqui para que tambien se pueda usar la funcion del enter
            generaBinariaDeArchivo(storedImage, filteredImage);
            dilate(filteredImage, dilateFilter, Mat (10, 10, CV_8U));
            erode(dilateFilter, closing, Mat (10, 10, CV_8U));
            //imshow("Image", closing);

            Mat oil;
            OilDrop(closing, oil);
            imshow("oildrop", oil);
        }

        if (key == 's' && phi1[0] != -1) { //save phi to array
            //only saves the phis of the blue area

            if (savedPhis < 15) {
                PhisArray[savedPhis][0] = phi1[0];
                PhisArray[savedPhis++][1] = phi2[0];
                cout << "Phis of Sample " << savedPhis << " were saved:\n";
                cout << "\tPhi1: " << PhisArray[savedPhis-1][0] << "\tPhi2: " << PhisArray[savedPhis-1][1] << '\n';
            } else {
                cout << "Max number of saved Phis (" << savedPhis << ") reached\n";
            }

        } else if (key == 's') {
            cout << "Phi 1 and 2 not yet captured" << '\n';
        }

        if (key == 'd' && savedPhis > 0) {
            savedPhis--;
            cout << "Deleted last pair of Phis\n";
        } else if (key == 'd') {
            cout << "No Phis to delete\n";
        }


        if (key == '1' && savedPhis > 0) { //save phis to files
/*
            char option;
            cout << "Saving Phi to new file or existing file? (n/e) ";
            option = getchar();

            if (option == 'n') {
                outFile.open("../src/main/data/phi.txt");
                phiListFile.open("../src/main/data/philist.txt");
            } else {
 */               outFile.open("../src/main/data/phi.txt", ios::app);
                phiListFile.open("../src/main/data/philist.txt", ios::app);
 //           }

            double sum1 = 0, sum2 = 0;
            double average1 = 0, average2 = 0;
            for (int i = 0; i < savedPhis; i++) {
                sum1 += PhisArray[i][0];
                sum2 += PhisArray[i][1];
                phiListFile << PhisArray[i][0] << ' ' << PhisArray[i][1] << '\r';
            }
            phiListFile << '\r';
            phiListFile.close();
            average1 = sum1 / savedPhis;
            average2 = sum2 / savedPhis;

            // Variance
            double variance1 = 0, variance2 = 0;
            for (int i = 0; i < savedPhis; i++) {
                variance1 += pow(PhisArray[i][0], 2);
                variance2 += pow(PhisArray[i][1], 2);
            }
            variance1 /= savedPhis;
            variance2 /= savedPhis;
            variance1 -= pow(average1, 2);
            variance2 -= pow(average2, 2);

            outFile << average1 << ' ' << average2 << ' ' << variance1 << ' ' << variance2 << '\n';
            outFile.close();

            cout << "--Phi 1 and 2 averages were saved to the output file--\n\n";
        } else if (key == '1') {
            cout << "Phis not yet saved, use key '1' to save captured Phis\n";
        }

        if (savedPhis != lastSavedPhis) {
            
            graphPhis(PhisArray, savedPhis);

            lastSavedPhis = savedPhis;
        }


        /*************************************/
        key = waitKey(5);
	}
} //main

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
    //uchar aux;
    int colorIndex=0;
    Vec3b negro=Vec3b(0, 0, 0); //Use to compare with black color
    
    int state=1;  //Variable for state machine for exploring
    int factor; // Variable to explore in spiral
    int delta;

    int m00[maxAreas]; //Variable to get the size of the blobs
    long m10[maxAreas];
    long m01[maxAreas];
    long m20[maxAreas];
    long m02[maxAreas];
    long m11[maxAreas];
    double u11[maxAreas];   //u00 = m00
    double u20[maxAreas];
    double u02[maxAreas];
    double angle[maxAreas];
    double xtest[maxAreas];
    double ytest[maxAreas];

    double n20[maxAreas];
    double n02[maxAreas];
    double n11[maxAreas];

    Point centroid[maxAreas];
/*
    cvHuMoments = moments(dst, true);
    HuMoments(cvHuMoments, cvPhis);
*/
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

        n20[i] = u20[i] / pow(m00[i], 2); // power = p+q/2 +1
        n02[i] = u02[i] / pow(m00[i], 2);
        n11[i] = u11[i] / pow(m00[i], 2);

        phi1[i] = n20[i] + n02[i];
        phi2[i] = pow(n20[i] - n02[i], 2) + 4 * pow(n11[i], 2);


        cout<<"Region"<< i+1 << "\tValor:"<<endl;
        /*
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
        */
        cout << "PHI 1" << "\t" << phi1[i] << endl;
        cout << "PHI 2" << "\t" << phi2[i] << endl;

    //test
        //cout << "cvPHI 1" << "\t" << cvPhis[0] << endl;
        //cout << "cvPHI 2" << "\t" << cvPhis[1] << endl;

        cout << endl;
        }

} //End Oil Drop

void generaBinariaDeArchivo (const Mat &sourceImage, Mat &destinationImage)
{
    bool gray;

    int channels = sourceImage.channels();

    Mat transformedImage;
    transformedImage = HSVImage;
    generateHSV (sourceImage,transformedImage);
    
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

void graphPhis (double PhisArray[][2], int savedPhis) {
    #define ROWS 500
    #define COLS 500

    Mat canvas(ROWS, COLS, CV_8UC1, Scalar(0,0,0));

    // Phi1 are X, Phi2 are Y
    Point P;
    for (int i = 0; i < savedPhis; i++) {
        P = Point( (int)(PhisArray[i][0]*COLS), (int)(ROWS - PhisArray[i][1] * ROWS) );
        //cout << P.x << ' ' << ROWS - P.y << '\n'; //debug
        circle(canvas, P, 3, Scalar( 255, 255, 255), CV_FILLED);
    }
    imshow("Phi Graph", canvas);
    cvMoveWindow("Phi Graph", XVIDEO, YIMAGE + 500);
}