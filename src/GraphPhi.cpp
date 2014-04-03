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
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>

#define YIMAGE 20
#define XVIDEO 1250
#define ROWS 500
#define COLS 500

using namespace cv;
using namespace std;

void graphPhis (double, double);

Mat canvas(ROWS, COLS, CV_8UC3, Scalar(0));
ifstream inFile;

vector<Scalar> vecs;
vector<Scalar>::iterator it;

int main(int argc, char *argv[])
{
    string address;
    double phi1, phi2, var1, var2;
    vecs.push_back(Scalar(255, 0, 0 ,0));      //blue
    vecs.push_back(Scalar(0, 255, 0 ,0));      //green
    vecs.push_back(Scalar(0, 0, 255 ,0));     //red
    vecs.push_back(Scalar(255, 255, 0 ,0));    //cyan
    vecs.push_back(Scalar(0, 255, 255 ,0));    //yellow
    vecs.push_back(Scalar(255, 0, 255 ,0));    //magenta
    vecs.push_back(Scalar(0, 154, 255 ,0));    //orange
    vecs.push_back(Scalar(0, 128, 128 ,0));    //olive
    vecs.push_back(Scalar(0, 100, 0 ,0));      //darkgreen
    vecs.push_back(Scalar(128, 0, 0 ,0));       //navy
    it = vecs.begin();

    if (argc > 1) {
        address = argv[1];
    } else {
        address = "../src/main/data/philist.txt";
    }
    inFile.open(address.c_str());

    int i = 0;
    char c;
    while (!inFile.eof()) {
            inFile >> phi1;
            inFile >> phi2;
            inFile.get(); //eat 1 \r
            cout << "Read:\t";
            cout << phi1 << ' ' << phi2 << '\n';

            graphPhis (phi1, phi2);

            if (inFile.peek() == '\r') {
                it++;
                if (it == vecs.end())
                    it = vecs.begin();
                inFile.get(); //eat 1 \r
                cout << '\n';
            }

            while ( (inFile.peek() < '0' || inFile.peek() > '9') && !inFile.eof()) {
                inFile.get();
            }
    }

    char key = 0;
    while (key != 27) {
        key = waitKey(5);
    }

    return 0;
} //main

void graphPhis (double phi1, double phi2) {
    
    // Phi1 are X, Phi2 are Y
    Point P;
    P = Point( (int)(phi1 * COLS), (int)(ROWS - phi2 * ROWS) );
    //cout << P.x << ' ' << ROWS - P.y << '\n'; //debug
    circle(canvas, P, 3, *it, CV_FILLED);
    if (it == vecs.end()) it = vecs.begin();
    
    imshow("Phi Graph", canvas);
    cvMoveWindow("Phi Graph", XVIDEO, YIMAGE + 500);
}