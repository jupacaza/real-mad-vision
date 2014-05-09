#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
#include "CHeli.h"
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <queue>
#include <fstream>
#include <string>
#include <climits>
#define _USE_MATH_DEFINES
#define XIMAGE 0
#define YIMAGE 20
#define XVIDEO 1250

#define maxColors 10
#define maxAreas 10

#define DERECHA 1
#define IZQUIERDA 2
#define ADELANTE 3
#define ATRAS 4

#define GROWS 500
#define GCOLS 500

using namespace cv;
using namespace std;

/***************** GLOBALS ***********************/
Mat currentImage;
Mat storedImage, backupImage;
Mat filteredImage;
Mat HSVImage;
Mat closing;
Mat mapaOrig;
Mat mapaEnsanchado;

vector<Point> points;
Point orig, dest;

Point mapPoints[20];
int Npoints=0;

unsigned char PARROT = 1;     // TURN PARROT ON (1) or OFF (0)
CHeli *heli;
CRawImage *image;
bool stop = false;


//************Joystick related************//

int hover,joyRoll, joyPitch;
float verticalSpeed, yaw;
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadVerticalSpeed, joypadYaw;
bool PLUSjoypadRoll, MINUSjoypadRoll, PLUSjoypadPitch, MINUSjoypadPitch, navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover, joypadAuto, joypadManual;
bool automatico=true;  //Variable que habilita el control manual o automatico
//*************************//

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

ofstream outFile, phiListFile;
ifstream inFile;
Vec3b maxVec, minVec;
double phi1[maxAreas], phi2[maxAreas];
double refPhis[4][4];

// **Start timer **
clock_t endwait;
int move1 = 0, move2 = 0;
int state = -1;
bool begin;

Mat canvas(GROWS, GCOLS, CV_8UC3, Scalar(0));
vector<Scalar> vecs;
vector<Scalar>::iterator it;

int NumberRegions;
/*************************************************/

/***************** HEADERS ***********************/
void rawToMat( Mat &destImage, CRawImage* sourceImage);
Vec3b getK(int &x);
void OilDrop(const Mat &dst, Mat &colorDst);
void generaBinariaDeArchivo (const Mat &origin, Mat &destination);
void generateHSV(const Mat &origin, Mat &destination);
void graphPhis (double PhisArray[][2], int savedPhis);
void graphPhis (double, double);

void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param);


void generaimagenFiltradaBinaria (const Mat &origin, Mat &destination);
void Rellenar(Mat &colorDst, Point seed);
void drawRotatedSquare(Mat &origin, Mat &mapaEnsanchado, int degree,Point center, Point verticesRotated[4]);

int connectedPoint(Point p1, Point p2)
{
  
    bool conecta=true;
    int ax, ay;
    ax=p2.x-p1.x;
    ay=p2.y-p1.y;
    float dx, dy;
    dx=ax/20.0;
    dy=ay/20.0;
    float x,y;
    x=p1.x;
    y=p1.y;

    for (int it=0; (it<20 && conecta); it++ )
    {
        x=x+dx;
        y=y+dy;

        
        if (mapaEnsanchado.at<Vec3b>(Point((int)x,(int)y))==Vec3b(255,0,0))
            conecta=false;
    }

    if(!conecta)
        return INT_MAX;

    return (int)(sqrt(ax*ax + ay*ay));

} // END connected points
void mapCallback(int event, int x, int y, int flags, void* param)
{

    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            if(Npoints < 18)
            {
                if (mapaEnsanchado.at<Vec3b>(Point(x,y))!=Vec3b(255,0,0))
                {
                    cout << "  Mouse X, Y: " << x << ", " << y << endl;
                    mapPoints[Npoints]=(Point(x, y));
                    Npoints++;
                    circle(mapaEnsanchado,Point(x,y), 3, Scalar(0,255,0),-1);
                    imshow("Mapa Ensanchado", mapaEnsanchado);
                }
                else
                    cout<< "ERROR ES UN OBSTACULO"<< endl;

            } else 
                cout << "ERROR LIMITE DE PUNTOS"<<endl;
            

            break;
       
    }
}// END mapCallback

void dijkstra(int w[20][20], int n/*, int l[20], int t[20]*/)
{
    int l[20],  t[20];
    //int f[20];

    for(int i = 1; i < n; i++)
    {
        t[i] = 0;
        l[i] = w[0][i];
        //f[i] = l[i];
    }

    for(int x = 1; x <= n - 1; x++)
    {
        int vnear, min = INT_MAX;
        for(int i = 1; i < n; i++)
        {
            if(0 <= l[i]  && l[i] < min)
            {
                min = l[i]; //closest distance
                vnear = i;  //closest node
            }
        }

        for(int i = 1; i < n; i++)
        {
            if( (l[vnear] + w[vnear][i]) >= 0 &&  (l[vnear] + w[vnear][i]) < l[i] )
            {
                l[i] = l[vnear] + w[vnear][i];
                t[i] = vnear;

                //f[i] = l[i];    //add path from t[vnear] to vnear
            }
        }

        l[vnear] = -1;
    }

    //cout << "distance to " << n << " : " << f[n - 1] << endl;
    //cout << "path: " << endl;

    int x = n - 1;

    do
    {

        line(mapaEnsanchado, mapPoints[x], mapPoints[t[x]], Scalar(0,0,255),1,8);
        x = t[x];

    }while(x != 0);

    imshow("Mapa Ensanchado", mapaEnsanchado);
}

/*************************************************/

/****************** MAIN *************************/
int main(int argc, char *argv[]) {

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
    cout << (int)minVec[0] << ' ' << (int)minVec[1] << ' ' << (int)minVec[2] << '\n';
    cout << (int)maxVec[0] << ' ' << (int)maxVec[1] << ' ' << (int)maxVec[2] << '\n';
    // read phi 1 & 2 averages and variations from file
    inFile.open("../src/main/data/phi.txt"); 
    for (int i = 0; i < 4; i++) { 
        for (int j = 0; j < 4; j++) { 
                inFile >> refPhis[i][j]; 
            } 
    } 
    inFile.close();

	char MainFunction = 0;
    
	while (MainFunction != 'q') { // q de quit
        cout << "\033[2J\033[1;1H";     // Clear Console
        //cvDestroyAllWindows();            // Clear the GUI (close all windows) 
		cout << "--- Select the function you want to use:\n";
        cout << "\t1. Fly Parrot with saved parameters\n";
        cout << "\t2. Calibrate Filter values (limits)\n";
        cout << "\t3. Calibrate Hu Moments (Phi) for flight control (phi, philist)\n";
        cout << "\t4. Graph saved moments in philist\n";
        cout << "\t5. Show the map\n";
        cout << "Choose and press <ENTER>: ";
		MainFunction = getchar();

        unsigned char key = 0;
        VideoCapture camera;
        int savedPhis, lastSavedPhis;
        double PhisArray[15][2];

        string address;

        Point center;
        int angleObstacle;
        Point vertices[4];


        switch (MainFunction) {


        /*   Ver mapa ensanchado       */

            case '5':
           
            mapaOrig = imread("../src/main/espacio.png");   // Read the file
            mapaEnsanchado=mapaOrig.clone();
          //Son 746 Renglones X 725 Columnas
            //Drawing origin
            circle(mapaEnsanchado,Point(362, 125),10,Scalar(0,255,0),-1);
            //Drawing target
            circle(mapaEnsanchado,Point(362, 650),10,Scalar(0,255,0),-1);
            //Drawing an obstacle
            center=Point(362, 525);
            drawRotatedSquare(mapaOrig, mapaEnsanchado, 0 , center, vertices);

            cout<<"Obstacle 1 angle (degree)"<<endl;
            cin>>angleObstacle;

            center=Point(362, 325);
            drawRotatedSquare(mapaOrig, mapaEnsanchado, angleObstacle, center, vertices);
            
            for (int i=0; i<4;i++)
            cout<<"vertice ["<<i<<"]= "<< vertices[i].x<<" "<< vertices[i].y<<endl;
            

            //imshow("Mapa", mapaOrig);
            
    
            namedWindow("Mapa Ensanchado");
            imshow("Mapa Ensanchado", mapaEnsanchado);
            mapPoints[0] = Point(362, 125);
            Npoints=1;
            setMouseCallback("Mapa Ensanchado", mapCallback);


           //Escape Sequence
            waitKey(0);

            mapPoints[Npoints] = Point(362, 650);
            Npoints++;

            int ady[20][20];

            for(int i = 0; i < Npoints; i++)
                ady[i][i] = 0;

            for(int i = 0; i < Npoints; i++)
            {
                for(int j = i + 1; j < Npoints; j++)
                {
                    ady[i][j] = connectedPoint(mapPoints[i],mapPoints[j]);
                    ady[j][i] = ady[i][j];

                    if(ady[i][j] != INT_MAX)
                    {   
                        line(mapaEnsanchado, mapPoints[i], mapPoints[j], Scalar(0,255,0),1,8);
                        
                    }            
                
                }
            }
            imshow("Mapa Ensanchado", mapaEnsanchado);

            waitKey(0);
            dijkstra(ady,Npoints);

            waitKey(0);
            //CleanUp
            mapaOrig.release();
            mapaEnsanchado.release();
            destroyAllWindows();

            break;
/*****************************************************/
/*        VUELO DEL PARROT                           */
/*****************************************************/

            case '1':
                //establishing connection with the quadcopter
                heli = new CHeli();
                //this class holds the image from the drone 
                image = new CRawImage(320,240);
                // Destination OpenCV Mat   
                currentImage = Mat(240, 320, CV_8UC3);
                // Show it  
                namedWindow("Video");

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
                cout << (int)minVec[0] << ' ' << (int)minVec[1] << ' ' << (int)minVec[2] << '\n';
                cout << (int)maxVec[0] << ' ' << (int)maxVec[1] << ' ' << (int)maxVec[2] << '\n';
                // read phi 1 & 2 averages and variations from file
                inFile.open("../src/main/data/phi.txt"); 
                for (int i = 0; i < 4; i++) { 
                    for (int j = 0; j < 4; j++) { 
                            inFile >> refPhis[i][j]; 
                        } 
                } 
                inFile.close();

                savedPhis = 0;
                lastSavedPhis = 0;
                
                phi1[0] = -1;


                // *********Initialize joystick*************//
                SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
                useJoystick = SDL_NumJoysticks() > 0;
                if (useJoystick)
                {
                    SDL_JoystickClose(m_joystick);
                    m_joystick = SDL_JoystickOpen(0);
                }
                // ****************************************//

                while (key != 27)
                {


                    //********************Joystick Events****************//
                   if (useJoystick)
                    {
                        SDL_Event event;
                        SDL_PollEvent(&event);


                        PLUSjoypadRoll = SDL_JoystickGetButton(m_joystick, 1);     //Derecha B
                        MINUSjoypadRoll = SDL_JoystickGetButton(m_joystick, 0);    //Izquierda A
                        PLUSjoypadPitch = SDL_JoystickGetButton(m_joystick, 5);    //Delante Z
                        MINUSjoypadPitch = SDL_JoystickGetButton(m_joystick, 2);   //Atras  C
                        joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);  //UP & DOWN (bajar o subir el parrot)
                        joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);            //LEFT & RIGHT (yaw) 
                        joypadTakeOff = SDL_JoystickGetButton(m_joystick, 9);      //Start Takeoff
                        joypadLand = SDL_JoystickGetButton(m_joystick, 7);         //Gatillo derecho Aterrizaje
                        joypadHover = SDL_JoystickGetButton(m_joystick, 6);        //Gatillo izquierdo Estabiliza
                        joypadAuto = SDL_JoystickGetButton(m_joystick, 3);         //X Pasa a automatico 
                        joypadManual = SDL_JoystickGetButton(m_joystick, 4);       //Y Pasa a Manual
                    }

                    //Puedes ver el nivel de bateria con la tecla b
                    if (key=='b')
                    {
                    printf("===================== Parrot status =====================\n\n");
                    fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
                    fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
                    fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
                    }
                    
                    //Status del parrot en manual
                    if (!automatico)
                    {
                    // Clear the console
                    printf("\033[2J\033[1;1H");

                    // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
                    printf("===================== Parrot status =====================\n\n");
                    fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
                    fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
                    fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
                    fprintf(stdout, "Hover   : %d \n", hover);
                    fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
                    fprintf(stdout, "  Roll    : %d \n", joyRoll);
                    fprintf(stdout, "  Pitch   : %d \n", joyPitch);
                    fprintf(stdout, "  Yaw     : %.2lf \n", yaw);
                    fprintf(stdout, "  V.S.    : %.2lf \n", verticalSpeed);
                    fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
                    fprintf(stdout, "  Land    : %d \n", joypadLand);
                    fprintf(stdout, "state: %d \n",state);
                    // *******************************************//

                    }

                    //image is captured
                    heli->renewImage(image);
                    // Copy to OpenCV Mat
                    rawToMat(currentImage, image);

                    if (currentImage.data) 
                    {
                        /* Show image */
                        imshow("Video", currentImage);
                        cvMoveWindow("Video", XVIDEO, YIMAGE);
                        storedImage.release();
                        //points.clear(); //remove all stored points on the vector
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
                        cvMoveWindow("oildrop", XIMAGE, YIMAGE);

                        //*******************Debugging************//
                        
                            
                            for (int i=0; i < NumberRegions; i++)
                            {
                                for (int j=0; j< 4; j++)
                                {
                                    //cout << phi1[i] << ' ' << phi2[i] << endl;
                                    if (pow(phi1[i]-refPhis[j][0], 2) + pow(phi2[i]-refPhis[j][1], 2) <= refPhis[j][2] + refPhis[j][3])
                                    {
                                        //go(j);
                                        switch(j)
                                        {
                                            case 0:
                                                if (move1 == 0)
                                                    move1=DERECHA; //R 10
                                                break;
                                            case 1:
                                                if (move1 == 0)
                                                    move1=IZQUIERDA; //matraz 20
                                                break;
                                            case 2:
                                                if (move2 == 0)
                                                    move2=ADELANTE; // Ml 03
                                                break;
                                            case 3:
                                                if (move2 == 0)
                                                    move2=ATRAS; //J 04
                                                break;
                                        }
                                        cout << move1 << ' '  << move2 << endl;
                                    }
                                }

                            }
                            
                        //***************************************/
                        
                    }

                    else
                    {
                        cout << "No image data.. " << endl;
                    }

                    //********************Puedes despegar con el la tecla 'l'**************//
                    if (key=='l')
                    {
                        heli->takeoff();
                        state=0;
                        begin = true;

                    }

                    //***Si pasas a automatico puedes recordar el filtro y el estado en el que vas
                    if (joypadAuto)
                    {
                    // Clear the console
                    printf("\033[2J\033[1;1H");
                    cout<<"MODO AUTOMATICO"<<endl;
                        automatico=true;
                cout<<"State: "<<state<<endl;;
                cout << "Read file:\n";
                cout << (int)minVec[0] << ' ' << (int)minVec[1] << ' ' << (int)minVec[2] << '\n';
                cout << (int)maxVec[0] << ' ' << (int)maxVec[1] << ' ' << (int)maxVec[2] << '\n';
                
                    }

                    if (joypadManual)
                        automatico=false;
                    
                    //************************************

                    if (automatico)
                    switch(state)
                    {
                        case -1:
                            move1 = move2 = 0;
                            break;

                        case 0:
                            if (begin)
                            {
                                endwait=time(NULL);
                                endwait = clock()+15*CLOCKS_PER_SEC;
                                begin = false;
                            }

                            heli->setAngles(0, -300, 0, -5000, 0);

                            if (clock()>=endwait)
                            {
                                state=1;
                                begin = true;
                            }
                            move1 = move2 = 0;
                            break;

                        case 1:
                            heli->setAngles(0, 0, 0, 0, 1);

                            for (int i=0; i < NumberRegions; i++)
                            {
                                for (int j=0; j< 4; j++)
                                {
                                    //cout << phi1[i] << ' ' << phi2[i] << endl;
                                    if (pow(phi1[i]-refPhis[j][0], 2) + pow(phi2[i]-refPhis[j][1], 2) <= refPhis[j][2] + refPhis[j][3])
                                    {
                                        //go(j);
                                        switch(j)
                                        {
                                            case 0:
                                                if (move1 == 0)
                                                    move1=DERECHA; //"V" 01
                                                break;
                                            case 1:
                                                if (move1 == 0)
                                                    move1=IZQUIERDA; //"[" 20
                                                break;
                                            case 2:
                                                if (move2 == 0)
                                                    move2=ADELANTE; // "I" 03
                                                break;
                                            case 3:
                                                if (move2 == 0)
                                                    move2=ATRAS; //"R" 04
                                                break;
                                        }
                                        cout << move1 << ' '  << move2 << endl;
                                    }
                                }

                            }

                            if(move1 != 0 && move2 != 0)
                            {
                              state=2;
                                cout<<"move1: "<<move1<<endl;
                                cout<<"move2: "<<move2<<endl;
                            }
                            break;

                        case 2:
                        if (begin)
                        {
                            endwait=time(NULL);
                            endwait = clock()+3*CLOCKS_PER_SEC;
                            begin = false;
                        }

                        if(move1 == DERECHA)
                            heli->setAngles(0, 5000, 0, 0, 0);
                        else
                            heli->setAngles(0, -5000, 0, 0, 0);


                        if (clock()>=endwait)
                        {
                            state=3;
                            begin = true;
                        }
                        break;

                        case 3:
                        if (begin)
                        {
                            endwait=time(NULL);
                            endwait = clock()+2*CLOCKS_PER_SEC;
                            begin = false;
                        }

                        heli->setAngles(0, 0, 0, 0, 1);

                        if (clock()>=endwait)
                        {
                            state=4;
                            begin = true;
                        }
                        break;

                        case 4:
                        if (begin)
                        {
                            endwait=time(NULL);
                            endwait = clock()+3*CLOCKS_PER_SEC;
                            begin = false;
                        }

                        if(move2 == ADELANTE)
                            heli->setAngles(-5000, 0, 0, 0, 0);
                        else
                            heli->setAngles(5000, 0, 0, 0, 0);

                        if (clock()>=endwait)
                        {
                            state=5;
                            begin = true;
                        }
                        break;

                        case 5:
                        if (begin)
                        {
                            endwait=time(NULL);
                            endwait = clock()+0.5*CLOCKS_PER_SEC;
                            begin = false;
                        }

                        heli->setAngles(0, 0, 0, 1000, 1);

                        if (clock()>=endwait)
                        {
                            heli->land();
                            state=-1;
                         }
                        break;

                        default:
                        heli->setAngles(0, 0, 0, 0, 1);
                        break;
                             
                    }

                        //**************Controlar el parrot manualmente**********//
                    else
                    {
                        //Si despegas manualmente, habilita para que el modo automatico actue desde el estado de busqueda de regiones
                        if (joypadTakeOff) 
                        {
                        heli->takeoff();
                        state=1;
                        begin=true;
                        }
                        if (joypadLand) {
                         heli->land();
                        }

                        hover = joypadHover ? 1 : 0;
                        //Nota: el 5000 es para que no sea tan brusco el movimiento
                        joyRoll = PLUSjoypadRoll ? 5000 : (MINUSjoypadRoll ? -5000 : 0);
                        joyPitch = PLUSjoypadPitch ? -5000 : (MINUSjoypadPitch ? 5000 : 0);
                        //Nota: Siempre |joypadyaw| y |joypadverticalspeed|= 32767 
                        if (joypadYaw>0)
                        yaw = 5000.0;
                        else if (joypadYaw<0)
                        yaw=-5000.0;
                        else
                        yaw=0;

                        if (joypadVerticalSpeed>0)
                        verticalSpeed = 5000.0;
                        else if (joypadVerticalSpeed<0)
                        verticalSpeed=-5000.0;
                        else
                        verticalSpeed=0;

                        
                        heli->setAngles(
                                joyPitch, 
                                joyRoll, 
                                yaw, 
                                verticalSpeed, 
                                hover
                                );
                         
                    }
                                                
                    /****************************************/
                    

                    key = waitKey(5);
                }

                heli->land();  //Aterrizar al dar escape
                SDL_JoystickClose(m_joystick);
                delete heli;
                delete image;
                //destroyAllWindows();
                break;



/*****************************************************/
/*        CALIBRACION DEL FILTRO                     */
/*****************************************************/

            case '2':
                if (!PARROT) {  // no parrot configured, open PC webcam
                    camera.open(0);
                } else {        // parrot configured
                    heli = new CHeli();
                    image = new CRawImage(320,240);
                    currentImage = Mat(240, 320, CV_8UC3);
                }

                namedWindow("Video");
                

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
                if (key == 27) {
                    destroyAllWindows();
                }
                
                break;


/*****************************************************/
/*        CALIBRACION DE PHI                         */
/*****************************************************/

            case '3':
                if (!PARROT) {  // no parrot configured, open PC webcam
                    camera.open(0);
                } else {        // parrot configured
                    heli = new CHeli();
                    image = new CRawImage(320,240);
                    currentImage = Mat(240, 320, CV_8UC3);
                }

                namedWindow("Video");

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

                savedPhis = 0;
                lastSavedPhis = 0;
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
                break;



/*****************************************************/
/*        GRAFICA DE PHIS                            */
/*****************************************************/

            case '4':
                
                double phi1, phi2;
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

                address = "../src/main/data/philist.txt";
                inFile.open(address.c_str());

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

                key = 0;
                while (key != 27) {
                    key = waitKey(5);
                }

                break;
            


            default:
                break;
        }
        
		usleep(10000);
        MainFunction = 'q'; //just quit
	}

}
/*************** END OF MAIN *********************/


/*************** PROCEDURES **********************/
// Convert CRawImage to Mat *for parrot use*
void rawToMat( Mat &destImage, CRawImage* sourceImage) {   
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
    NumberRegions=colorIndex;

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

            /*
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
                    cout << "PHI 1" << "\t" << phi1[i] << endl;
                    cout << "PHI 2" << "\t" << phi2[i] << endl;
                    cout << endl;
            */
        int x0, y0, x1, y1;
        int length = (50 + m00[i] / 500);

        circle(colorDst, Point(xtest[i],ytest[i]), 3, Scalar(128,128,128), -1, 8);

        x0 = xtest[i] - length * cos(angle[i]);
        y0 = ytest[i] - length * sin(angle[i]);

        x1 = xtest[i] + length * cos(angle[i]);
        y1 = ytest[i] + length * sin(angle[i]);

        line(colorDst,
            Point(x0, y0),
            Point(x1, y1),
            Scalar(255,255,255), 1, 8);
        length = (30 + m00[i] / 800);
        x0 = xtest[i] - length * cos(angle[i] + 1.57);
        y0 = ytest[i] - length * sin(angle[i] + 1.57);

        x1 = xtest[i] + length * cos(angle[i] + 1.57);
        y1 = ytest[i] + length * sin(angle[i] + 1.57);

        line(colorDst,
            Point(x0, y0),
            Point(x1, y1),
            Scalar(255,255,255), 1, 8);
        }

} //End OilDrop


void drawRotatedSquare(Mat &origin, Mat &mapaEnsanchado, int degree,Point center, Point verticesRotated[4])
{
        float angle=(degree*M_PI)/180.0;
        int r=15; //radio=30cm/2
        int rRobot=30; //radio del robot


            Point vertices[4];
            vertices[0]=Point(center.x-r,center.y-r);
            vertices[1]=Point(center.x-r,center.y+r);
            vertices[2]=Point(center.x+r,center.y+r);
            vertices[3]=Point(center.x+r,center.y-r);
            
            
            for (int i = 0; i < 4; i++)
            { 
            verticesRotated[i]=Point(
            center.x+ (vertices[i].x-center.x)*cos(angle)-(center.y-vertices[i].y)*sin(angle),
            center.y-((vertices[i].x-center.x)*sin(angle)+(center.y-vertices[i].y)*cos(angle))
            );
            
           
            }
            //fillConvexPoly(Mat& img, const Point* pts, int npts, const Scalar& color, int lineType=8, int shift=0)
            fillConvexPoly(origin, &verticesRotated[0], 4, Scalar (255, 0, 0));
            fillConvexPoly(mapaEnsanchado, &verticesRotated[0], 4, Scalar (255, 0, 0));



            Point derivedVertices[2];
            //Primera iteracion
            double ax,ay,nMag,nx,ny;
      
            ax=verticesRotated[0].x-verticesRotated[3].x;
            ay=verticesRotated[0].y-verticesRotated[3].y;
            nMag=sqrt(pow(ax,2)+pow(ay,2));
            nx=ay/nMag; //n=(ay,-ax)
            ny=-1*(ax/nMag);
            derivedVertices[1]=Point(
                verticesRotated[3].x-rRobot*nx,
                verticesRotated[3].y-rRobot*ny
                );
            derivedVertices[0]=Point(
                verticesRotated[0].x-rRobot*nx,
                verticesRotated[0].y-rRobot*ny
                );

            line(mapaEnsanchado, derivedVertices[1], derivedVertices[0], Scalar(255,0,0));
            //Iteraciones posteriores
            for (int i=0; i<3;i++)
            {
                ax=verticesRotated[i+1].x-verticesRotated[i].x;
                ay=verticesRotated[i+1].y-verticesRotated[i].y;
                nMag=sqrt(pow(ax,2)+pow(ay,2));
                nx=ay/nMag; //n=(ay,-ax)
                ny=-1*(ax/nMag);
                derivedVertices[1]=Point(
                    verticesRotated[i].x-rRobot*nx,
                    verticesRotated[i].y-rRobot*ny
                    );
                derivedVertices[0]=Point(
                    verticesRotated[i+1].x-rRobot*nx,
                    verticesRotated[i+1].y-rRobot*ny
                    );
                line(mapaEnsanchado, derivedVertices[1], derivedVertices[0], Scalar(255,0,0));
                //Arco
                ellipse(mapaEnsanchado,
                    verticesRotated[i],  //Pivote
                    Size(rRobot,rRobot),
                    (i*-90)-90-degree, //arco rotado
                    0,  
                    -90, 
                    Scalar(255,0,0));
            }

                //Arco Final
                ellipse(mapaEnsanchado,
                    verticesRotated[3],  //Pivote
                    Size(rRobot,rRobot),
                    -degree, //arco rotado
                    0,  
                    -90, 
                    Scalar(255,0,0));

               Rellenar(mapaEnsanchado, verticesRotated[0]);

}


void Rellenar(Mat &colorDst, Point seed) {
    Vec3b k = Vec3b(255, 0, 0);
            queue<Point> Fifo;
            Fifo.push(seed);

           colorDst.at<Vec3b>(seed) = k;   // color the seed pixel
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

                    if (coord.y >= 0 && coord.y < colorDst.rows && coord.x >= 0 && coord.x < colorDst.cols) { //is in range
                        Vec3b temp = colorDst.at<Vec3b>(coord);
                        if (temp != k) {                            //AND is not colored with k on the destination yet
                         //   aux = dst.at<uchar>(coord);
                            if (colorDst.at<Vec3b>(coord) != k) {        //AND is not 0 in the binary image
                                colorDst.at<Vec3b>(coord) = k;      //color the destination
                                Fifo.push(Point(coord));

                            }
                        }
                    }
                }

                Fifo.pop();
            }

        } //End OilDrop

void generaBinariaDeArchivo (const Mat &sourceImage, Mat &destinationImage) {
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
} // END generaimagenFiltrada

void generateHSV (const Mat &origin,  Mat &destination) {
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
} // END generateHSV

void graphPhis (double PhisArray[][2], int savedPhis) {
    Mat canvas(GROWS, GCOLS, CV_8UC1, Scalar(0,0,0));

    // Phi1 are X, Phi2 are Y
    Point P;
    for (int i = 0; i < savedPhis; i++) {
        P = Point( (int)(PhisArray[i][0]/1.5*GCOLS), (int)(GROWS - PhisArray[i][1] * GROWS) );
        //cout << P.x << ' ' << GROWS - P.y << '\n'; //debug
        circle(canvas, P, 3, Scalar( 255, 255, 255), CV_FILLED);
    }
    imshow("Phi Graph", canvas);
    cvMoveWindow("Phi Graph", XVIDEO, YIMAGE + 500);
} // END graphPhis

void graphPhis (double phi1, double phi2) {
    
    // Phi1 are X, Phi2 are Y
    Point P;
    P = Point( (int)(phi1/1.5 * GCOLS), (int)(GROWS - phi2 * GROWS) );
    //cout << P.x << ' ' << GROWS - P.y << '\n'; //debug
    circle(canvas, P, 3, *it, CV_FILLED);
    if (it == vecs.end()) it = vecs.begin();
    
    imshow("Phi Graph", canvas);
    cvMoveWindow("Phi Graph", XVIDEO, YIMAGE + 500);
} // END graphPhis







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
                        rectangle(tmpImage, orig, dest,Scalar(0, 255, 0));
                         imshow("Image", tmpImage);
                    }
                }
            break;
        case CV_EVENT_LBUTTONUP:
            
            clicked=false;
            break;
    }
}// END mouseCoordinatesExampleCallback

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
    tolerance[2] = 5; //V
    
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
} // END generaimagenFiltradaBinaria
/*************************************************/