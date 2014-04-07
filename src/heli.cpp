#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
#include <time.h>
/*
 * A simple 'getting started' interface to the ARDrone, v0.2 
 * author: Tom Krajnik
 * The code is straightforward,
 * check out the CHeli class and main() to see 
 */
#include <stdlib.h>
#include "CHeli.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

#define DERECHA 1
#define IZQUIERDA 2
#define ADELANTE 3
#define ATRAS 4

using namespace std;
using namespace cv;

bool stop = false;
CRawImage *image;
CHeli *heli;
float pitch, roll, yaw, height;
int hover,joyRoll, joyPitch;


// **Start timer **
clock_t endwait;

int move1 = 0, move2 = 0;

int state=-1;
bool begin;

// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadVerticalSpeed, joypadYaw;
bool PLUSjoypadRoll, MINUSjoypadRoll, PLUSjoypadPitch, MINUSjoypadPitch, navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover, joyPadAccion;


// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage)
{	
	uchar *pointerImage = destImage.ptr(0);
	
	for (int i = 0; i < 240*320; i++)
	{
		pointerImage[3*i] = sourceImage->data[3*i+2];
		pointerImage[3*i+1] = sourceImage->data[3*i+1];
		pointerImage[3*i+2] = sourceImage->data[3*i];
	}
}

int main(int argc,char* argv[])
{
	//establishing connection with the quadcopter
	heli = new CHeli();
	
	//this class holds the image from the drone	
	image = new CRawImage(320,240);
	
	// Initial values for control	
    pitch = roll = yaw = height = 0.0;
    joyPitch = joyRoll = joypadYaw = joypadVerticalSpeed = 0.0;

	// Destination OpenCV Mat	
	Mat currentImage = Mat(240, 320, CV_8UC3);
	// Show it	
	imshow("ParrotCam", currentImage);

    // Initialize joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick)
    {
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }

    while (stop == false)
    {
        // Clear the console
        printf("\033[2J\033[1;1H");


       if (useJoystick)
        {
            SDL_Event event;
            SDL_PollEvent(&event);

            PLUSjoypadRoll = SDL_JoystickGetButton(m_joystick, 3);
            MINUSjoypadRoll = SDL_JoystickGetButton(m_joystick, 0);
            PLUSjoypadPitch = SDL_JoystickGetButton(m_joystick, 4);
            MINUSjoypadPitch = SDL_JoystickGetButton(m_joystick, 1);
            joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
            joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
            joypadTakeOff = SDL_JoystickGetButton(m_joystick, 6);
            joypadLand = SDL_JoystickGetButton(m_joystick, 2);
            joypadHover = SDL_JoystickGetButton(m_joystick, 7);
            joyPadAccion = SDL_JoystickGetButton(m_joystick, 9);
        }

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Parrot Basic Example =====================\n\n");
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        fprintf(stdout, "Hover   : %d \n", hover);
        fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        fprintf(stdout, "  PLUSRoll    : %d \n", PLUSjoypadRoll ? 1 : 0);
        fprintf(stdout, "  MINUSRoll    : %d \n", MINUSjoypadRoll ? 1 : 0);
        fprintf(stdout, "  PLUSPitch   : %d \n", PLUSjoypadPitch  ? 1 : 0);
        fprintf(stdout, "  MINUSPitch   : %d \n", MINUSjoypadPitch ? 1 : 0);
        fprintf(stdout, "  Yaw     : %d \n", joypadYaw);
        fprintf(stdout, "  V.S.    : %d \n", joypadVerticalSpeed);
        fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
        fprintf(stdout, "  Land    : %d \n", joypadLand);
        fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);
        fprintf(stdout, "state: %d \n",state);
		//image is captured
		heli->renewImage(image);

		// Copy to OpenCV Mat
		rawToMat(currentImage, image);
		imshow("ParrotCam", currentImage);

        char key = waitKey(5);
		switch (key) {
			case 'a': yaw = -20000.0; break;
			case 'd': yaw = 20000.0; break;
			case 'w': height = -20000.0; break;
			case 's': height = 20000.0; break;
			case 'q': heli->takeoff(); break;
			case 'e': heli->land(); break;
			case 'z': heli->switchCamera(0); break;
			case 'x': heli->switchCamera(1); break;
			case 'c': heli->switchCamera(2); break;
			case 'v': heli->switchCamera(3); break;
			case 'j': roll = -20000.0; break;
			case 'l': roll = 20000.0; break;
			case 'i': pitch = -20000.0; break;
			case 'k': pitch = 20000.0; break;
            case 'h': hover = (hover + 1) % 2; break;
            case 27: stop = true; break;
            default: pitch = roll = yaw = height = 0.0;
		}

        if (joypadTakeOff) {
            heli->takeoff();
        }
        if (joypadLand) {
            heli->land();
        }
        hover = joypadHover ? 1 : 0;
        joyRoll = PLUSjoypadRoll ? 20000 : (MINUSjoypadRoll ? -20000 : 0);
        joyPitch = PLUSjoypadPitch ? 200000 : (MINUSjoypadPitch ? -20000 : 0);

        //setting the drone angles
        if (joyRoll != 0 || joyPitch != 0 || joypadVerticalSpeed != 0 || joypadYaw != 0)
        {
            heli->setAngles(joyPitch, joyRoll, joypadYaw, (joypadVerticalSpeed*-1)/30, hover);
            navigatedWithJoystick = true;
        }
        // else
        // {
        //     heli->setAngles(pitch, roll, yaw, height, hover);
        //     navigatedWithJoystick = false;
        // }
        if (joyPadAccion)
        {
            heli->takeoff();
            state=0;
            begin = true;
            
        }

        //
        move1 = IZQUIERDA;
        move2 = ADELANTE;
        //

        switch(state)
        {
        case 0:
        if (begin)
        {
            endwait=time(NULL);
            endwait = clock()+15*CLOCKS_PER_SEC;
            begin = false;
        }

        heli->setAngles(0, 0, 0, -3000, 1);
           
        if (clock()>=endwait)
        {
            state=1;
            begin = true;
        }
        break;

        case 1:
        if (begin)
        {
            endwait=time(NULL);
            endwait = clock()+2*CLOCKS_PER_SEC;
            begin = false;
        }

        if(move1 == DERECHA)
            heli->setAngles(0, 5000, 0, 0, 0);
        else
            heli->setAngles(0, -5000, 0, 0, 0);


        if (clock()>=endwait)
        {
            state=2;
            begin = true;
        }
        break;

        case 2:
        if (begin)
        {
            endwait=time(NULL);
            endwait = clock()+2*CLOCKS_PER_SEC;
            begin = false;
        }

        heli->setAngles(0, 0, 0, 0, 1);

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
            endwait = clock()+3*CLOCKS_PER_SEC;
            begin = false;
        }

        if(move2 == ADELANTE)
            heli->setAngles(-5000, 0, 0, 0, 0);
        else
            heli->setAngles(5000, 0, 0, 0, 0);

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

        heli->setAngles(0, 0, 0, 1000, 1);

        if (clock()>=endwait)
        {
            heli->land();
            //endwait=clock()+5*CLOCKS_PER_SEC;
            state=-1;
         }
        break;

        default:
        heli->setAngles(pitch, roll, yaw, height, hover);
        break;
                 
        }
        
        usleep(10000);
	}
	
	heli->land();
    SDL_JoystickClose(m_joystick);
    delete heli;
	delete image;
	return 0;
}

