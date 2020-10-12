#include <stdio.h>
#include <string.h>
#include <math.h>


#include "XPLMPlugin.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMCamera.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"


#include <gl\gl.h>
#include <gl\glu.h>

#define IGNOREDPARAMETER 1

XPLMDataRef gLatitude = NULL;
XPLMDataRef gLongitude = NULL;
XPLMDataRef gElevation = NULL;
XPLMDataRef gPlaneX = NULL;
XPLMDataRef gPlaneY = NULL;
XPLMDataRef gPlaneZ = NULL;
XPLMDataRef gPlaneTheta = NULL;
XPLMDataRef gPlanePhi = NULL;
XPLMDataRef gPlanePsi = NULL;

XPLMMenuID gAcquireAircraftMenu;
int gAcquireAircraftSubMenuItem;

char* gpAircraft[4];
char gAircraftPath[4][256];

const double kFullPlaneDist = 5280.0 / 3.2 * 3.0;
static inline float sqr(float a) { return a * a; }
static inline float CalcDist3D(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return sqrt(sqr(x2 - x1) + sqr(y2 - y1) + sqr(z2 - z1));
}

void AcquireAircraftMenuHandlerCallback(void* inMenuRef, void* inItemRef);
float AcquireAircraftFlightLoopCB(float elapsedMe, float elapsedSim, int counter, void* refcon);
void AcquireAircraftPlanesAvailableCallback(void* inRefcon);
int AcquireAircraftDrawCallback(XPLMDrawingPhase inPhase,
	int inIsBefore,
	void* inRefcon);
void AcquireAircraft(void);

typedef struct LLA {
	double latitude, longitude, altitude, x, y, z;
}LLA;

LLA intruders[4];


//---------------------------------------------------------------------------

PLUGIN_API int XPluginStart(char* outName,
	char* outSig,
	char* outDesc)
{

	intruders[1] = { 47.498073, 10.669715, 3166.969391, 0, 0, 0 };
	intruders[2] = { 47.503 , 10.689, 3050, 0, 0, 0 };
	intruders[3] = { 47.553 , 10.663, 3050, 0, 0, 0 };


	strcpy(outName, "AcquireAircraft");
	strcpy(outSig, "xplanesdk.SandyBarbour.AcquireAircraft");
	strcpy(outDesc, "A plugin that controls aircraft.");

	gAcquireAircraftSubMenuItem = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "AcquireAircraft", 0, IGNOREDPARAMETER);
	gAcquireAircraftMenu = XPLMCreateMenu("AcquireAircraft", XPLMFindPluginsMenu(), gAcquireAircraftSubMenuItem, AcquireAircraftMenuHandlerCallback, 0);
	XPLMAppendMenuItem(gAcquireAircraftMenu, "Acquire Planes", "Acquire Planes", IGNOREDPARAMETER);
	XPLMAppendMenuItem(gAcquireAircraftMenu, "Release Planes", "Release Planes", IGNOREDPARAMETER);
	XPLMAppendMenuItem(gAcquireAircraftMenu, "Load Aircraft", "Load Aircraft", IGNOREDPARAMETER);

	gLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
	gLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
	gElevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
	gPlaneX = XPLMFindDataRef("sim/flightmodel/position/local_x");
	gPlaneY = XPLMFindDataRef("sim/flightmodel/position/local_y");
	gPlaneZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
	gPlaneTheta = XPLMFindDataRef("sim/flightmodel/position/theta");
	gPlanePhi = XPLMFindDataRef("sim/flightmodel/position/phi");
	gPlanePsi = XPLMFindDataRef("sim/flightmodel/position/psi");

	XPLMRegisterDrawCallback(AcquireAircraftDrawCallback, xplm_Phase_Airplanes, 0, NULL);

	return 1;
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginStop(void)
{
	XPLMUnregisterDrawCallback(AcquireAircraftDrawCallback, xplm_Phase_Objects, 0, NULL);
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginDisable(void)
{
}

//---------------------------------------------------------------------------

PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,
	long inMessage,
	void* inParam)
{
}

//---------------------------------------------------------------------------

void AcquireAircraftMenuHandlerCallback(
	void* inMenuRef,
	void* inItemRef)
{
	char FileName[256], AircraftPath[256];

	if (!strcmp((char*)inItemRef, "Acquire Planes"))
	{
		AcquireAircraft();
	}

	if (!strcmp((char*)inItemRef, "Release Planes"))
	{
		XPLMReleasePlanes();
	}

	if (!strcmp((char*)inItemRef, "Load Aircraft"))
	{
		XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, FileName, AircraftPath);
		XPLMSetAircraftModel(1, AircraftPath);
		XPLMSetAircraftModel(2, AircraftPath);
		XPLMSetAircraftModel(3, AircraftPath);
	}
}

//---------------------------------------------------------------------------

void AcquireAircraftPlanesAvailableCallback(void* inRefcon)
{
	OutputDebugString("AcquireAircraftPlanesAvailableCallback\n");
	AcquireAircraft();
}

//---------------------------------------------------------------------------

int AcquireAircraftDrawCallback(XPLMDrawingPhase inPhase,
								int inIsBefore,
								void* inRefcon)
{
	int planeCount;
	double x, y, z, x1, y1, z1;

	float distMeters, Latitude, Longitude, Elevation;
	float Heading, Pitch, Roll;

	int drawFullPlane;
	int Index;

	XPLMCountAircraft(&planeCount, 0, 0);

	if (planeCount <= 1)
		return 0;

	XPLMCameraPosition_t cameraPos;

	XPLMReadCameraPosition(&cameraPos);

	Latitude = XPLMGetDataf(gLatitude);
	Longitude = XPLMGetDataf(gLongitude);
	Elevation = XPLMGetDataf(gElevation);
	Pitch = XPLMGetDataf(gPlaneTheta);
	Roll = XPLMGetDataf(gPlanePhi);
	Heading = XPLMGetDataf(gPlanePsi);

	//char buf[500];
	//sprintf(buf, "User Heading: %f \0", Heading);

	//XPLMDebugString(buf);

	XPLMWorldToLocal(Latitude, Longitude, Elevation, &x, &y, &z);
	XPLMWorldToLocal(intruders[1].latitude, intruders[1].longitude, intruders[1].altitude, &intruders[1].x, &intruders[1].y, &intruders[1].z);
	XPLMWorldToLocal(intruders[2].latitude, intruders[2].longitude, intruders[2].altitude, &intruders[2].x, &intruders[2].y, &intruders[2].z);
	XPLMWorldToLocal(intruders[3].latitude, intruders[3].longitude, intruders[3].altitude, &intruders[3].x, &intruders[3].y, &intruders[3].z);

	//int ai = 1;
	for (Index = 1; Index < 3; Index++)
	{
		//x1 = x - (Index * 50.0);
		//y1 = y;
		//z1 = z + (Index * 50.0);

		distMeters = CalcDist3D(intruders[Index].x, intruders[Index].y, intruders[Index].z, cameraPos.x, cameraPos.y, cameraPos.z);
		if (cameraPos.zoom != 0.0)
			distMeters /= cameraPos.zoom;

		drawFullPlane = distMeters < kFullPlaneDist;

		float backAzimuth = 0;
		if (Heading >= 180) {
			backAzimuth = Heading - 180;
		}
		else {
			backAzimuth = Heading + 180;
		}


		XPLMPlaneDrawState_t DrawState;
		DrawState.structSize = sizeof(XPLMPlaneDrawState_t);
		DrawState.gearPosition = 1;
		DrawState.flapRatio = 1.0;
		DrawState.spoilerRatio = 0;
		DrawState.speedBrakeRatio = 0;
		DrawState.slatRatio = 0;
		DrawState.wingSweep = 0;
		DrawState.thrust = 0;
		DrawState.yolkPitch = 0;
		DrawState.yolkHeading = 0;
		DrawState.yolkRoll = 0;

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(intruders[Index].x, intruders[Index].y, intruders[Index].z);
		glRotatef(backAzimuth, 0.0, -1.0, 0.0);
		glRotatef(Pitch, 1.0, 0.0, 0.0);
		glRotatef(Roll, 0.0, 0.0, -1.0);
		XPLMDrawAircraft(Index, (float)intruders[Index].x, (float)intruders[Index].y, (float)intruders[Index].z, Pitch, Roll, backAzimuth, drawFullPlane ? 1 : 0, &DrawState);
		//ai++;
		//if (ai > (planeCount - 1))
		//	ai = 1;
		glPopMatrix();
	} 
	return 1;
}

//---------------------------------------------------------------------------

void AcquireAircraft(void)
{
	int PlaneCount;
	int Index;
	char FileName[256], AircraftPath[256];

	XPLMCountAircraft(&PlaneCount, 0, 0);
	if (PlaneCount > 1)
	{
		for (Index = 1; Index < PlaneCount; Index++)
		{
			XPLMGetNthAircraftModel(Index, FileName, AircraftPath);
			strcpy(gAircraftPath[Index - 1], AircraftPath);
			gpAircraft[Index - 1] = (char*)gAircraftPath[Index - 1];

		}
		if (XPLMAcquirePlanes((char**)&gpAircraft, AcquireAircraftPlanesAvailableCallback, NULL))
		{
			OutputDebugString("Aircraft Acquired successfully\n");
		}
		else
		{
			OutputDebugString("Aircraft not Acquired\n");
		}
	}
}

