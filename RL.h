// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <conio.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define FALSE 0
#define TRUE 1

//Physical System Constantsw
#define PIE 3.141592654

//Sample Dialog Constants
#define MAXNAMELEN 35
#define MAXSSLEN 12
#define MAXIDLEN 4
#define MAXSECLEN 3
#define MAXCUSTOMLEN 20

#define MAX_LOADSTRING 100

//Data Parameter Constants
#define MAXDATAFILENAMELEN 20
#define MAXPOINTDIG 5

//Other Constants
#define TIMER_ID 555

//Special Dialog Controls
const int IDC_BANGBANGSLIDE = 200;
const int IDC_BANGBANGVALUE = 201;

#define DEFMASTERFILENAME "master.fle"
#define DEFDATAFILENAME "output.dat"
#define DEFWEIGHTFILENAME "wegiths.wgt"
#define DEFNUM_OUTPOINTS "100"
#define DEFNUM_SKIPPOINTS "20"
#define DEF_COLLECTSYNC FALSE
#define MAX_STEPS 1000
#define MAX_RUNS 10
#define MAX_TRIALS 30

#define DEF_FREQEUENCY "2.33"
#define DEFINFREQ 111
#define DEFSINAMP 10

#define TITLE_SIZE 80
#define DEF_WINDOW_TITLE "Inverted Pendulum Controller:"
#define WINCLASSNAME "TIPWindow"

// Graphics Parameters page 97
//Symbolic constants
#define NUMCLRS 16 // Number of colors in colorArray
#define NUMDATAPOINTS 9 // Number of points in data array
#define SPACEATBOTTOM 60 // Reserved pixels below chart
#define SPACEATLEFT 80 //Reserved pixels to left of chart
#define SPACEATTOP 40  //Reserved pixels at top of chart
#define SPACEATRIGHT 40 //Reserved pixels at right of chart
#define YSCALEMAX 100.0 // Maximum Y scale value
#define YSCALEINCREMENT 10.0 // Increment for Y scale markers
//DISPLAY VARIABLES Constant expressions
#define SPACEVERTICAL (SAPACEATTOP + SPACEATBOTTOM)
#define SPACEHORIZONTAL (SPACEATLEFT + SPACEATRIGHT)
#define HALFSPACEBOTTOM (SPACEATBOTTOM / 2)
#define HALFSPACEATRIGHT (SPACEATRIGHT /2 )
#define YSCALEMARKERS (YSCALEMAX/YSCALEINCREMENT)
#define YAXISX (SPACEATLEFT/2)
#define YAXISY (SPACEATTOP/2)
#define YMARKERX1 (SPACEATLEFT/4)
#define YMARKERX2 (YMARKERX1+(SPACEATLEFT/2))
//GRAPH PARAMETERS
#define YAXISDEFMAXVALUE 100
#define DEFVOLTSTOPDEGREES 25
#define DEFMAGPIXELSVOLT 3
#define DEFMAGPIXELSDEGREE 10

// CONTROL ALGORITHM TYPES
#define PID 0
#define NEURAL_ACE_ASE 1
#define FUZZY 2 //Not implemented

// DEFAULTS FOR SYSTEM MODEL
#define NS 2 // Number of States in State Space Model (dimensions)

double pi = PIE;

/* Alexandeer Declarations (left blank for now) */

// NEURAL CONTROL ACE and ASE
// NEURAL DEFAULTS
#define DEFALPHA 1000
#define MAX_NUM_BOXES 20
#define DEF_NUM_OF_THETA_BOXES 3
#define DEF_NUM_OF_DTHETA_BOXES 3
#define NUM_OF_BOXES (NUM_OF_THETA_BOXES * NUM_OF_DTHETA_BOXES)
#define MAX_NUM_NODES 100
#define NUM_OF_ISNODES 25
#define DEF_BANG_BANG_GAIN 45 //45
#define DEF_OVERLAP_VALUE 0.30 //0.30
#define DEF_THETA_EXTREME 12.0 // deg
#define DEF_DTHETA_EXTREME 50.0 // deg/sec
#define THETA_MAX 90
#define DTHETA_MAX 50
// Global ACE and ASE variables
int NumThetaBoxes = DEF_NUM_OF_THETA_BOXES;
int NumDThetaBoxes = DEF_NUM_OF_DTHETA_BOXES;
float ThetaExtreme = DEF_THETA_EXTREME;
float DThetaExtreme = DEF_DTHETA_EXTREME;
int NumOfNotdes = NUM_OF_ISNODES;
int trx[100];
int TrialNum = 0, RunNum = 0;
int Trials[MAX_TRIALS], Runs[MAX_RUNS], steps, t, Alpha;
int LifeTime[MAX_RUNS][MAX_TRIALS];
double wt[MAX_NUM_NODES], vt[MAX_NUM_NODES];
double elg[MAX_NUM_NODES], elg0[MAX_NUM_NODES], xbar[MAX_NUM_NODES];
double tempelg[MAX_NUM_NODES], vtseq[MAX_NUM_NODES], wtseq[MAX_NUM_NODES];
double ncy[MAX_NUM_NODES];
double ncx[MAX_NUM_NODES];
double ISNode[MAX_NUM_NODES];
double predlast, pred, tstep, Delta, fs = 45.0;
int reinf;

int IC = 0;

//page 99
double internal_reinf;
float BangBangGain = (float) DEF_BANG_BANG_GAIN;
float Overlap = (float) DEF_OVERLAP_VALUE;
float systime[MAX_STEPS]; //huge
float ang[MAX_STEPS]; //huge
static float xxc[MAX_STEPS]; //huge
static float yyc[MAX_STEPS]; //huge
float force[MAX_STEPS]; //huge
int failure, NumRunFails[MAX_RUNS]={0}, method = 0;
int OtherWeights = 0;
float action;
double Beta, Gamma, Lamda;
float SigmaTheta[MAX_NUM_NODES], SigmaDTheta[MAX_NUM_NODES]; //RBF Overlap
double yZ[NS], x0[NS], states[NS]={3.14,0.0}, y[NS];
double s1[NS], s2[NS]; //used for Calculating Integration of
double s3[NS], s4[NS]; //States
double ys1[NS], ys2[NS], ys3[NS], ys4[NS];
int prevt;
float MaxAngMag = 1.0, MaxAngVelMag = 1.0;
//constants
double Rad2Ang = 180 / PIE;
double Ang2Rad = PIE / 180;
float FailAng;

double x[NS];
int TempISNode[25];
float xcmac[NS];
bool bFailedToLearn = true;
int iFailCount = 0;

//functions
bool InitializeNeuralACEASE();
int U[MAX_NUM_NODES][MAX_STEPS]; // Membership for Clustering Algorithm  huge
static float J[MAX_NUM_NODES];
void ace();
void ase();
int cmac_id;
int cart_state[NS];
double angle, dtheta;
void decoder();
void PoleModelSolve();
void PoleStateSpaceModel(double dtdx[], double t, double pstate[], float u);
enum TrapezoidType { reg, left, right } Typ;
float Trap(TrapezoidType Typ, float a, float b, float c, float d, float in); // a,b,c,d corner points of trapezoid from left to right
//MISCELANEOUS GLOBAL VARIABLES
char gs[200] = "TEST";
char ex[5] = "NoNe";
char fn[200] = "TEMP";
char MasterFileName[200];
//int NIDAQENABLE = 0;
int SIMULATE = 1;
//long x = 1;
//page 100
int delay = 0;
int xx;
int xx2;
int cx, cy;
char SaveFileName[40] = DEFDATAFILENAME;
int MagPV = DEFMAGPIXELSVOLT, MagPD = DEFMAGPIXELSDEGREE;
int yGMax, yGMin;

int PlotOldDat = -1;
int tMax;
int ControlType = NEURAL_ACE_ASE; // Default PID

int data_rec;
int graph_output, graph_error;
int NumOfNodes;

struct TDataParamStruct {
    char DataFileName[MAXDATAFILENAMELEN];
    char NumOutPoints[MAXPOINTDIG];
    char NumSkipPoints[MAXPOINTDIG];
    bool CollectSync;
    int LifeTimes;
    int States;
};

struct TFrequency {
    char Freq[5];
};

struct PIDStruct {
    int SingleLoop;
    int DualLoop;
} PIDOptions;

struct RUNSTruct {
    int KeepSimGoing;
    int SaveWeightsToMemory;
    int SaveWeightsToFile;
    int DontSaveWeights;
} RUNOptions;

struct NeuralACEASEStruct {
    int ZeroizeWeights;
    int UseSimulationWeights;
    int WeightFromFile;
    char WeightFileName[40];
    int Uniform;
//page 101
    int Nonuniform;
    int CMAC;
    char cNumThetaBoxes[10];
    char cNumDThetaBoxes[10];
    char cThetaExtreme[10];
    char cDThetaExtreme[10];
    char cAlpha[40];
    int OutSigmoid;
    int OutBangBang;
    int DisturbanceYes;
    int DisturbanceNo;
    int RBF;
} NeuralACEASEOptions;

struct TCalibration {
    int DirectlyDriven;
    int CenterPosition;
    int EndPosition;
    char PoleAngle[10];
};

TCalibration Calibration;

float calibrate;

struct TSinWavRefParam {
    char Freq[10];
    char Amp[10];
};

//#include "ip.h"
//#include "unh_cmac.h" // CMAC header files
//#include "unh_cmac.c" // CMAC functions
/*
extern "C" {
#include "alexfunc.c" // Chris Alexander's Functions
}
#include "ipdlg3_1.cpp" // Defintions for all the Dialog Functions
#include "acease48.cpp" // Neural ACE and ASE control algorithm ???????
*/

int xMax, yMax; // Maximum client area coordinates
int yBase;
//page 102
int yOffset, xOffset;

const PIDStruct* GetPIDOptions() { return &PIDOptions; }
const TCalibration* GetCalibOptions() { return &Calibration; }
//const TGraphics* GetGraphics() { return &Graphics; }
//const RUNStruct* GetRunOptions() { return &RUNOptions; }
float NeuralACEASE(int IC, int jj, double states[]);
void ClusterInputSpace(/* TDC& */);
//void TrainCMAC();
int RunWeightSave();
void CmEmpInput();
void CmFrequency();
//void SetupGraph(TDC&);
//void Graph(float Angle[], float RefInput[], float CompOuput[], int TimeStep);
char DataFileName[50];
//char MasterFileName[50];
char FilePath[200];
void SetMaxCoordinates();
//void DrawGrid(TDC& dc);
//bool EvEraseBkgnd(HDC);
//oid Line(HDEC hDC, int x1, inty1, int x2, int y2w, COLORREF color);
//void EvChar(UINT, UINT, UINT);
int NumOfDataPoints;
TDataParamStruct DataParamStruct;
TFrequency Frequency;
PIDStruct PIDOptoins;
//RUNStruct RUNOptions;
//TGraphics Graphics;
//TWindowOptions WindowOptions;
TSinWavRefParam SinWavRefParam;
//TMenu* windowMenu;
//void Paint(TDC&, BOOL, TRect&);
void SetupWindow();
void CmFileExit();
void CmSimulate();
void CmNIDAQEnable();
void CmSetupData();
bool InitOptions();
//    TOpenSaveDialog::TData* FileData;
void CmFileOpen();
void CmFileSave();
void CmPID();
void CmNeuralACEASE();
//page 103
bool CmCalibration();
void CmDisplay();
void CmOldDataGraph();
bool CmBeginControl(void);
void CmRefSineWave();
void CmRefSeriesoOfSteps();
//void EvSize(UINT sizeType, TSize& size);
//void EvKeyDown(UINT key, UINT repeatCoutn, UINT flags);
//void EvMouseMove(UINT modKeys, TPoint& point);
//void EvMButtonDown(UINT modKeys, TPoint& point);
