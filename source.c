/* 
 *
 *
 *  C++ Source Code from Original Masters Thesis Named:
 *
 *  "Reinforcement Learning Experiments with State Classifiers
 *  for Controlling an Inverted Pendulum"
 *
 *  by John Hiett, Arizona State University 1997
 *  
 *  (hardware/software: IBM PC clone running MS Windows 3.10, compiled with
 *   Borland C++ Compiler 4.52)
 * 
 */


//-----------------------------------------------------------------------------
//  Page 96


// filename ip.h
#include <owl\owlpch.h>
#include <owl\applicat.h>
#include <owl\dialog.h>
#include <owl\framewin.h>
#include <owl\edit.h>
#include <owl\checkbox.h>
#include <owl\validate.h>
#include <owl\inputdia.h>
#include <owl\opensave.h>
#include <owl\radiobut.h>
#include <owl\menu.h>
#include <owl\scrollba.h>
#include <owl\gdiobjec.h>
#include <owl\dc.h>
#include <string.h>  // for strcpy and strcat #include <stdlib.h>  // for atoi
#include <stdlib.h>
#include <ctype.h>  // for isdigit and isalpha
#include <conio.h>
#include <float.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "wdaq_bc.h"
#include "ip.rh"

// Physical System Constants
#define MOTORVOLT_GAIN 10.58 // See Alexander Thesis
                             //#define PIE 3.141592654
#define   PIE 3.14159265358979323846

// Sample Dialog Constants
#define MAXNAMELEN 35
#define MAXSSLEN 12
#define MAXIDLEN 4
#define MAXDEPTLEN 7
#define MAXSECLEN 3
#define MAXCUSTOMLEN 20

//Data Parameter Constants
#define MAXDATAFILENAMELEN 20
#define MAXPOINTDIG 5

// Other Constants
#define TIMER_ID 555

// Special Dialog Controls
const WORD IDC_BANGBANGSLIDE = 200;


//-- Page 97 -------------------------------------------------------------------
const WORD IDC_BANGBANGVALUE = 201;
#define DEFMASTERFILENAME "master.fle"
#define DEFDATAFILENAME "output.dat"
#define DEFWEIGHTFILENAME "weights.wgt"
#define DEFNUM_OUTPOINTS "100"
#define DEFNUM_SKIPPOINTS "20"
#define DEF_COLLECTSYNC FALSE
#define MAX_STEPS 1000
#define MAX_RUNS 10
#define MAX_TRIALS 30

#define DEF_FREQUENCY "2.33"
#define DEFSINFREQ 111
#define DEFSINAMP 10

#define TITLE_SIZE 80
#define DEF_WINDOW_TITLE "Inverted Pendulum Controller: "
#define WINCLASSNAME "TIPWindow"

/* Graphics Parameters */

// SYMBOLIC CONSTANTS
#define NUMCLRS 16           // Number of colors in colorArray
#define NUMDATAPOINTS 9      // Number of points in data array
#define SPACEATBOTTOM 60     // Reserved pixels below chart
#define SPACEATLEFT 80       // Reserved pixels to left of chart
#define SPACEATTOP 40        // Reserved pixels at top of chart
#define SPACEATRIGHT 40      // Reserved pixels at right of chart
#define YSCALEMAX 100.0      // Maximum Y scale value
#define YSCALEINCREMENT 10.0 // Increment for Y scale markers
                             // DISPLAY VARIABLES Constant expressions
#define SPACEVERTICAL (SPACEATTOP + SPACEATBOTTOM)
#define SPACEHORIZONTAL (SPACEATLEFT + SPACEATRIGHT)
#define HALFSPACEATBOTTOM (SPACEATBOTTOM / 2)
#define HALFSPACEATRIGHT (SPACEATRIGHT / 2)
#define YSCALEMARKERS (YSCALEMAX / YSCALEINCREMENT)
#define YAXISX (SPACEATLEFT / 2)
#define YAXISY (SPACEATTOP / 2)
#define YMARKERX1 (SPACEATLEFT / 4)
#define YMARKERX2 (YMARKERX1 + (SPACEATLEFT / 2))
// GRAPH PARAMETERS
#define YAXISDEFMAXVALUE 100
#define DEFVOLTSTODEGREES 25;
#define DEFMAGPIXELSVOLT 3;
#define DEFMAGPIXELSDEGREE 10;

// CONTROL ALOGRITHM TYPES
#define PID 0;
#define NEURAL_ACE_ASE 1;

//-- Page 98 -------------------------------------------------------------------
#define FUZZY 2; // Not implemented

// DEFAULTS FOR SYSTEM MODEL
#define NS 2 // Number of States in State Space Model
double pi = PIE;

/* Alexander Declarations */
int err_num, jj, board, IC, graph_error, out, jk, jkk;
long color;
unsigned int count, t_count;
float comp_in;
double countt, fcount, tme;
char cr, c;
float rads;

// NEURAL CONTROL ACE and ASE
// NEURAL DEFAULTS
#define DEFALPHA 1000
#define MAX_NUM_BOXES 20
#define DEF_NUM_OF_THETA_BOXES 5
#define DEF_NUM_OF_DTHETA_BOXES 5
#define NUM_OF_BOXES (NUM_OF_THETA_BOXES * NUM_OF_DTHETA_BOXES)
#define MAX_NUM_NODES 100
#define NUM_OF_ISNODES 25
#define DEF_BANG_BANG_GAIN 45.0
#define DEF_OVERLAP_VALUE 0.60
#define DEF_THETA_EXTREME 12.0  // deg
#define DEF_DTHETA_EXTREME 50.0 // deg/sec
#define THETA_MAX 90
#define DTHETA_MAX 50
// Global ACE and ASE variables
int NumThetaBoxes = DEF_NUM_OF_THETA_BOXES;
int NumDThetaBoxes = DEF_NUM_OF_DTHETA_BOXES;
float ThetaExtreme = DEF_THETA_EXTREME;
float DThetaExtreme = DEF_DTHETA_EXTREME;
int NumOfNodes = NUM_OF_ISNODES;
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
double predlast, pred, tstep, Delta;
int reinf;


//-- Page 99 -------------------------------------------------------------------

double internal_reinf;
float BangBangGain = DEF_BANG_BANG_GAIN;
float Overlap = DEF_OVERLAP_VALUE;
float huge systime[MAX_STEPS]; // array is large
float huge ang[MAX_STEPS];
static float huge xxc[MAX_STEPS];
static float huge yyc[MAX_STEPS];
float huge force[MAX_STEPS];
int failure, NumRunFails[MAX_RUNS], method, OtherWeights = 0;
float action;
double Beta, Gamma, Lamda;
float SigmaTheta[MAX_NUM_NODES], SigmaDTheta[MAX_NUM_NODES]; // RBF Overlap
double y0[NS], x0[NS], states[NS], y[NS];
double s1[NS], s2[NS]; // Used for Calculating Integration of
double s3[NS], s4[NS]; // States
double ys1[NS], ys2[NS], ys3[NS], ys4[NS];
int prevt;
float MaxAngMag = 1.0, MaxAngVelMag = 1.0;
// float xbox[MAX_BOXES]; // Box Membership Value
// constants
double Rad2Ang = 180/PIE;
double Ang2Rad = PIE/180;
float FailAng;
// functions
void InitializeNeuralACEASE(int, int);
int huge U[MAX_NUM_NODES][MAX_STEPS]; // Membership for Clustering Algorithm
static float J[MAX_NUM_NODES];        // Cost--for Clustering Algorithm
void ace();
void ase();
int cmac_id;
int cart_state[NS];
double angle, dtheta;
void decoder();
void PoleModelSolve();
void PoleStateSpaceModel(double dtdx[], double t, double states2[], float u);
enum TrapezoidType { reg, left, right } Typ;
// a,b,c,d corner points of trapezoid from left to right
float Trap(TrapezoidType Typ, float a, float b, float c, float d, float in);
// MISCELANEOUS GLOBAL VARIABLES
char gs[200] = "TEST";
char ex[5] = "NoNe";
char fn[200] = "TEMP";
char MasterFileName[200];
int NIDAQENABLE = 0; // Controls All Commands To NIDAQ Card, 1-Enable
int SIMULATE = 1;
long x = 1;
TColor white(255, 255, 255), green(0, 255, 0), black(0, 0, 0);
TColor red(255, 0, 0), blue(0, 0, 255), yellow(255, 255, 0);
TColor colorArray[10];

//-- Page 100 ------------------------------------------------------------------

int delay = 0;
int xx;
int xx2;
int cx, cy;
char SaveFileName[40] = DEFDATAFILENAME;
int MagPV = DEFMAGPIXELSVOLT, MagPD = DEFMAGPIXELSDEGREE;
int yGMax yGMin;

// ALEXANDER GLOBAL VARIABLES
float huge ep[MAX_STEPS];
float huge vr[MAX_STEPS];
float huge co[MAX_STEPS];

int PlotOldData = -1;
int tMax;
int ControlType = NEURAL_ACE_ASE; // Default PID

struct TDataStruct
{
  char DataFileName[MAXDATAFILENAMELEN];
  char NumOutPoints[MAXPOINTDIG];
  char NumSkipPoints[MAXPOINTDIG];
  BOOL CollectSync;
  WORD LifeTimes;
  WORD States;
};

struct TFrequency
{
  char Freq[5];
};

struct PIDStruct
{
  WORD SingleLoop;
  WORD DualLoop;
};

struct RUNStruct
{
  WORD KeepSimGoing;
  WORD SaveWeightsToMemory;
  WORD SaveWeightsToFile;
  WORD DontSaveWeights;
};

struct NeuralACEASEStruct
{
  WORD ZeroiseWeights;
  WORD UseSimulationWeights;
  WORD WeightFromFile;
  char WeightFileName[40];
  WORD Uniform;


  //-- Page 101 ------------------------------------------------------------------

  WORD Nonuniform;
  WORD CMAC;
  char NumThetaBoxes[10];
  char NumDThetaBoxes[10];
  char ThetaExtreme[10];
  char DThetaExtreme[10];
  char Alpha[40];
  WORD OutSigmoid;
  WORD OutBangBang;
  WORD DisturbanceYes;
  WORD DisturbanceNo;
  WORD RBF;
} NeuralACEASEOptions;

struct TCalibration
{
  WORD DirectlyDriven;
  WORD CenterPosition;
  WORD EndPosition;
  char PoleAngle[10];
};

struct TGraphics
{
  WORD GraphicsOn;
  WORD GraphicsOff;
  char PixelsPerVolt[10];
  char PixelsPerDegree[10];
};

struct TWindowOptions
{
  char WindowTitle[TITLE_SIZE];
};

struct TSinWavRefParam
{
  char Freq[10];
  char Amp[10];
};

#include "ip.h"
#include "unh_cmac.h" // CMAC header files
#include "unh_cmac.c"  //  CMAC functions

extern "C"
{
#include "alexfunc.c"  //  Chris Alexanders Functions
}

#include "ipdlg3_1.cpp" // Definitions for Dialog functions
#include "acease48.cpp" // Neural ACE, ASE control algorithm
                        //------------------------------------------------------------------------------
class TIPWindow : public TFrameWindow
{
  int xMax, yMax; // Max client area coordinates
  int yBase;


//-- Page 102 ------------------------------------------------------------------

  int yOffset, xOffset;

  public:
  TIPWindow(TWindow* parent);
  ~TIPWindow(); // For Destructor
  const PIDStruct* GetPIDOptions() { return &PIDOptions; }
  const TCalibration* GetCalibOptions() { return &Calibration; }
  const TGraphics* GetGraphics() { return &Graphics; }
  const RUNStruct* GetRUNOptions() { return &RUNOptions; }
  float far NeuralACEASE(int IC, int jj, double states[]);
  void ClusterInputSpace(TDC&);
  void TrainCMAC();
  int RunWeightSave();
  void CmEmpInput();
  void CmFrequency();
  void SetupGraph(TDC&);
  void Graph(float far Angle[], float far RefInput[], float far CompOutput[], int TimeStep);
  char DataFileName[50];
  char MasterFileName[50];
  char FilePath[200];
  void SetMaxCoordinates();
  void DrawGrid(TDC& dc);
  BOOL EvEraseBkgnd(HDC);
  void Line(HDC hDC, int x1, int y1, int x2, int y2, COLORREF color);
  void EvChar(UINT, UINT, UINT);
  int NumOfDataPoints;

  private:
  TDataParamStruct DataParamStruct;
  TFrequency Frequency;
  PIDStruct PIDOptions;
  RUNStruct RUNOptions;
  TCalibration Calibration;
  TGraphics Graphics;
  TWindowOptions WindowOptions;
  TSinWavRefParam SinWavRefParam;
  TMenu* windowMenu;

  protected:
  void Paint(TDC&, BOOL, TRect&);
  void SetupWindow();
  void CmFileExit();
  void CmSimulate();
  void CmNIDAQEnable();
  void CmSetupData();
  void InitOptions();
  TOpenSaveDialog::TData* FileData;
  void CmFileOpen();
  void CmFileSave();
  void CmPID();
  void CmNeuralACEASE();

//-- Page 103 ------------------------------------------------------------------
  void CmCalibration();
  void CmDisplay();
  void CmOldDataGraph();
  void CmBeginControl();
  void CmRefSineWave();
  void CmRefSeriesOfSteps();
  void EvSize(UINT sizeType, TSize& size);
  void EvKeyDown(UINT key, UINT repeatCount, UINT flags);
  void EvMouseMove(UINT modKeys, TPoint& point);
  void EvMButtonDown(UINT modKeys, TPoint& point);
  DECLARE_RESPONSE_TABLE(TIPWindow);
};

DEFINE_RESPONSE_TABLE1(TIPWindow, TFrameWindow)
  EV_WM_KEYDOWN,
  EV_WM_MOUSEMOVE,
  EV_WM_MBUTTONDOWN,
  EV_WM_ERASEBKGND,
  EV_COMMAND(CM_FREQUENCY, CmFrequency),
  EV_COMMAND(CM_FILEOPEN, CmFileOpen),
  EV_COMMAND(CM_FILESAVE, CmFileSave),
  EV_COMMAND(CM_FILEEXIT, CmFileExit),
  EV_COMMAND(CM_SETUPSIMULATE, CmSimulate),
  EV_COMMAND(CM_SETUPNIDAQENABLE, CmNIDAQEnable),
  EV_COMMAND(CM_SETUPDATA, CmSetupData),
  EV_COMMAND(CM_PIDCONTROL, CmPID),
  EV_COMMAND(CM_NEURAL_ACEASE, CmNeuralACEASE),
  EV_COMMAND(CM_CALIBRATION, CmCalibration),
  EV_COMMAND(CM_DISPLAYSETUP, CmDisplay),
  EV_COMMAND(CM_OLD_DATA_GRAPH, CmOldDataGraph),
  EV_COMMAND(CM_BEGINCONTROL, CmBeginControl),
  EV_COMMAND(CM_SINEWAVE, CmRefSineWave),
  EV_COMMAND(CM_SERIESOFSTEPS, CmRefSeriesOfSteps),
  END_RESPONSE_TABLE;

// MAIN WINDOW CONSTRUCTOR(INITIALIZATION)
TIPWindow::TIPWindow(TWindow* parent) : TFrameWindow(parent, ""), TWindow(parent, "")
{
  SIZE size;
  TClientDC dc(*this);

  GetTextExtentPoint(dc, "A", 1, &size);  // A is arbitrary, gets text size
  cx = size.cx; cy = size.cy;
  FileData = new TOpenSaveDialog::TData(OFN_HIDEREADONLY | OFN_FILEMUSTEXIST,
      "Data Files (*.dat)|*.dat|Weight Files (*.wgt)|*.wgt|Master Files (*.fle)|*.fle|All Files (*.*)|*.*|",0,"DAT","WGT");

  //-- Page 104 ------------------------------------------------------------------
  AssignMenu(200);
  InitOptions();
  Attr.X = GetSystemMetrics(SM_CXSCREEN) / 8;
  Attr.Y = GetSystemMetrics(SM_CYSCREEN) / 8;
  Attr.H = Attr.Y * 6;
  Attr.W = Attr.X * 6;
}

void TIPWindow::SetupWindow()
{
  TFrameWindow::SetupWindow();
  // Create Menu Object Interface (for check marks on NIDAQ ENABLE and SIM)
  windowMenu = new TMenu(HWindow);
  if (NIDAQENABLE) // SETUP REAL-TIME vs. SIMULATION
    windowMenu -> CheckMenuItem(CM_SETUPNIDAQENABLE, MF_BYCOMMAND | MF_CHECKED);
  else
    windowMenu -> CheckMenuItem(CM_SETUPSIMULATE, MF_BYCOMMAND | MF_CHECKED);

  SetCaption(WindowOptions.WindowTitle);
  SetMaxCoordinates();
  xOffset = 10; yOffsetyMax/2;
  board = getBoardToUse();  // int board is in case of >1 boards on system.
}

// DESTRUCTOR: They do this to free up memory.
TIPWindow::~TIPWindow()
{
  delete windowMenu;
}


void TIP Window::InitOptions()
{
  strcpy(MasterFileName, DEFMASTERFILENAME);
  initial_values();
  wsprintf(WindowOptions.WindowTitle, "%s%s", DEF_WINDOW_TITLE, DEFDATAFILENAME);
  strcpy(DataParamStruct.DataFileName, DEFDATAFILENAME);
  strcpy(DataParamStruct.NumOutPoints, DEFNUM_OUTPOINTS);
  strcpy(DataParamStruct.NumSkipPoints, DEFNUM_SKIPPOINTS);
  DataParamStruct.CollectSync = DEF_COLLECTSYNC;
  DataParamStruct.LifeTimes = TRUE;
  DataParamStruct.States = FALSE;
  strcpy(Frequency.Freq, DEF_FREQUENCY);

//-- Page 105 ------------------------------------------------------------------

  PIDOptions.SingleLoop = FALSE;
  PIDOptions.DualLoop = TRUE;
  RUNOptions.KeepSimGoing = TRUE;
  RUNOptions.SaveWeightsToMemory = FALSE;
  RUNOptions.SaveWeightsToFile = FALSE;
  RUNOptions.DontSaveWeights = FALSE;
  Calibration.DirectlyDriven = FALSE;
  Calibration.CenterPosition = FALSE;
  Calibration.EndPosition = FALSE;
  NeuralACEASEOptions.ZeroizeWeights = FALSE;
  NeuralACEASEOptions.UseSimulationWeights = TRUE;
  NeuralACEASEOptions.WeightFromFile = FALSE;
  strcpy(NeuralACEASEOptions.WeightFileName, DEFWEIGHTFILENAME);
  NeuralACEASEOptions.Uniform = TRUE;
  NeuralACEASEOptions.Nonuniform = FALSE;
  NeuralACEASEOptions.CMAC = FALSE;
  NeuralACEASEOptions.RBF = FALSE;

  NumOfNodes NumThetaBoxes NumDThetaBoxes;

  wsprintf(NeuralACEASEOptions.NumThetaBoxes, "%i", DEF_NUM_OF_THETA_BOXES);
  wsprintf(NeuralACEASEOptions.NumDThetaBoxes, "%i", DEF_NUM_OF_DTHETA_BOXES);
  sprintf(NeuralACEASEOptions.ThetaExtreme, "%6.2f", ThetaExtreme);
  sprintf(NeuralACEASEOptions.DThetaExtreme, "%6.2f", DThetaExtreme);
  wsprintf(NeuralACEASEOptions.Alpha, "%i", DEFALPHA);
  NeuralACEASEOptions.OutSigmoid = FALSE;
  NeuralACEASEOptions.OutBangBang = TRUE;
  NeuralACEASEOptions.DisturbanceYes = FALSE;
  NeuralACEASEOptions.DisturbanceNo = TRUE;

  if (gear_type == 1)
    Calibration.DirectlyDriven = TRUE;
  else if (gear_type == 2)
    Calibration.CenterPosition = TRUE;
  else if (gear_type == 3)
    Calibration.EndPosition = TRUE;

  Graphics.GraphicsOn = TRUE;
  Graphics.GraphicsOff = FALSE;
  sprintf(Graphics_PixelsPerVolt, "%i", DEFMAGPIXELSVOLT);
  sprintf(Graphics.PixelsPerDegree, "%i", DEFMAGPIXELSDEGREE);

  xx = cx * 5 + 1;
  xx2 = cx * 5 + 11; // Initial Graphing parameters

  wsprintf(SinWavRefParam.Freq, "%i", DEFSINFREQ);
  wsprintf(SinWavRefParam.Amp, "%i", DEFSINAMP);

  colorArray[0] = white;
  colorArray[1] = yellow;
  colorArray[2] = blue;

//-- Page 106 ------------------------------------------------------------------
  colorArray[3] = green;
  colorArray[4] = red;
  colorArray[5] = white;
  colorArray[6] = yellow;
  colorArray[7] = blue;
  colorArray[8] = green;
  colorArray[9] = red;
  colorArray[10] = white;
}  // End of TipWindow::InitOptions()



void TIPWindow::Paint(TDC& dc, BOOL, TRect&)
{
  int t;
  if (PlotOldData == 1 || Graphics GraphicsOn)
  {
    Setup Graph(dc); // Initialize Graphics Window // Plot Data
    for (t = 0; (t < xMax) && (t < tMax); t++)
    {
      Line(dc, t + xOffset, ep[t] + yOffset, t+1 + xOffset, ep[t] + yOffset, blue);
      Line(dc, t + xOffset, vr[t] + yOffset, t+1 + xOffset, ep[t+1] + yOffset, green);
      Line(dc, t + xOffset, co[t] + yOffset, t+1 + xOffset, co[t+1] + yOffset, red);
    }
  }
}



BOOL TIPWindow::EvEraseBkgnd(HDC hDC)
{
  RECT rect;
  ::GetClientRect(HWindow, &rect);
  FillRect(hDC, &rect, HBRUSH(GetStockObject(BLACK_BRUSH)));
}



void TIPWindow::EvChar(UINT, UINT, UINT)
{
  TClientDC dc(*this);
  char s[100] = "YOU PRESSED A KEY";
  dc.TextOut(10, 10, s, strlen(s));
}



void TIPWindow::EvMouseMove(UINT modKeys, TPoint& point)
{
  if (NIDAQENABLE) AO_VWrite(board, 0, 0.0);
  if (delay > 500) { data_rec = -1; delay = 0; }
}



void TIPWindow::EvMButtonDown(UINT modKeys, TPoint &point)
{
  if (delay > 500) { data_rec = -1; delay = 0; }
}



//-- Page 107 ------------------------------------------------------------------

void TIPWindow::EvKeyDown(UINT key, UINT, UINT)
{
  TClientDC dc(*this);
  FILE *fptr;
  int ii;
  static float del, buff;
  char c;
  if (IC == 1)
  {
    goto NOIC;
  }
  /* Set "del" to the change in the digital compensator coefficient
   * and set "buff" to the digital compensator gain
   */
  ii = 0;
  del = delta_g[0];
  buff = c_gain[0];
NOIC:
  dc.TextOut(10, 10, "EvKeyDown", 9);
  switch (key)
  {
    case 82: // R key pressed
      rotate_buff = rotate_buff + 1;
      if (rotate_buff > 1) rotate_buff = 0;
      switch (rotate_buff)
      {
        case 0:
          buff = c_gain[0];
          del = delta_g[0];
          break;
        case 1:
          buff = c_gain[1];
          del = delta_g[1];
          break;
      }

      /*
       *  If a "U" is pressed, increase the digital gain by the given amount
       *  ("delta_g"). If a "u" is pressed, increase the digital gain by the
       *  given amount divided by 10. Both "D" & "d" do the same thing except
       *  they decrease the digital gain.
       */

    case 85: // U pressed
      buff = buff + del;
      goto jp;
      break;

    case 90: // Z pressed, Before Alexander used 'u'
      buff = buff + (del / 10.0);
      goto jp;

//-- Page 108 ------------------------------------------------------------------

      break;

    case 68:  // D pressed
      buff = buff - del;
      goto jp;
      break;

    case 89:  // Y pressed, Before Alexander used 'd'
      buff = buff - (del/10.0);

jp:
      switch (rotate_buff)
      {
        case 0:
          c_gain[0] = buff;
          delta_g[0] = del;
          break;
        case 1:
          c_gain[1] = buff;
          delta_g[1] = del;
          break;
      }
      break;

    case 83:  
      /*  S pressed: If a "S" is pressed set the "data_rec" flag high start data
       *  recording. If any other key is pressed (except those mentioned
       *  above) set the flag with low logic (stop the control of the pendulum).
       */
      data_rec = 1;
      break;

    case 67:  // C pressed
      calibrate = current_measure1;
      break;

    case VK_SPACE:
      if (NIDAQENABLE) AO_VWrite(board, 0, 0.0);
      data rec=-1;
      break;

    default:
      if (NIDAQENABLE)
        AO_VWrite(board, 0, 0.0);
      /*
       * If record flag is set put system variables (position,
       * compensator output, reference and other info) into a file.
       */

      if (data_rec == 1 || sync == 1)
      {
        sprintf(SaveFileName, "%s.dat\0", DataParamStruct.DataFileName);
        NumOfDataPoints = jj;
        CmFileSave();
        sprintf(DataParamStruct.DataFileName, "%s.dat\0", SaveFileName);
      }

//-- Page 109 ------------------------------------------------------------------
      if (delay > 500)
      {
        data_rec = -1;
        delay = 0;
      }
      break;
  }  // END CASE
}



void TIPWindow::Line(HDC hDC, int x1, int y1, int x2, int y2, COLORREF color)
{
  HPEN hPen = CreatePen(PS_SOLID, 1, color);
  HPEN hOldPen = HPEN(SelectObject(hDC, hPen));

  // Keep Everything Inside Display Context
  if (x1 < 0 || x1 > xMax) x1 = 0;
  if (y1 < yGMin) y1 = yGMin;
  else if (y1 > yGMax) y1 = yGMax;
  if (x2 < 0 || x2 > xMax) x2 = 0;
  if (y2 < yGMin) y2 = yGMin;
  else if (y2 > yGMax) y2 = yGMax;

  MoveTo(hDC, x1, y1);
  LineTo(hDC, x2, y2);
  SelectObject(hDC, hOldPen);
  DeleteObject(hPen);
}



void TIPWindow::SetupGraph(TDC& dc)
{
  SIZE size;
  TPen pen(white);
  TPen* pen2;
  RECT rect;
  char txt[40] = "", txt2[40] = "";
  int htxt = 0;
  // Define Safe haven for text (graph will not overwrite this region)
  yGMin = 3*cy; yGMax = yMax-3*cy;  // Allows 4 lines of text to be displayed on top and bottom

  dc.SetBkColor(black);
  dc.SetTextColor(white);
  dc.SetTextAlign(TA_CENTER);  // Aligns text about its center
  sprintf(txt, "FILE: %s", SaveFileName);
  dc.TextOut(xMax/2, 10, txt, strlen(txt));
  dc.SetTextAlign(TA_LEFT|TA_TOP);  // Back to default
  dc.SetBkColor(black);
  dc.SetTextColor(blue);
  strcpy(txt, "Blue=Encoder Output (Angle in degrees)");
  dc.TextOut(10 + cx*6, yMax - (10 + cy*2), txt, strlen(txt));
  dc.SetTextColor(green);
  htxt = strlen(txt);
  strcpy(txt, "Green=Reference Input");
  dc.TextOut(10 + cx*(4 + htxt), yMax - (10 + cy*2), txt, strlen(txt));
//-- Page 110 ------------------------------------------------------------------
  dc.SetTextColor(red);
  htxt+= strlen(txt);
  strcpy(txt, "Red=Compensator Output");
  dc.TextOut(10 + cx*(4 + htxt), yMax - (10 + cy*2), txt, strlen(txt));
  dc.SelectObject(pen);
  // Plot Horiz. and Vert. Axis
  Line(dc, cx*2, yOffset, xMax-10, yOffset, white); // Time Axis
  dc.TextOut(10, yOffset, "0.0", 3);
  Line(dc, cx*5, yGMin, cx*5, yGMax, white);   // Vertical Axis
  dc.RestorePen();
  dc.SetTextColor(yellow);
  strcpy(txt, "COMPENSATOR TYPE: ");

  switch (ControlType)
  {
    case PID:
      strcpy(txt2, "PID");
      break;
    case NEURAL_ACE_ASE:
      strcpy(txt2, "Neural ACE & ASE");
      break;
  }
  strcat(txt, txt2);
  dc.TextOut(10 + cx*6, 10 + cy*1, txt, strlen(txt));  //  ?? cy*1
  htxt = strlen(txt);
  strcpy(txt, "RUN STATUS: ");
  if (SIMULATE)
    strcat(txt, "SIMULATION");
  else
    strcat(txt, "REAL-TIME CONTROL");
  dc.TextOut(10 + cx*(16 + htxt), 10 + cy*1, txt, strlen(txt));  //  ??
}



void TIPWindow::Graph(float far Angle[], float far RefInput[], float far CompOutput[], int jj)
{
  TClientDC dc(*this);
  char Buff[100];
  int al, a2, rl, r2, cl, c2;
  float kPD = MagPD*Rad2Ang;
  // Define Variables To Graph
  a1 = Angle[jj-1]*kPD/Rad2Ang + yOffset;
  a2 = Angle[jj]*kPD + yOffset;
  r1 = RefInput[jj-1] * kPD + yOffset;
  r2 = RefInput[jj]*kPD + yOffset;
  c1 = CompOutput[jj-1]*MagPV + yOffset;
  c2 = CompOutput[jj]*MagPV + yOffset;

  // Check if Graph at right edge of window. If so, move to vertical axis
  xx++;
  xx2++;
  if (xx >= xMax) xx = cx*5;
  if (xx2 >= xMax) xx2 = cx*5;

//-- Page 111 ------------------------------------------------------------------

  // Erase Old Graph
  if (xx2 != cx*5)  // cx*5 is Vertical (Angular) Axis Line
  {
    Line(dc, xx2, yGMin, xx2, yGMax, black);  // Eraser Line
    Line(dc, xx2, yOffset, xx2+1, yOffset, white);  // Refresh time axis pixel
  }
  else
    Line(dc, xx2, yGMin, xx2, yGMax, white);  // Refresh vertical axis

  // Draw New Graph
  Line(dc, xx, a1, xx+1, a2, blue);
  Line(dc, xx, r1, xx+1, r2, green);
  Line(dc, xx, c1, xx+1, c2, red);

  dc.SetTextColor(white);
  dc.SetBkColor(black);
  if (SIMULATE)
    sprintf(Buff, "Simulation Run Time: %10.2f seconds\0", systime[steps]);
  else
    sprintf(Buff, "Run Time: %10.2f seconds\0", jj/fs);

  // Display Numerical Values
  dc.SetTextAlign(TA_CENTER);  // Aligns text about its center
  dc.TextOut(xMax/2, 10 + 2*cy, Buff, strlen(Buff));
  dc.SetTextAlign(TA_LEFT|TA_TOP);  // back to default
  sprintf(Buff, "Encoder=%6.2f Refinput=%6.2f CompOutput=%6.2f \0", Angle[jj], RefInput[jj], CompOutput[jj]);
  dc.SetTextColor(yellow);
  dc.TextOut(10 + 6*cx, yMax - (10 + cy*1), Buff, strlen(Buff));
}


//  Set up maximum coordinate values
void TIPWindow::SetMaxCoordinates()
{
  TRect rect;
  GetClientRect(rect);
  xMax = rect.right;
  yMax = rect.bottom;
  yBase =  yMax - SPACEATBOTTOM;
}


void TIPWindow::EvSize(UINT sizeType, TSize& size)
{
  TFrameWindow::EvSize(sizeType, size);
  SetMaxCoordinates();
  Invalidate();
  UpdateWindow();  // Optional
}


//-- Page 112 ------------------------------------------------------------------

void TIPWindow::CmNIDAQEnable()  // Allow Real-Time Control
{
  windowMenu->CheckMenuItem(CM_SETUPNIDAQENABLE, MF_BYCOMMAND | MF_CHECKED);
  NIDAQENABLE = 1;
  SIMULATE = 0;
  windowMenu -> CheckMenuItem(CM_SETUPSIMULATE, MF_BYCOMMAND | MF_UNCHECKED);
}



void TIPWindow::CmSimulate()  // Allow Simulation
{
  windowMenu -> CheckMenuItem(CM_SETUPSIMULATE, MF_BYCOMMAND | MF_CHECKED);
  NIDAQENABLE = 0;
  SIMULATE = 1;
  windowMenu -> CheckMenuItem(CM_SETUPNIDAQENABLE, MF_BYCOMMAND | MF_UNCHECKED);
}



void TIPWindow::CmSetupData()
{
  char DataParamInfo[sizeof(TDataParamStruct) + 5 + 1];
  char *ss;
  if (TDataDlg(this, "DATA", DataParamStruct).Execute() == IDOK) {
    strcpy(DataParamInfo, DataParamStruct.DataFileName);
    strcat(DataParamInfo, "\n");
    strcat(DataParamInfo, DataParamStruct.NumOutPoints);
    strcat(DataParamInfo, "\n");
    strcat(DataParamInfo, DataParamStruct.NumSkipPoints);
    strcat(DataParamInfo, "\n");
    MessageBox(DataParamInfo, "Data Param Settings", MB_OK);
    ss = "You Selected Ok";
  }

  else
    ss = "You Selected Cancel";
  MessageBox(ss, GetApplication()->GetName(), MB_OK);
}



void TIPWindow::CmFrequency()
{
  char s[40];
  TClientDC dc(this);
  if (TFreqDlg(this, "FREQUENCY", Frequency).Execute() == IDOK)
  {
    wsprintf(s, "%s", "You Selected Frequency");
    dc.TextOut(10, 20, s, strlen(s));
    fs = atoi(Frequency.Freq);
  }
}


//-- Page 113 ------------------------------------------------------------------

void TIPWindow::CmPID()
{
  TPIDDlg* PIDDlg = new TPIDDlg(this, &PIDOptions);
  ControlType = PID;

  windowMenu->CheckMenuItem(CM_PIDCONTROL, MF_BYCOMMAND | MF_CHECKED);
  windowMenu->CheckMenuItem(CM_NEURAL_ACEASE, MF_BYCOMMAND | MF_UNCHECKED);

  if (PIDDlg->Execute() == IDOK) ControlType = PID;
}

void TIPWindow::CmNeuralACEASE() // Setup
{
  char ext[5] = "";
  char *p1, *p2, fileloc[200] = "";
  char filename[200] = "";
  FILE *fdata;
  int i;

  TNeuralACEASEDlg *NeuralACEASEDlg = new TNeuralACEASEDlg(this, NeuralACEASEOptions);
  windowMenu->CheckMenuItem(CM_NEURAL_ACEASE, MF_BYCOMMAND | MF_CHECKED);
  windowMenu->CheckMenuItem(CM_PIDCONTROL, MF_BYCOMMAND | MF_UNCHECKED);

  ControlType = NEURAL_ACE_ASE;
  if (NeuralACEASEDlg->Execute() == IDOK)
  {
    NumThetaBoxes = atoi(NeuralACEASEOptions.NumThetaBoxes);
    NumDThetaBoxes = atoi(NeuralACEASEOptions.NumDThetaBoxes);

    if (NeuralACEASEOptions.Uniform)
      NumOfNodes = NumThetaBoxes * NumDThetaBoxes;
    else
      NumOfNodes = NUM_OF_ISNODES;

    ThetaExtreme = atof(NeuralACEASEOptions.ThetaExtreme);
    DThetaExtreme = atof(NeuralACEASEOptions.DThetaExtreme);

    MessageBox("Neural ASE ACE Controller Selected", "Neural ACEASE", MB_OK);

    if (NeuralACEASEOptions.ZeroizeWeights)
      OtherWeights = 0;
    if (NeuralACEASEOptions.UseSimulationWeights)
      OtherWeights = 0;

    if (NeuralACEASEOptions.WeightFromFile)
    {
      OtherWeights = 1;
      FileData = new TOpenSaveDialog::TData(
          OFN_HIDEREADONLY | OFN_FILEMUSTEXIST,
          "Weight Files (*.wgt)|*.wgt|All Files (*.*)|*.*|", 0, "WGT", "*");

      strcpy(FileData->FileName, NeuralACEASEOptions.WeightFileName);
      if (TFileOpenDialog(this, *FileData).Execute() == IDOK)
      {
        ifstream is(FileData->FileName);
        strcpy(fileloc, FileData->FileName);

        if (!is)
        {
          MessageBox("Unable to open file", "File Error", MB_OK | MB_ICONEXCLAMATION);
        }
        else
        {
          // Convert fileloc to lowercase
          for (i = 0; i < strlen(fileloc); i++)
            fileloc[i] = tolower(fileloc[i]);

          if ((p2 = strchr(fileloc, '.')) != NULL)
            strcpy(ext, p2 + 1);

          p1 = strrchr(fileloc, "\\\\"); // Corrected double backslash
          if (p1)
          {
            strncat(filename, p1 + 1, strlen(p1) - strlen(ext) - 2);
          }

          if (strcmp(ext, "wgt") == 0)
          {
            strcpy(NeuralACEASEOptions.WeightFileName, filename);
            if ((fdata = fopen(fileloc, "rt")) == NULL)
            {
              MessageBox("Cannot open input file.\n", "Open Error", MB_OK);
            }
            else
            {
              i = 0;
              while (fscanf(fdata, "%f%f%f%f", &wt[i], &vt[i], &elg[i], &xbar[i]) == 4)
              {
                i++;
              }

              if (i != (NumOfNodes + 1))
              {
                MessageBox("Number of Boxes Does not Match: May Cause Poor Performance",
                    "Data Mismatch Error", MB_OK);
              }
              fclose(fdata);
            }
          }
        }
      }
      Invalidate();
      delete FileData;
    }
  }
}



void TIPWindow::CmCalibration()
{
  if (!NIDAQENABLE)
  {
    MessageBox("Please select NIDAQ under setup to calibrate", "CALIBRATE", MB_OK);
    return;
  }

  TCalibDlg *CalibDlg = new TCalibDlg(this, &Calibration);
//-- Page 115 ------------------------------------------------------------------
  char Degrees[10];

  temp_graph_output = graph_output;
  graph_output = 0;
  cal_jmp = 1;
  out = 2;
  calibrate = 0.0;
  pos2 = 0.0;
  previous_measure = 0.0;

  snprintf(Degrees, sizeof(Degrees), " %.2f", Rad2Ang * Digital_Input(board, cal_jmp, IC));
  strcpy(Calibration.PoleAngle, Degrees);

  CalibDlg->Execute();
  delete CalibDlg;  // Prevent memory leak
}

void TIPWindow::CmDisplay()
{
  TGraphicsDlg* GraphicsDig = new TGraphicsDig(this, &Graphics);
  if (GraphicsDlg->Execute() == IDOK)
  {
    MessageBox("Graphics OK", "DISPLAY", MB_OK);
    MagPV = atoi(Graphics.PixelsPerVolt);
    MagPD = atoi(Graphics.PixelsPerDegree);
  }
}



void TIPWindow::CmFileOpen()
{
  int i = 0, j;
  char ext[5] = "";
  char *p1, *p2, fileloc[200] = "";
  char filename[200] = "";
  FILE *fdata;
  int MaxYAxis = 10;

  // Initialize FileData before using it
  FileData = new TOpenSaveDialog::TData(OFN_HIDEREADONLY | OFN_FILEMUSTEXIST, "Data Files (*.dat)|*.dat|All Files (*.*)|*.*|", 0, "DAT", "*");

  if (TFileOpenDialog(this, *FileData).Execute() == IDOK)
  {
    ifstream is(FileData->FileName);
    strcpy(fileloc, FileData->FileName);

    if (!is)
    {
      MessageBox("Unable to open file", "File Error", MB_OK | MB_ICONEXCLAMATION);
    }
    else
    {
      // Convert filename to lowercase
      for (i = 0; i < strlen(fileloc); i++)
        fileloc[i] = tolower(fileloc[i]);

      // Extract file extension
      if ((p2 = strchr(fileloc, '.')) != NULL)
        strcpy(ext, p2 + 1);

      // Extract filename (without path)
      p1 = strrchr(fileloc, "\\");
      if (p1)
      {
        size_t len = strlen(p1) - strlen(ext) - 2;
        if (len > sizeof(filename) - 1)
          len = sizeof(filename) - 1;
        strncat(filename, p1 + 1, len);
        filename[len] = '\0';
      }

      if (strcmp(ext, "fle") == 0)
      {
        strcpy(MasterFileName, filename);
        read_mfile();
      }
      else if (strcmp(ext, "dat") == 0)
      {
        wsprintf(WindowOptions.WindowTitle, "%s%s", DEF_WINDOW_TITLE, filename);
        strcpy(DataParamStruct.DataFileName, filename);
        SetCaption(WindowOptions.WindowTitle);

        if ((fdata = fopen(fileloc, "rt")) == NULL)
        {
          MessageBox("Cannot open input file.\n", "Open Error", MB_OK);
        }
        else
        {
          i = 0;
          ratio = 1;
          MaxAngMag = MaxAngVelMag = 1;

          while (fscanf(fdata, "%f", &ang[i]) == 1)
          {
            if (fabs(ang[i]) > MaxAngMag)
              MaxAngMag = fabs(ang[i]);
            if (i > 0 && fabs((ang[i] - ang[i - 1]) * fs) > MaxAngVelMag)
              MaxAngVelMag = fabs((ang[i] - ang[i - 1]) * fs);
            i++;
          }

          NumOfDataPoints = i;
          fclose(fdata);

          // Only Invalidate if the file was successfully opened and read
          Invalidate();
        }
      }
    }
  }
}



void TIPWindow::CmFileSave()
{
  char ext[5] = "";
  char *p1, *p2, fileloc[200] = "";
  char filename[200] = "";
  FILE *fdata;
  int i;
  float temp;

  FileData = new TOpenSaveDialog::TData(OFN_HIDEREADONLY | OFN_FILEMUSTEXIST, "Weight Files(*.wgt)|*.wgt|Data Files (*.dat)|*.dat|All Files (*.*)|*.*|", 0, "WGT", "DAT");

  strcpy(FileData->FileName, SaveFileName);

  if (TFileSaveDialog(this, *FileData).Execute() == IDOK)
  {
    ofstream is(FileData->FileName);
    strcpy(fileloc, FileData->FileName);

    if (!is)
    {
      MessageBox("Unable to Save file", "File Error", MB_OK | MB_ICONEXCLAMATION);
    }
    else
    {
      // Convert file name to lowercase
      for (i = 0; i < strlen(fileloc); i++)
        fileloc[i] = tolower(fileloc[i]);

      // Get file extension
      if ((p2 = strchr(fileloc, '.')) != NULL)
      {
        strcpy(ext, p2 + 1);
        p1 = strrchr(fileloc, '\\');
        strncat(filename, p1 + 1, strlen(p1) - strlen(ext) - 2);
      }

      if ((fdata = fopen(fileloc, "wt")) == NULL)
      {
        MessageBox("Cannot write to file.\n", "Open Error", MB_OK);
      }
      else
      {
        strcpy(SaveFileName, filename); // Update save file name

        if (strcmp(ext, "wgt") == 0)
        {
          for (i = 1; i < NumOfNodes; i++)
          {
            //  Write Weights to a File
            fprintf(fdata, "%f %f %f %f\n", wt[i], vt[i], elg[i], xbar[i]);
          }
          fclose(fdata);
          MessageBox(filename, "SAVED WEIGHT FILE", MB_OK);
        }
        else if (strcmp(ext, "dat") == 0)
        {
          if (!SIMULATE)
          {
            for (i = 0; i < jj; i++)
              fprintf(fdata, "%10.3f%10.3f%10.3f\n", encoder_position[i], volt[i], V_reff[i]);
          }
          else if (DataParamStruct.States)
          {
            for (i = 0; i < steps; i++)
              fprintf(fdata, "%10.3f\n", ang[i]);
          }
          else
          {
            fprintf(fdata, "BangBangMag=%f\n", BangBangGain);
            fprintf(fdata, "NumThetaBoxes=%i\n", NumThetaBoxes);
            fprintf(fdata, "NumDThetaBoxes=%i\n", NumDThetaBoxes);
            fprintf(fdata, "MAX_STEPS=%i\n", MAX_STEPS);
            fprintf(fdata, "MAX_TRIALS=%i\n", MAX_TRIALS);
            fprintf(fdata, "MAX_RUNS=%i\n", MAX_RUNS);

            for (RunNum = 0; RunNum < MAX_RUNS; RunNum++)
            {
              fprintf(fdata, "Number_of_Failures_per_Run=%i\n", NumRunFails[RunNum]);
              for (TrialNum = 0; TrialNum < MAX_TRIALS; TrialNum++)
              {
                fprintf(fdata, "%10i %10i %10 \n ", RunNum, TrialNum, LifeTime[RunNum][TrialNum]);
              }
            }
            RunNum = 0;
          }
          MessageBox(filename, "SAVED DATA FILE", MB_OK);
          fclose(fdata);
        }
        else
        {
          MessageBox(filename, "UNKNOWN FILE TYPE, NOTHING SAVED", MB_OK);
          fclose(fdata);
        }
      }
    }
    delete FileData;
    Invalidate();
  }
}



void TIPWindow::CmFileExit()
{
  CmExit();
}



void TIPWindow::CmOldDataGraph
{
  PlotOldData = -PlotOldData;
  strcpy(gs, "Inside CmOldDataGraph");
  Invalidate();
}



void TIPWindow::CmBeginControl()
{
  int start = 0, Mag = 500;
  char txt[30];
  static float oldx1 = 0;
  MSG msg;
  BOOL fRetVal = TRUE;
  TClientDC dc(*this);

  // Create Dialog
  TBeginControlDlg *BeginControlDlg = new TBeginControlDlg(this, BEGINCONTROLDIALOG);

  if (!SIMULATE)
    start = BeginControlDlg->Execute();

  delete BeginControlDlg; // Prevent memory leak

  if (start == IDCANCEL)
  {
    MessageBox("Control Cancelled", "CONTROL STOPPED", MB_OK);
    return;
  }
  else if (start == IDHELP)
  {
    std::string s;
    s += "HINT: Be sure that everything is setup properly\n";
    s += "under the Setup Menu. It is highly likely\n";
    s += "that the system will not work as expected\n";
    s += "if you do not have options set correctly.\n";
    s += "Try choosing Default under setup.\n";
    s += "If you are not sure, choose CANCEL.\n";
    MessageBox(s.c_str(), "Help!", MB_OK);
  }

  if (NIDAQENABLE)
  {
    board = getBoardToUse();
    err_num = DIG_Prt_Config(board, 0, 0, 0);
    errCheck(board, "DIG_Prt_Config", err_num);
    err_num = DIG_Prt_Config(board, 1, 0, 0);
    errCheck(board, "DIG_Prt_Config", err_num);
    err_num = AI_Config(board, 1, 1, 1);
    errCheck(board, "AI_Config", err_num);
  }

  jj = 1;
  data_rec = 0;
  IC = 0;
  rotate_buff = 0;
  graph_error = 0;
  jk = 0;
  jkk = 1;

  fcount = 2000000.0 / fs;
  fcount = floor(fcount);
  count = (unsigned int)fcount;

  if (Graphics.GraphicsOn)
    SetupGraph(dc);

  // Automatic Calibration
  Digital_Input(board, cal_jmp, IC);
  calibrate = current_measure1;
  V_reff[ij] = Ref(IC);

  while (true)
  {
    delay++;
    if (delay > 99999)
      delay = 0;

    if (NIDAQENABLE)
      ICTR_Setup(board, 0, 0, count, 1);

    errCheck(board, "ITCR_Setup", err_num);

    if (!SIMULATE || ControlType == PID)
      V_reff[jj] = Ref(IC);
    else
      V_reff[jj] = 0.0;

    encoder_position[jj] = Digital_Input(board, cal_jmp, IC);

    if (!SIMULATE)
    {
      states[0] = encoder_position[jj];
      states[1] = (states[0] - oldx1) * fs;
      oldx1 = states[0];
    }

    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
    {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }

    switch (ControlType)
    {
      case PID:
        volt[jj] = PIDController(IC, jj);
        break;
      case NEURAL_ACE_ASE:
        volt[jj] = NeuralACEASE(IC, jj, states) / 10.58;
        break;
    }

    if (Graphics.GraphicsOn)
      Graph(encoder_position, V_reff, volt, jj);

    if (NIDAQENABLE)
      AO_VWrite(board, 0, volt[jj]);

    if (data_rec < 0)
      break; // Replaces "goto end;"

    jj++;

    if (j == 15001)
      break; // Replaces "goto end;"

    if (jk == ndpts - 1)
    {
      sync = (in_sync == 1);
      jk = -1;
      jkk = 1;
    }

    jk++;

    if (NIDAQENABLE)
      AI_VRead(board, 1, 1, &countt);

    if (!NIDAQENABLE)
      countt = 1;

    if (jk == 5 * jkk)
    {
      if (NIDAQENABLE)
        ICTR_Read(board, 0, &t_count);
      per_t_rem = 100 * (1.0 - ((double)count - (double)t_count) / (double)count);
      jkk++;

      if (countt < 1.0)
        continue;
      else
        continue;
    }
  }
  if (NIDAQENABLE)
    AO_VWrite(board, 0, 0.0);
}



void TIPWindow::CmRefSineWave()
{
  if (TSinWavRefDlg(this, "SINE WAVE REF", SinWavRefParam).Execute() == IDOK)
  {
    MessageBox("Sine Wave Ref OK", SinWavRefParam.Amp, MB_OK);
    Ref_type = 0;
  }
}


//-- Page 122 ------------------------------------------------------------------

void TIPWindow::CmRefSeriesOfSteps()
{
  Ref_type = 1;
}


float far TIPWindow::NeuralACEASE(int IC, int jj, double states[])
{
  // Neural Control/Simulation: Returns Voltage to Motor if in Control Mode
  MSG msg;
  TClientDC dc(*this);
  int i, Mag = 5, Choice;
  float tt = 0;
  int dlg;
  char trialtxt[80], txt[40];

  if (!IC)
  {
    if (NeuralACEASEOptions.Nonuniform)
      ClusterInputSpace(dc);
    if (NeuralACEASEOptions.CMAC)
      TrainCMAC();
    InitializeNeuralACEASE(IC, OtherWeights);
  }

  randomize();

// main loop Neural ACE ASE:
NextTrial:

  if (SIMULATE)
  {
    // assume zero input for ASE at the 1st time step
    xx = cx * 5 + 1;
    xx2 = cx * 5 + 11; // reset graph line
    tt = 0;
    ace();
    ase(); // initial action
    Invalidate();
  }
  IC = 1;

NextStep:
  // Check for User Interrupt During Simulation
  if (SIMULATE && PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
  {
    TranslateMessage(&msg);
    DispatchMessage(&msg);
  }

  if (data_rec == -1)
  {
    // User Interrupt,Stop Run
    MessageBox("USER INTERRUPT", "ACE & ASE", MB_OK);
    goto Interrupt; // End Training
  }
  wsprintf(txt, "Runs=%i Trials=%i Steps=%i", RunNum, TrialNum + 1, steps);
  dc.SetTextColor(green);
  //-- Page 123 -----------------------------------------------------------------
  dc.SetBkColor(black);
  if (SIMULATE)
    steps++;
  else
    steps = jj;

  // force is voltage applied to windings (max/min +/-.2)
  force[steps] = BangBangGain * action;
  if (SIMULATE)
    PoleModelSolve();

  tt = tt + tstep;
  systime[steps] = tt;

  for (i = 0; i < NS; i++)
    y0[i] = states[i];

  ang[steps] = states[0] * Rad2Ang;

  if (fabs(ang[steps]) > MaxAngMag)
    MaxAngMag = fabs(ang[steps]);

  if ((fabs(ang[steps] - ang[steps - 1]) * fs) > MaxAngVelMag)
    MaxAngVelMag = fabs(ang[steps] - ang[steps - 1]) * fs;

  if (abs(states[0]) > FailAng)
  {
    failure = 1;
    reinf = -1;
    NumRunFails[RunNum] += 1;
  }
  decoder(); // Find which Box states are in
  predlast = pred;
  ace();
  ase(); // initial action

  if (SIMULATE && Graphics.GraphicsOn)
    Graph(ang, V_reff, force, steps); // DISPLAY GRAPH

  if (!failure)
  {
    for (i = 1; i <= NumOfNodes; i++)
    {
      elg[i] = Delta * elg[i] + (1 - Delta) * action * ISNode[i];
      xbar[i] = Lamda * xbar[i] + (1 - Lamda) * ISNode[i];
    }
  }

  dc.SetTextColor(white);
  dc.TextOut(xMax - cx * (strlen(txt) + 5), yMax - 10 - cy, txt, strlen(txt));

  if (!SIMULATE && !failure)
    return (float far)force[steps] / MOTORVOLT_GAIN;

  if (!failure && steps < (MAX_STEPS - 2) && SIMULATE)
    goto NextStep;

Interrupt: // User Interrupt or Failure Occurs then Jump to Here

  LifeTime[RunNum][TrialNum] = steps - prevt;
  TrialNum++;

  if (failure)
  {
    // If SIMULATE=1 END Trial or Failure Occurs, not time to reset.
    // Reset Variables for next trial
    for (i = 0; i < NS; i++)
      y0[i] = 0;
    // Find New Node Centers Based on Clustering of Input Data
    // From all Previous Trials
    failure = 0;
    reinf = 0;
    predlast = 0;
    for (i = 1; i <= NumOfNodes; i++)
    {
      //-- Page 124 ----------------------------------------------------
      elg[i] = elg0[i];
      xbar[i] = elg0[i];
      ISNode[i] = 0.0;
    }
    prevt = steps;
    if (SIMULATE)
    {
      // Running Simulation or Training
      if (TrialNum < MAX_TRIALS && steps < (MAX_STEPS - 2))
        goto Next Trial;
      else
      {
        randomize();
        InitializeNeuralACEASE(IC, 0);
        RunNum++;
        // MessageBox("Completed Run", "NEXT RUN",MB_OK);
        if (RunNum > MAX_RUNS)
        {
          RunNum = 0;
          TrialNum = 0;
          IC = 0;
          RunWeightSave();
          goto EndACEASE;
        }
        TrialNum = 0;
        goto NextTrial;
        // MessageBox("EXCEEDED MAX TRIALS","RUN STOPPED", MB_OK);
        // RunWeijghtSave();  // Execute Weight Save Options
      }
    }
    else
    {
      // Real-Time Control
      if (NIDAQENABLE)
        AO_VWrite(board, 0, 0.0);
      MessageBox("FAILURE HAS OCCURED! RUN AGAIN TO/ TRAIN", "CONTROL STOPPED", MB_OK);
      RunWeightSave(); // Execute Weight Save Options
      Invalidate();
      if (RUNOptions.KeepSimGoing)
        CmBeginControl();
    }
  }
  else
  {
    //  NO FAILURE: USER INTERRUPT OR SIM END (BEYOND MAX/ STEPS)
    if (steps >= (MAX_STEPS - 2))
    {
      // MessageBox("MAX STEPS EXCEEDED","END",MB_OK); //RunWeight Save(); // Execute Weight Save Options
      if (TrialNum < MAX_TRIALS)
      {
        steps - 1;
        prevt = 0;
        goto NextTrial;
      }
      else
      {
        randomize();
        InitializeNeuralACEASE(IC, 0);
        RunNum++;
        if (RunNum > MAX_RUNS)
        {
          RunNum = 0;
          TrialNum = 0;
          IC = 0;
          RunWeightSave(); // Execute Weight Save Options

          goto EndACEASE;
        }

        //-- Page 125 ---------------------------------------------------------------------

        TrialNum = 0;
        goto NextTrial;
      }
    }
    else
    {
      if (SIMULATE)
      { // Running Simulation or Training
        MessageBox("SIMULATION STOPPED", "USER INTERRUPT", MB_OK);
        Choice - RunWeightSave(); // Execute Weight Save Options
        if (RUNOptions.KeepSimGoing && steps < (MAX_STEPS - 2))
        {
          data_rec = 1;
        }
        delay = 0;
        goto NextStep : if (Choice IDCANCEL) goto NextStep;
        else
        { // Real-Time Control
          if (NIDAQENABLE)
            AO_VWrite(board, 0, 0.0); MessageBox("CONTROL STOPPED","USER/
                      INTERRUPT", MB_OK);
                  RunWeightSave(); // Execute Weight Save Options
        }
      }
    EndACEASE:
      if (SIMULATE)
        MessageBox("END SIMULATION/CONTROL", "END", MB_OK);
      data rec = 1;
      IC = 0;
      return 0.0; // Simulation Ends return arbitrary float value
    }
  }
}

void TIPWindow::ClusterInputSpace(TDC &dc)
{
  float DDMAXAVG = 0, DD = 0;
  static int a, i, j, ig.jg, k, xOff, yOff, G, pp;
  static float Nx, Ny, JT, OldJT, JTFirst, d1, d2, Gain = 2.0;
  static double xsum, ysum;
  int ri, cc, c - NumOfNodes, xo - xMax / 2, yo - yMax / 2; // xo x offset yo y offset
  int IN, ccm;
  static long r, th, p1, p2, si;
  static float a1, a2, a3, a4, a5, iscx, iscy;
  static double nx, ny, cxx, cyy;
  static float dd, dmin, dis;
  static float avgx, avgy, PG;
  float ThetaBoxSpacing = 2 * ThetaExtreme / NumThetaBoxes;
  float DThetaBoxSpacing = 2 * DThetaExtreme / NumDThetaBoxes;
  static char txt[50];
  int ridx;
  TRect rect;
  HPEN RedPen, BluePen, WhitePen, BlackPen :
      //-- Page 126 ---------------------------------------------------------------------
      HPEN hOldPen,
      GreenPen;
  IN = 0;
  sprintf(txt, "xMax=%i yMax-%i ", xMax, yMax);
  dc.TextOut(10, 10, txt, strlen(txt));
  Line(dc, xMax / 2, 10, xMax / 2, yMax - 10, yellow); // vertical
  Line(dc, 10, yMax / 2, xMax - 10, yMax / 2, yellow); // horizontal
  MessageBox("Choose Data File to Cluster", "CLUSTERING", MB_OK);
  CmFileOpen();
  if (NumOfDataPoints < 1)
  {
  }
  MessageBox("No Data File Chosen", "Error", MB_OK);
  return;
  // find center of mass of input space avgx=0; avgy=0;
  Nx - MaxAngMag : Ny - MaxAng VelMag;

  for (j = 1; j <= NumOfDataPoints; j++)
  {
    // angle and angular velocity normalized data between -1.0 and 1.0
    yyc[j] = ang[j] / Nx;
    xxc[j] = (ang[j] - angl[j - 1]) * fs / Ny;
    avgx += xxc[j];
    avgy += yyc[j]; // plot data to be clustered
    dc.Ellipse(xxc[j] * xMax / 2 + xo, yyc[j] * yMax / 2 + yo, xxc[j] * xMax / 2 + 5 + xo, yyc[j] * yMax / 2 + 5 + yo);
    iscx - avgx / NumOfDataPoints;
    iscy - avgy / NumOfDataPoints;
    // find closest data point to center of mass of input space data // start dmin with value of 1st data point (initial seed center) dmin-pow(xxc[1]-iscx,2)+pow(yyc[1]-iscy,2); ccm=1; for(j=1;j<=NumOfDataPoints;j++) {
  }
  dis - pow(xxc[j] - iscx, 2) + pow(yyc[j] - iscy, 2);
  if (dis <= dmin)
  {
    ccm - j;
    dmin = dis;
  }
  // Set initial center closest to center of mass of input space cxx-ncx[1]*xMax/2+xo; cyy-ncy[1]*yMax/2+yo;
  Line(dc, cxx - 20, cyy, cxx + 20, cyy, red);
  Line(dc, cxx, cyy - 20, cxx, cyy + 20, red);

  for (i = 1; i < NumOfNodes; i++)
  {

  NextIteration:
    ridx = random(NumOfDataPoints);
    ncx[i] = xxc[ridx];
    ncy[i] = yyc[ridx];
    si = 20 + 1 * 20;
    cxx ncx[i + 1] * xMax / 2 + xo;
    cyy - ncy[i + 1] * yMax / 2 + yo;
    Line(dc, cxx - si, cyy, cxx + si, cyy, blue);

    // Display All Initial Centers
    Line(dc, cxx, cyy - si, cxx, cyy + si, blue);
    // Determine Cluster Membership Matrix
    // initialize all U[i][j] to 0
    // until all points have been
    // checked. Then find group (ig)with

    //-- Page 127 --------------------------------------------------------------------
    // smallest distance to point (jg) // and set that U[ig][ig]=1
    for (j = 1; j <= NumOfDataPoints; j++)
    {                                                           // Data Loop
      dmin - pow(xxc[j] - ncx[1], 2) + pow(yyc[j] - ncy[1], 2); // Set Min. Val to Start
      ig = 1;
      for (i = 1; i <= c; i++)
      { // Center's Loop
        Ui = 0;
        dd - pow(xxc[j] - ncx[i], 2) + pow(yyc[j] - ncy[i], 2);
        if (dd <= dmin)
        {
          dmin - dd;
          ig = i;
        }
      }
      U[ig][j] = 1;
    }
    // Compute Cost
    JT = 0;
    for (i = 1; i <= c; i++)
    {
    }
    J[i] = 0;
    for (k = 1; k <= NumOfDataPoints; k++)
      JT += J[i];
    if (!IN)
      JTFirst - JT;
    if (U[i][k])
      J[i] + -pow(xxc[k] - ncx[i], 2) + pow(yyc[k] - ncy[i], 2);
    if (JTFirst - 0.0 && JT > 0.0)
      JTFirst - JT;
    IN++;
    for (i = 1; i <= c; i++)
    {
      G = 0;
      xsum = ysum = 0.0;
      for (j = 1; j <= NumOfDataPoints; j++)
      {
        G += U[i][j];
        if (U[i][j])
        {
          xsum + -xxc[j];
          ysum + -yyc[j];
        }
        dc.TextOut(10, 100, "ERROR", 5);
        if (G == 0)
        {
          goto end;
        }
        ncx[i] = xsum / (double)G;
        ncy[i] = ysum / (double)G;
      }
    end:
      if (IN < 50)
        goto NextItcration;
      sprintf(txt, "IN=%i Init. Cost-%15.2f Final Cost JT=%15.2f", IN, JTFirst, JT);
      dc.TextOut(10, 30, txt, strlen(txt));
      for (i = 1; i <= c; i++) // Draw final centers as white
      {
        // De-normalize
        cxx - ncx[i] * xMax / 2 + xo;
        cyy - ncy[i] * yMax / 2 + yo;
        Line(dc, cxx - 50, cyy, cxx + 50, cyy, white);
      }
      Line(dc, cxx, cyy - 50, cxx, cyy + 50, white);

      //-- Page 128 ------------------------------------------------------------------
    }
  }
}

void TIPWindow::TrainCMACO
{
  static int i;
  static int states[NS];
  static int beta = 2;
  MessageBox("Choose Data File to Train CMAC", "CMAC", MB_OK);
  CmFileOpen();
  if (NumOfDataPoints < 1)
  {
    ("No Data File Chosen", "Error", MB_OK);
    return;
  }
  for (i = 1; i < NumOfDataPoints; i++)
  {
    states[0] = int((float)random(1000) / 1000.0 * ThetaExtreme * 2 - ThetaExtreme);
    states[1] = int((float)random(1000) / 1000.0 * DThetaExtreme * 2 - DThetaExtreme);
    train_cmac(cmac_id, states, (int)ang, beta, 40); // ang is training desired resp
                                                     // in radians
    MessageBox("Finished Training CMAC", "CMAC", MB_OK);
  }
}

 int TIPWindow::RunWeightSave()
 {
   int Choice;
   TRUNDIg* RUNDIg= new TRUNDIg(this, &RUNOptions);
   Choice-RUNDIg->Execute();
   if(ChoiceIDOK)
   {
     data_rec=1;
     if(RUNOptions.SaveWeightsToMemory)
       OtherWeights=1;
     if(RUNOptions.SaveWeightsToFile)
     {
       sprintf(SaveFileName,"%s\0",NeuralACEASEOptions. WeightFileName);
       NumOfDataPoints=steps;
       CmFileSave();
       sprintf(NeuralACEASEOptions.WeightFileName,"%s\0", SaveFileName);
       Other Weights=1;
       if(RUNOptions.DontSaveWeights)
         Other Weights=0;
     }
   }
   else
     data_rec=1;
   return Choice;
 }
 class TIPControlApp: public TApplication {
   public:


//-- Page 129 ------------------------------------------------------------------

	TIPControlApp(const char* name)
		: TApplication(name) {};
	void InitMainWindow();
	void InitInstance();
};


void TIPControlApp::InitMainWindow()
{
	EnableCtl3d();
	TIPWindow* IPWindow = new TIPWindow(0);
	IPWindow -> GetPIDOptions();
	IPWindow -> GetCalibOptions();
	IPWindow -> GetGraphics();
	MainWindow = IPWindow;
}


void TIPControlApp::InitInstance()
{
	if (hPrevInstance) {
		HWND hwnd = ::FindWindow(WINCLASSNAME, 0); if (hwnd) {
			hwnd = GetLast ActivePopup(hwnd);
			Bring WindowToTop(hwnd);
			ShowWindow(hwnd, SW_RESTORE);
			PostAppMessage(GetCurrentTask(), WM_QUIT, 0, 0);
		}
	} else
}



    TApplication::InitInstance();
#pragma argsused
    int
      OwlMain(int /*arge*/, char* /*argv*/ [])
      {

        TIPControlApp app("IPControlApp");
        return app.Run();

        // FILENAME=acease.cpp
        // Neural Adaptive Critic Element (ACE) and Associative Search Element (ASE) Bang-Bang Control
        // Algorithm:
        // Initialize Neural ACE ASE variables here



void InitializeNeuralACEASE(int IC, int OtherWeights)
{
	//-- Page 130 ------------------------------------------------------------------
	int i, j, B;
	float s0to1, s1toб, sбto12;
	float ThetaBoxSpacing = ThetaExtreme / NumThetaBoxes;
	float DThetaBoxSpacing = DThetaExtreme / NumDThetaBoxes;
	float MinDisX, MinDisY, Dx, Dy;
	static int quant[2] = {10, 10};
	IC = 1;
	tstep = 1/fs;
	NumOfNodes = NumThetaBoxes * NumDThetaBoxes;

	if(NeuralACEASEOptions.CMAC) {  // Allocate Memory for CMAC decoder
		cmac_id = 0;
		cmac_id = allocate_cmac(NS, quant, NumOfNodes, 16, 1000, RECTANGULAR, 0);
		if(lcmac_id)
      exit(-1);
		clear_cmac_weights(cmac_id);
	}

	if(!OtherWeights)
    TrialNum = 0;

	for (i = 0; i < 100; i++)
    trx[i] = 0;

	steps = 0;
	jj = 1;
	failure = 0;
	t = 0;

	//learning parameters:
	Alpha = 1000;
	Delta = 0.9;
	Beta = 0.5;
  Gamma = 0.95;
	Lamda = 0.8;

	if(!Other Weights) {
for (i=1; i <= NumOfNodes; i++) {
			wt[i] = 0;
			vt[i] = 0;
			elg0[i] = 0;
			elg[i] = 0;
			xbar[i] = 0;
			tempelg[i] = 0;
		}
		Other Weights=1;
	}
	predlast = 0;  //last prediction

	for (i=0; i < MAX_STEPS; i++) {
		ang[i] = 0;
		systime[i] = 0;
		force[i] = 0;
		V_reff[i] = 0;
	}
	// Determine Sigma (Overlap) for all centers

//-- Page 131 ---------------------------------------------------------------------

	for(i=1; i<=NumOfNodes; i++)
  {
		Sigma Theta[i] = sqrt(- (pow(ThetaBoxSpacing, 2)) / (2.0 *log(Overlap)));
		Sigma Theta[i] = sqrt(- (pow(DThetaBoxSpacing, 2)) / (2.0 *log(Overlap)));
		FailAng- 12 * Ang2Rad; //Failure Angle (by degrees)
		if(NeuralACEASEOptions.Uniform)
		{
			for(i=0; i<NumThetaBoxes; i++)
			{
				// Input space normalized to -1 and 1
				for(j=0; j<NumDThetaBoxes; j++)
				{
					// nc = node or box center
					ncx[j+1+i*NumDThetaBoxes] = -DThetaExtreme+j*DThetaBoxSpacing*2+DThetaBoxSpacing;
					-ThetaExtreme+i * ThetaBoxSpacing* 2 + ThetaBoxSpacing,
						ncy[j+1+i*NumDThetaBoxes]=
			}
}

for(i = 1; i<=NumOfNodes; i++) ISNode[i] = 0.0;

reinf = 0;
prevt = 0;

yo[0] = 0; // State 1 (Angle) theta Initial Condition
yo[1] = 0; // State 2 (Angular Velocity) theta prime

void ace()
{
	double vtsum = 0;
	int i, j;
	// ADAPTIVE CRITIC ELEMENT
	// RETURNS: internal reinforcement (internal_reinf),
	// weights for ACE(vt), and predition (pred)

	if (failure) vtsum = 0;
	else for(i = 1; i <= NumOfNodes; i++) vtsum = vtsum + vt[i] * ISNode[i];

	pred = vtsum;
	// Internal Reinforcement (for on-line adaptation)
	internal_reinf = reinf + (Gamma *pred) - predlast;
	// Update Value Function for All Nodes
	for (i=1; i <= NumOfNodes; i++)
		vt[i] = vt[i] + (Beta * internal_reinf * xbar[i]);
}

// Action Network
        // Associative Search Element

void ase()
{
	double noise;
	int ij;
//-- Page 132 ------------------------------------------------------------------
	double wtsum = 0.0, dom;
	// x=0 means zero input
	// variance = 0.01
	// to produce a probability density function value
	noise ((double) (random(700)-300))/10000;
	if (failure)
	{
	  // failure code
	}
	else
	{
		wtsum = 0.0;
	}
	for (i=1; i <= NumOfNodes; i++)
    wtsum = wtsum + ISNode[i] * wt[i];
	dom = wtsum + noise;
	if(NeuralACEASEOptions.OutSigmoid)
		action = 2 * (1/(1 + exp(-dom)) - 0.50);  // Sigmoidal Function (between +1 and 1)
	else
	{
		// Bang Bang Output
	}
	if (dom >= 0)
		action = 1.0;
	else
		action =- 1.0;
	// Ref is used as a disturbance signal
	if(NeuralACEASEOptions.DisturbanceYes) {
		V_reff[steps]=Ref(1)*10;
		action+ V_reff[steps];
	}
	//update the weights:
	for (i = 1; i <= NumOfNodes; i++)
		wt[i] = wt[i] + Alpha*internal_reinf*elg[i];
	}
void decoder()
{
	int i, j, idx;
	double Theta, DTheta, D;
	double tn, td, dn, dd, et, ed;
	double ThetaBoxSpacing-ThetaExtreme/NumThetaBoxes;
	double DThetaBoxSpacing-DThetaExtreme/NumDThetaBoxes;
	static float x[NS];
	static int TempISNode[25];
	static float xcmac[NS];
	if((TempISNode (int*)malloc(NumOfNodes))-NULL) exit(-1); //Dynamically Allocate Temp Memory
	// Decoder for states,
RETURNS: BoxNum //Input--2 state vetors from pole system: Normalized x[0]=states[0]*Rad2Ang; //angle of the pole with the vertical x[1] states[1]*Rad2Ang; //angular velocity all in degrees
	for(i=1; i <= NumOfNodes; i++) ISNode[i] = 0.0;  // Clear Boxes for New State
	if (failure) return;

//-- Page 133 ------------------------------------------------------------------

if(NeuralACEASEOptions.RBF)
	for (i = 1; i <= NumOfNodes; i++)
	{

		// 2D Gausian, pow(x,y) = x to power of y
		tn = -pow((x[0] - ncy[i]), 2);
		td = (2 * pow(SigmaTheta[i], 2));
									dn = -pow((x[1] - ncx[i]), 2);
		dd = (2 * pow(SigmaDTheta[i], 2));
		et = exp(tn / td); // Radial Basis Function for Theta (angle)
		ed = exp(dn / dd); // Radial Basis Function for DTheta (angular vel.)

		ISNode[i] = et * ed;
		else if (NeuralACEASEOptions.Uniform) {
			// Uniform Binary Grid With No Overlap (Rectangular) for(i=0;i<NumThetaBoxes;i++) {
			// Set Boxes for Extreme Negative Angular Velocity if((x[1]<(ncx[i*NumDThetaBoxes+1]+DThetaBoxSpacing)) && (x[0]=(ncy[i*NumDThetaBoxes+1]-ThetaBoxSpacing)) && (x[0]<(ncy[i*NumDThetaBoxes+1]+ThetaBoxSpacing)))
			else
				ISNode[i * NumDThetaBoxes+1] = 1.0; // Binary output
			ISNode[i * NumDThetaBoxes+1] = 0.0; // Binary Output
																					//Set Boxes Between Extremes
			for(j = 2; j < NumDThetaBoxes; j++) {
				idx-j+i NumDThetaBoxes;
				if((x[0] (ncy[idx]-ThetaBoxSpacing)) &&
						(x[0] < (ncy[idx] + ThetaBoxSpacing)) && (x[1] (ncx[idx] - DThetaBoxSpacing)) && (x[1] < (ncx[idx] + DThetaBoxSpacing)))
					ISNode[idx] = 1.0; // Binary Output
			}
			else
				ISNode[idx] = 0.0; // Binary Output
													 // Set Boxes for Extreme Positive Angular Velocity
			if((x[1] = ncx[(i+1) * NumDThetaBoxes] - DThetaBoxSpacing) && (x[0] = ncy[(i+1) * NumDThetaBoxes] - ThetaBoxSpacing) && (x[0] < ncy[(i+1) * NumDThetaBoxes] + ThetaBoxSpacing))
				ISNode[(i+1)*NumDThetaBoxes] - 1.0;
			ISNode[(i-1)*NumDThetaBoxes] - 0.0;
			else
		}
		}
		else if(NeuralACEASEOptions.CMAC) {
			for (i = 0; i <= NumOfNodes; i++)
				TempISNode[i] = 0; // convert to int
			for (i = 0; i < NS; i++)
				xcmac[i] = (int)1000 * x[i];
			// cmac_response(cmac_id,xcmac,TempISNode); // x is in degrees
			j for (i = 1; i <= NumOfNodes; i++) ISNode[i] = ((float)TempISNode[i - 1]) / 1000;

			// free(TempISNode);

//-- Page 134 ------------------------------------------------------------------

}


// SOLVE MODEL FOR SIMULATION AND/OR TRAINING: Returns x[NS]=Model States void PoleModelSolve
void PoleModelSolve()

/*
	 function sts-polemod(uf, y0, t, tstep, method)
	 cart-pole simulation function Solve ODE by using various methods:
	 method = 0 default: Euler method; method = 1: Runge-Kutta 2nd order method
	 method = 2: Runge-Kutta 4th order method
	 */
{
int i;

for(i=0; i<NS; i++) y[i]-y0[i];
PoleStateSpaceModel(s1, t, y, force[steps]); // Euler's
for(i=0; i<NS; i++) ys2[i] = y[i] + (tstep/2)*s1[i];
PoleStateSpaceModel(s2, t+tstep/2, ys2, force[steps]);

if (method == 2)
{ // Runge-Kutta 4th order
	for (i = 0; i < NS; i++)
		ys3[i] = y[i] + (tstep / 2) * s2[i];
	PoleStateSpaceModel(s3, t + tstep / 2, ys3, force[steps]);
	for (i = 0; i < NS; i++)
		ys4[i] = y[i] + tstep * s3[i];
	PoleStateSpaceModel(s4, t + tstep, ys4, force[steps]);
}

//solution:
if (method == 0) // Euler's Method
	for (i=0; i<NS; i++) y[i] = y[i] + tstep*s1[i];
else if (method == 1)
	for (i=0; i<NS; i++) y[i] = y[i] + tstep*s2[i];
else if (method == 2)
	for (i=0; i<NS; i++) y[i]=y[i]+tstep*s1[i]/6+ \
		tstep*s2[i]/3+tstep*s3[i]/3+tstep*s4[i]/6,
		for(i=0; i<NS; i++) states[i] = y[i];  // return states
}                                  


	// STATE SPACE MODEL FOR POLE SYSTEM SIMULATIONS: Returns dtdx[NS]
void PoleStateSpaceModel(double dtdx[], double t, double x[], float u)
{
	double g, l, md, mr, r, k1, k2, jm;
	double a1, a2, a3, a4;

		// Physical Constants of Inverted Pendulum System
		g = 9.8;       // m/sec, Gravity
		l = 0.49927;   // m, Length of Pole
		md = 0.26164;  // kg, Mass of Disc
		mr = 0.04240;  // kg, Mass of Pole
		r = 1.44;      // Ohms, resistance of motor windings
		k1 = 0.0833;   // Nm/amp, Proportionality const. between


//-- Page 135 ------------------------------------------------------------------

	k2 = 0.0821;    // Torque delivered and winding current  
	jm = 0.000044;  // Vsec/rad, Proportionality const between induced emf & angular velocity.

	// u is DC voltage applied to motor (input)
	// //states:
	// acceleration
	// // // kgm^2, Armature Inertia
	// xl = angle, dxldt = angular velocity-x2, dx2dt-angular
	// Il constants
	a1 = g*(md*l + mr*1/2);
	a2 = (k1*k2) / г;
	a3 = k1 / r;
	a4 = md l*1*1+ (mr * 1 *1 )/ 3 + jm;
	//equations:
	dtdx[0]=x[1];
	dtdx[1] (a1*sin(x[0]-a2*x[1]+a3*u)/a4;
			}

			// Dialog Definitions
			// *******
			// class TDataDig: public TDialog {
			// public:
			// dataparams);
			// protected:
			// private:
			// TDataDig(TWindow* parent, const char* name, TDataParamStruct&
			// void CmSync();
													// DECLARE_RESPONSE_TABLE(TDataDlg);
			// DEFINE_RESPONSE_TABLE1(TDataDlg, TDialog)
			// EV_COMMAND(IDC_SYNC, CmSync),
			// END_RESPONSE_TABLE;
			// class TFreqDlg: public TDialog {
			// };
													// public:
			// TFreqDlg(TWindow* parent, const char* name, TFrequency& freq);
													// class TPIDDIg: public TDialog {
			// public:
													// TPIDDlg(TWindow parent, PIDStruct* PIDOptions);
													//

//-- Page 136 ---------------------------------------------------------------------
                         };
    class TRUNDlg : public TDialog
                          {
      };

      public:
    TRUNDIg(TWindow * parent, RUNStruct * RUNOptions);
    class TNeuralACEASEDlg : public TDialog{} : public : protected : TNeuralACEASEDlg(TWindow * parent,
        NeuralACEASEStruct & NeuralACEASEOptions);
    void SetupWindow();
    void EvBangBangSlide(UINT code);
    void EvOverlapSlide(UINT code);
    TStatic BBMagSTxt;
    TStatic OverlapSTxt;
    NeuralACEASEStruct ACEASEOptions;
    TScrollBar BangBangSlider, TScrollBar OverlapSlider, DECLARE_RESPONSE_TABLE(TNeuralACEASED !g);
    DEFINE_RESPONSE_TABLE1(TNeuralACEASEDlg, TDialog) EvBangBangSlide), EV_CHILD_NOTIFY_ALL_CODES(IDC_BANGBANGMAG, EV_CHILD_NOTIFY_ALL_CODES(IDC_OVERLAPSLIDER, EvOverlap Slide),
    END_RESPONSE_TABLE;

class TCalibDig: public TDialog {
          }:

          public:
          private:
          protected:

          TCalibDlg(TWindow* parent, TCalibration Calibration); TStatic *SAngleTxt;
          ~TCalibDlg();
          // Destructor
          // virtual void SetupWindow();
          // TRect rect;
          int xxMax,yyMax;
          int x;
          // TCalibration "Calib,
          int CalDone; // for calibration Loop
          void CmCalibOk();
          void CmCalibCancel();
                              // void CmCalibZeroize();
          void EvTimer(UINT timerld);
          DECLARE_RESPONSE_TABLE(TCalibDlg);
          DEFINE_RESPONSE_TABLE1(TCalibDlg, TDialog) EV_COMMAND(IDOKCALIB, CmCalibOk),


//-- Page 137 ---------------------------------------------------------------------

          EV_COMMAND(IDCANCEL CALIB, CmCalibCancel).EV_COMMAND(IDC_ZEROIZE, CmCalibZeroize), EV_WM_TIMER, END_RESPONSE_TABLE;
          class TGraphicsDig: public TDialog {
          };
          public:
          TGraphicsDlg(TWindow* parent, TGraphics* Graphics);
          class TSin WavRefDlg: public TDialog {
            public:
          };
          TSinWavRefDlg(TWindow* parent, const char* name, TSinWavRefParam& SinWavRefParam);
          class TBeginControlDlg: public TDialog {
            public:
                              };
          TBeginControlDlg(TWindow* parent, TResId resId) : TDialog(parent,resId) {}

          // Dialog Constructors
          ///#
          TDataDig::TDataDig(TWindow* parent, const char* name,
              }
              TDataParamStruct& dataparams)
            : TDialog(parcnt, DATADIALOG), TWindow(parent)
          new TEdit(this, DATAFILENAME, sizeof(dataparams.DataFileName));
          new TEdit(this, NUMBEROFDATAPOINTS, sizeof(dataparams.NumOutPoints));
          new TEdit(this, NUMSKIrDATAPOINTS, sizeof(dataparams.NumSkipPoints));
          new TRadioButton(this, IDC_SYNC);
          new TRadioButton(this, IDC_LIFETIMES);
          new TRadioButton(this, IDC_STATES);
          dataparams.CollectSync = FALSE;
          TransferBuffer = (void far * )&dataparams;
          TPIDDIg::TPIDDIg(TWindow* parent, PIDStruct* PIDOptions)
          {
          }
          :TDialog(parent, PIDDIALOG)
          new TRadioButton(this, IDC_SINGLELOOP);
          new TRadioButton(this, IDC_DUALLOOP);
          SetTransferBuffer(PIDOptions);
          TRUNDig::TRUNDlg(TWindow* parent, RUNStruct* RUNOptions)
            ///01~


 //-- Page 138 --------------------------------------------------------------------


            : TDialog(parent, DIASIMBREAK)
            new TRadioButton(this, IDC_KEEPSIMGOING); 
          new TRadioButton(this,IDC_SAVEWEIGHTSMEM);
          new TRadioButton(this, IDC_SAVEWEIGHTSFILE); 
                              new TRadioButton(this, IDC_DONTSAVEWEIGHTS);
          SetTransferBuffer(RUNOptions);
          // For Setup
          TNeuralACEASEDlg::TNeuralACEASEDlg(TWindow* parent,
              NeuralACEASEStruct& NeuralACEASEOptions): TDialog(parent,
                NEURALACEASEDLG)
      {
        char txt[10];
        // note: The order of the new statements must be kept for proper operation
                            // and their size in memory must be the same as transferbuffer
        new TRadioButton(this, IDC_ZEROIZEWEIGHTS);
        new TRadioButton(this, IDC_USESIMULATIONWEIGHTS);
        new TRadioButton(this, IDC_WEIGHTSFROMFILE);
        new TEdit(this, IDC_WEIGHTFILENAME, sizeof(NeuralACEASEOptions.WeightFileName));
        new TRadioButton(this, IDC_UNIFORM);
        new TRadioButton(this, IDC_NONUNIFORM);
        new TRadioButton(this, IDC_CMAC);
        new TEdit(this, IDC_NUMTHETABOXES, sizeof(NeuralACEASEOptions.NumThetaBoxes));
        new TEdit(this, IDC_NUMDTHETABOXES, sizeof(NeuralACEASEOptions.NumDThetaBoxes));
        new TEdit(this, IDC_THETAEXTREME, sizeof(NeuralACEASEOptions.ThetaExtreme));
        new TEdit(this, IDC_DTHETAEXTREME, sizeof(NeuralACEASEOption.DThctaExtrcmc));
        new TEdit(this, IDC_ALPHA,sizeof(NeuralACEASEOptions.Alpha));
        new TRadioButton(this, IDC_SIGMOIDALOUT);
        new TRadioButton(this, IDC_BANGBANGOUT);
        new TRadioButton(this, IDC_DISTURBANCEYES);
        new TRadioButton(this, IDC_DISTURBANCENO);
        new TCheckBox(this, IDC_RBF),

        BangBangSlider = new TScrollBar(this, IDC_BANGBANGMAG);
        BBMagSTxt = new TStatic(this, IDC_VOLTS, 10);
        OverlapSlider = new TScrollBar(this, IDC_OVERLAPSLIDER);
        OverlapSTxt = new TStatic(this, IDC_OVERLAPVALUE, 10);
        TransferBuffer = (void far*)&NeuralACEASEOptions; //Set TransferBuffer(NeuralACEASEOptions); //ACEASEOptions = NeuralACEASEOptions;
      }



void TNeuralACEASEDlg::SetupWindow()
{
//-- Page 139 ------------------------------------------------------------------
  TWindow::SetupWindow();
  char txt[10] = "";
  BangBangSlider -> SetRange(1, 600);
  OverlapSlider -> SetRange(1, 100);
  // Initial Thumb Position
  BangBangSlider -> SetPosition((int)(BangBangGain* 10));
  OverlapSlider -> SetPosition(100*Overlap);
  sprintf(txt,"%6.2f", BangBangGain),
  BBMagSTxt -> SetText(txt);
  sprintf(txt,"%6.2f", Overlap);
  OverlapSTxt -> SetText(txt);
}


void TNeuralACEASEDig::EvBangBang Slide(UINT)
{
  char txt[10]="";
  BangBangGain = ((float)BangBang Slider -> GetPosition())/10;
  sprintf(txt,"%6.2f", BangBangGain);
  BBMagSTxt -> SetText(txt);
}


void TNeuralACEASEDlg::EvOverlapSlide(UINT)
{
  char txt[10]="";
  Overlap ((float)(OverlapSlider->GetPosition())/100);
  sprintf(txt,"%6.2f",Overlap);
  OverlapSTxt->SetText(txt);
  TCalibDlg::TCalibDlg(TWindow* parent, TCalibration* Calibration)
}
   :TDialog(parent, CALIBRATIONDIALOG)
   {
     new TRadioButton(this,IDC_DIRECTCONTROL);
     new TRadioButton(this,IDC_CENTERPOSITION);
     new TRadioButton(this, IDC_ENDPOSITION);
     new TControl(this, IDC_ZEROIZE);
     SAngleTxt = new TStatic(this,IDC_ENCODER);
     Calib Calibration; SetTransferBuffer(Calib);
     CalDone=0;
   }    
//Destructor




//-- Page 140 ---------------------------------------------------------------------


TCalibDlg::~TCalibDlg()
{
  KillTimer(TIMER_ID);
}



void TCalibDig::SetupWindow()
{
  TWindow::SetupWindow();
  SetTimer(TIMER_ID, 10);
}



void TCalibDlg::EvTimer(UINT /*timerId*/)
{
  char Degrees[10];
  char txt[30] = "";
  ClientDC dc(*this);
  printf(Degrees,"%f", Rad2 Ang Digital_Input(board.cal_jmp, 0));
  AngleTxt -> SetText(Degrees);
}



void TCalibDlg::CmCalibok()
{
  char txt[30];
  TransferData(tdGetData); // From Controls to Transfer Buffer
  if (Calib -> DirectlyDriven)
  {
    wsprintf(txt, "x%i", x);
    encoder_constant = 2 * pi / 1024.0, max_dif = 3.5;
    gear_type = 1;
  else if (Calib -> CenterPosition)
  {
    encoder_constant = (2*pi)/(1024.0*3.75);
    max_dif-0.6702;
    gear_type=2;
  else if (Calib -> EndPosition)
  {
    encoder_constant(2* pi)/(1024.0 14.05);
    max_dif - 0.1745;
    gear_type = 3;
  }
  KillTimer(1);
  Destroy(0);
  }


void TCalibDlg::CmCalibCancel()
{
  KillTimer(1);


//-- Page 141 ------------------------------------------------------------------

Destroy(0);
void TCalibDlg::CmCalibZeroize()
calibrate-current_measurel;
TGraphicsDlg::TGraphicsDlg(TWindow" parent, TGraphics Graphics)
:TDialog(parent, GRAPHICSDIALOG)
new TRadioButton(this, IDC_GRAPHICSON);
new TRadioButton(this, IDC_GRAPHICSOFF);
new TEdit(this, IDC_PIXELSVOLT, 10);
new TEdit(this, IDC_PIXELSDEGREE, 10);
SetTransferBuffer(Graphics)

void TDataDlg::CmSync
//dataparams CollectSync-dataparams CollectSync
if(!dataparams.CollectSync) return;
string s "Data Collected in Sync??";
MessageBeep(0);
MessageBox(s.c_str(), "Sync", MB_OK);
TFreqDlg::TFreqDlg(TWindow* parent, const char* name, TFrequency& freq)
:TDialog(parent, name), TWindow(parent)
new TEdit(this, IDC_FREQUENCY, sizeof(freq));
TransferBuffer = (void far*)&freq;
TSinWavRefDig: TSinWayRefDig(TWindow* parent, const char* name,
TSinWavRefParam& SinWayRefParam)
:TDialog(parent, DIASINEREF), TWindow(parent)
new TEdit(this, IDC_SINE_REF_AMP sizeof(SaWayRefParam.Amp));
new TEdit(this, IDC_SINE_REF_FREQ sizeof(SaWayRefParam.Freq));
TransferBuffer = (void for*)&S


