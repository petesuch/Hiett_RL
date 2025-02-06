// Resurrection of 1997 RL Inverted Pendulum Control Code Model imported & updated 2023

#include <iostream>
#include "RL.h"
//#include <unistd.h>



/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char** argv) {
	printf("Reinforcement Learning: Resurrected from 1997 ASU Inverted Pendulum Control Masters Thesis Control Code\n");

	if(InitOptions()) 
		printf("Initalization Successful\n");
	else
		printf("Initialization Failure.\n");
		
	if(CmCalibration())
		printf("Calibration Successful\n");
	else	
		printf("Calibration Failure\n");
		
		
    if (CmBeginControl())
    {
        printf("Completed Experiments.\n");
        if (bFailedToLearn)
            printf("Failed to Learn How to Balance.\n");
        else
            printf("Succeeded to Learn How to Balance.\n");
    }
	else
		printf("Failed to start control simulation.\n");
	
	printf("press any key to end\n");
     if (_getch() == 32) printf("You pressed space!\n");
			
	return 0;
}


// RL97 C Functions
//



bool InitOptions()
{
	printf("Starting to initialize options.\n");
    strcpy_s(MasterFileName, DEFMASTERFILENAME);
    strcpy_s(DataParamStruct.DataFileName, DEFDATAFILENAME);
    strcpy_s(DataParamStruct.NumSkipPoints, DEFNUM_SKIPPOINTS);
    DataParamStruct.CollectSync = DEF_COLLECTSYNC;
    DataParamStruct.LifeTimes = TRUE;
    DataParamStruct.States = FALSE;
    strcpy_s(Frequency.Freq, "1"); //DEF_FREQUENCY 1

    //page 105
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
    strcpy_s(NeuralACEASEOptions.WeightFileName, DEFWEIGHTFILENAME);
    NeuralACEASEOptions.Uniform = TRUE; //was true
    NeuralACEASEOptions.Nonuniform = FALSE; //was false
    NeuralACEASEOptions.CMAC = FALSE;
    NeuralACEASEOptions.RBF = TRUE; //was false
    NumOfNodes = NumThetaBoxes * NumDThetaBoxes;
    printf("Neural ACE ASE Option Settings:\n");
    sprintf_s(NeuralACEASEOptions.cNumThetaBoxes, "%i", DEF_NUM_OF_THETA_BOXES);
    printf("# Of Theta Boxes = %s\n",NeuralACEASEOptions.cNumThetaBoxes);
    sprintf_s(NeuralACEASEOptions.cNumDThetaBoxes, "%i", DEF_NUM_OF_DTHETA_BOXES);
    printf("# Of DTheta Boxes = %s\n",NeuralACEASEOptions.cNumDThetaBoxes);
    sprintf_s(NeuralACEASEOptions.cThetaExtreme, "%6.2f", ThetaExtreme);
    printf("Theta Extreme = %s\n",NeuralACEASEOptions.cThetaExtreme);
	sprintf_s(NeuralACEASEOptions.cDThetaExtreme, "%6.2f", DThetaExtreme);
	printf("DTheta Extreme = %s\n",NeuralACEASEOptions.cDThetaExtreme);
    sprintf_s(NeuralACEASEOptions.cAlpha, "%i", DEFALPHA);
    printf("Alpha = %s\n",NeuralACEASEOptions.cAlpha);
    NeuralACEASEOptions.OutSigmoid = FALSE;
    NeuralACEASEOptions.OutBangBang = TRUE;
    NeuralACEASEOptions.DisturbanceYes = FALSE;
    NeuralACEASEOptions.DisturbanceNo = TRUE;
    
    printf("\nCompleted Neural ACE ASE Option Settings\n");

    xx = cx * 5 + 1; xx2 = cx * 5 + 11; //;Initial Graphing parameters

  
    //page 106
  
    return true;
    
}


//page 107
//page 110


    //Set up maximum coordinate values

    //page 112

void CmSimulate() // ALLOW SIMULATION
{
    SIMULATE = 1;
}

void CmSetupData()
{
}

void CmFrequency()
{
   
}

void CmPID()
{
    ControlType = PID;
 
}


void CmNeuralACEASE() // Setup
{
    char ext[5] = "";
    char /* * p1, * p2, */ fileloc[200] = "";
    char filename[200] = "";
 
    ControlType = NEURAL_ACE_ASE;
    
}

bool CmCalibration()
{
    char Degrees[10] = { 0 };
    int cal_jmp, out;
    float pos2, previous_measure;

    //page 115
    graph_output = 0;
    cal_jmp = 1;
    out = 2;
    calibrate = 0.0;
    pos2 = 0.0;
    previous_measure = 0.0;

    strcpy_s(Calibration.PoleAngle, Degrees);

    return true;
}


void CmDisplay()
{
   
}

void CmFileOpen()
{
    char ext[5] = "";
    char /*p1, * p2,*/ fileloc[200] = "";
    char filename[200] = "";
 
}

void CmFileSave()
{
    char ext[5] = "";
    //char* p1, * p2, 
    char fileloc[200] = "";
    char filename[200] = "";
 
}


bool CmBeginControl(void)
{
    //int t_count=0, sync, rotate_buff, per_t_rem, ndpts=0, jj, jk, jkk, in_sync=0, IC, count, countt, start, Mag = 500; 
    int t_count=0, sync, rotate_buff, per_t_rem, ndpts=0, jj, jk, jkk, in_sync=0, count, countt, Mag = 500; 
    //char txt[30];
    static float oldx1 = 0;
    float fcount, current_measure1 = 0.0;
    float volt[1000];
    //int iLifeTime; //number of steps until failure

      bool fRetVal = TRUE;
 
    jj = 1;
    data_rec = 0;
    IC = 0;
    rotate_buff = 0;
    graph_error = 0;
    jk = 0;
    jkk = 1;

    // Set frequency count so that proper system frequency is attained. 
    fcount = (float)(2000000.0 / fs);
    fcount = floor(fcount);
    count = (unsigned int)fcount;

    // Automatic Calibration (assume pole positoin is initially at zero degrees)
    //Digital_Input(board, cal_jmp, IC); //updates current_measure1
    calibrate = current_measure1;
    // Start loop 
//next_sample:
    delay++; if (delay > 99999) delay = 0;

    // Load counter B so that proper frequency is attained.  Determine
    // the reference command, input the encoder positoin (digital_input),
    // perform negative feedback to determine the
    //
    //page 130
    //
    //input to the compensator (comp_in) and call
    //Compensator to determine the output from
    //the compensator (input to plant.) *** */

    switch (ControlType)
    {
    case PID:
        //volt[jj] = PIDController(IC, jj);
        break;
    case NEURAL_ACE_ASE:
        volt[jj] = NeuralACEASE(IC, jj, states); // 10.59 = DC Motor Amp Gain
        break;
    case FUZZY:
        //Not implemented yet
        break;
    }

    // If graph_output is 1 display system variables (Graph), if graph_error is
    //1 end the program, if graph_output and IC are 0 display "message" to begin control of
    //the pendulum. 

   
    //Check if "keyboard" has been hit.  If
    //data_rec is negative put 0 volts to analog channel 0 and go to the GUI.
    //If data_rec is positive increment loop
    //counter and set "rec"ord to 1.  *** */
    IC = 1;

    //page 121
    jj = jj + 1;
    // after the 15,000 (x3) data points have been collected keep running loop placing data in last data points. 
   if (jj == 15001)
       goto theend;
    // Read from analog channel 1.  If the counter has low logic keep reading analog channel 1
    // until logic is high.  When high, go to the beginning of the main loop.  

    if (jk == ndpts - 1)
    {
        if (in_sync == 1)
        {
            sync = 1;
        }
        else
        {
            sync = 0;
        }
        jk = -1;
        jkk = 1;
    }
    jk = jk + 1;

//new_count: // Set system timing (delay) in Sync with NIDAQ board clock.

    //left out nidaqenable code this line
    countt = 1;
    if (jk == 5 * jkk)
    {
        //left another line for hardware, since I do not have the board
        per_t_rem = (int)(100 * (1.0 - ((double)count - (double)t_count)) / (double)count);
        jkk = jkk + 1;
    }

 
theend: // Set Motor Voltage to Zero..Turn it off.
    //left out NIDAQ code here, since don't have

     return true;
}






//page 122


float NeuralACEASE(int IC, int jj, double states[])
{
    //Neural Control/Simulation: Returns Voltage to Motor if in Control Mode

    int i, Mag = 5, Choice;
    float tt = 0;
    //int dlg;
//    char trialtxt[80], txt[40];
//    int iLifeTime;
    char txt[100];

    if (IC==0) {
        //if (NeuralACEASEOptions.Nonuniform) ClusterInputSpace(dc);
        //if (NeuralACEASEOptions.CMAC) TrainCMAC();
        if(InitializeNeuralACEASE())
            printf("Initialization of Neural ACE ASE Successful\n");
        else
            printf("Initialization of Naural ACE ASE Failed!\n");   
    }

 
    //main loop Neural ACE ASE:
NextTrial:
    if (SIMULATE) {
        //assume zero input for ASE at the 1st time step
        xx = cx * 5 + 1; xx2 = cx * 5 + 11; //reset graph line
        tt = 0;
        ace();
        ase(); //initial action
        //Invalidate();
    }
    IC = 1;
NextStep:
    // Check for User Interrupt During Simulatoin


    //page 123
    if (SIMULATE) steps++;
    else steps = jj;
    
   //if((RunNum >= 10) && (steps >= 19))
   // 	printf("RunNum >=10 steps >=19");
    // force is voltage applied to windings (max/min +/- .2)
    force[steps] = BangBangGain * action;
    if (SIMULATE) PoleModelSolve();
    tt = tt + steps;
    systime[steps] = tt;
    for (i = 0; i < NS; i++) yZ[i] = states[i];
    ang[steps] = (float)(states[0] * Rad2Ang);
    if (fabs(ang[steps]) > MaxAngMag)
        MaxAngMag = fabs(ang[steps]);
    if (fabs(ang[steps] - ang[steps - 1]) * fs > MaxAngVelMag)
        MaxAngVelMag = fabs(ang[steps] - ang[steps - 1]) * (float)fs;
    if (abs(states[0]) > FailAng) {
        failure = 1;
        reinf = -1;
        NumRunFails[RunNum] += 1;
        iFailCount += 1;
    }
    decoder(); // Find which Box states are in
    predlast = pred;
    ace();
    ase(); //initial action
    if (failure == 0) {
        for (i = 1; i <= NumOfNodes; i++) {
            elg[i] = Delta * elg[i] + (1 - Delta) * action * ISNode[i];
            xbar[i] = Lamda * xbar[i] + (1 - Lamda) * ISNode[i];
        }
    }
 
    if (!failure && steps < (MAX_STEPS - 2) && SIMULATE) goto NextStep;

//Interrupt:  // User Interrupt or Failure Occurs then Jump to Here

    //LifeTime[RunNum][TrialNum] = steps - prevt;
  
    TrialNum++;
    if (failure) {
    	printf("\nfailure!\n");
        printf("Life Time = %i steps \n", steps - prevt);
        //If SIMULATE=1 END Trial or Failure Occurs, not time to reset.
        //Reset Variables for next trial
        for(i=0;i<NS;i++) yZ[i]=0.0;
        //Find New Node Centers Based on Clustering of Input Data
        //From all Previous Trials
        failure = 0;
        reinf = 0;
        predlast = 0;
        for (i = 1; i <= NumOfNodes; i++) {
            //page 124
            elg[i] = elg0[i];
            xbar[i] = elg0[i];
            ISNode[i] = 0.0;
        }
        prevt = steps;
        if (SIMULATE) 
		{ // Running Simulation or Training
            if (TrialNum < MAX_TRIALS && steps < (MAX_STEPS - 2))
                goto NextTrial;
            else {
                sprintf_s(txt, "steps=%i, Trials=%i, Runs=%i\n", steps, TrialNum, RunNum);
                printf(txt);
                //randomize();
                //int Pass = false;
                InitializeNeuralACEASE();     
                RunNum++;
                if (RunNum > MAX_RUNS) {
                    for (i = 0; i < MAX_RUNS; i++)
                    {
                        sprintf_s(txt,"NumRunFails[%i]=%i\n", i, NumRunFails[i]);
                        printf(txt);
                    }
                    RunNum = 0;
                    TrialNum = 0; IC = 0;
                    RunWeightSave();
                    printf("Press 'q' to quit, or any other key to keep training\n");
                    if (_getch() == 'q')
                        goto EndACEASE;
                    else
                        goto NextTrial;
                }
                TrialNum = 0;
                goto NextTrial;
            }
        }
        else { // Real-Time Control
            //left out NIDAQ code
            RunWeightSave(); // Execute Weight Save Options
            if (RUNOptions.KeepSimGoing) CmBeginControl();
        }
    }
    else { // NO FAILURE: USER INTERRUPT OR SIM END (BEYOND MAX STEPS)
        if (steps >= (MAX_STEPS - 2)) {
            if (TrialNum < MAX_TRIALS) { steps = 1; prevt = 0; goto NextTrial; }
            else {
                InitializeNeuralACEASE();
                RunNum++;
                if (RunNum > MAX_RUNS) {
                    for (i = 0; i < MAX_RUNS; i++)
                    {
                        sprintf_s(txt, "NumRunFails[%i]=%i\n", i, NumRunFails[i]);
                        printf(txt);
                    }
                    sprintf_s(txt, "FailCount=%i Total Trials=%i\n", iFailCount, RunNum * MAX_TRIALS);
                    printf(txt);

                    if (iFailCount < (RunNum * MAX_TRIALS))
                    {
                        printf("Succeeded to Learn How to Balance Inverted Pendulum\n");
                        bFailedToLearn = false;
                    }
                    else
                    {
                        printf("Failed to learn how to balance inverted pendulum, yeah, it was hard\n");
                        bFailedToLearn = true;
                    }

                    RunNum = 0;
                    TrialNum = 0; IC = 0;
                    RunWeightSave();
                    goto EndACEASE;
                }
                //page 125

                TrialNum = 0;
                goto NextTrial;
            }
        }
        else {
            if (SIMULATE) { // Running Simulation or Training
                Choice = RunWeightSave(); // Execute Weight Save Options
                if (RUNOptions.KeepSimGoing && steps < (MAX_STEPS - 2)) {
                    data_rec = 1;
                    delay = 0;
                    goto NextStep;
                }
            }
            else { // Real-Time Control
                // left out hardware codeline here
                RunWeightSave(); // Exsecute Weight Save Options
            }
        }
    }
EndACEASE:
    data_rec = -1;
    IC = 0;
    return 0.0; // Simulation Ends return arbitrary float value
}

float ThetaBoxSpacing = 2 * ThetaExtreme / NumThetaBoxes;
float DThetaBoxSpacing = 2 * DThetaExtreme / NumDThetaBoxes;


/*
void ClusterInputSpace(TDC& dc)
{
    float DDMAXAVG = 0, DD = 0;
    static int a, i, j, ig, jg, k, xOff, yOff, G, pp;
    static float Nx, Ny, JT, OldJT, JTFirst, d1, d2, Gain = 2.0;
    static double xsum, ysum;
    int ri, cc, c = NumOfNodes, xo = xMax / 2, yo = yMax / 2;// xo x offset yo y offset
    int IN, ccm;
    static long r, th, p1, p2, si;
    static float a1, a2, a3, a4, a5, iscx, iscy;
    static double nx, ny, cxx, cyy;
    static float dd, dmin, dis;
    static float avgx, avgy, PG;
    float ThetaBoxSpacing = 2 * ThetaExtreme / NumThetaBoxes;
    float DThetaBoxSpacing = 2 * DThetaExtrme / NumDThetaBoxes;
    static char txt[50];
    int ridx;
    Trect rect;
    //HPEN redPen, BluePen, WhitePen, BlackPen;
    //page 126
    //HPEN hOldPen, GreenPen;
    IN = 0;
    sprintf(txt, "xMax=%i yMax=%i ", xMax, yMax);
    //dc.TextOut(10, 10, txt, strlen(txt));
    //Line(dc, xMax / 2, 10, xMax / 2, yMax - 10, yellow); // vertical
    //Line(dc, 10, yMax / 2, xMax - 10, yMax / 2, yello); //horizontal

    //MessageBox("Choose Data File to Cluster", "CLUSTERING", MB_OK);
    //CmFileOpen();
    if (NumOfDataPoints < 1) {
        MesageBox("No Data File Chosen", "Error", MB_OK);
        return;
    }
    // find center of mass of input space
    avgx = 0; avgy = 0;
    Nx = MaxAngMag; Ny = MaxAngVelMag;
    for (j = 1; j <= NumOfDataPoints; j++) {
        // angle and angular velocity normalized data between -1.0 and 1.0
        yyc[j] = ang[j] / Nx; xxc[j] = (ang[j] - ang[j - 1]) * fs / Ny;
        avgx += xxc[j]; avgy += yyc[j]; // plot data to be clustered
        dc.Ellipse(xxc[j] * xMax / 2 + xo, yyc[j] * yMax / 2 + yo, xxc[j] * xMax / 2 + 5 + xo, yyc[j] * yMax / 2 + 5 + yo);
    }
    iscx = avgx / NumOfDataPoints;
    iscy = avgy / NumOfDataPoints;
    // find closets dat point to center of mass of input space data
    // start dmin with value of 1st data point (initial seed center)
    dmin = pow(xxc[1] - iscx, 2) + pow(yyc[1] - iscy, 2); ccm = 1;
    for (j = 1; j <= NumOfdataPoints; j++) {
        dis = pow(xxc[j] - iscx, 2) + pow(yyc[j] - iscy, 2);
        if (dis <= dmin) { ccm = j; dmin = dis; }
    }
    // Set initial center closest to center of mass of input space
    cxx = ncx[1] * xMax / 2 + xo; cyy = ncy[1] * yMax / 2 + yo;
    Line(dc, cxx - 20, cyy, cxx + 20, cyy, red);
    Line(dc, cxx, cyy - 20, cxx, cyy + 20, red);

    for (i = 1; i <= NumOfNodes; i++) {
        ridx = random(NumOfDataPoints);
        ncx[i] = xxc[ridx]; ncy[i] = yuyc[ridx];
        si = 20 + i * 20; cxx = ncx[i + 1] * xMax / 2 + xo; cyy = ncy[i + 1] * yMax / 2 + yo;
        Line(dc, cxx - si, cyy, cxx + si, cyy, blue); // Display All Inital Centers
        Line(dc, cxx, cyy - si, cxx, cyy + _si, blue);
    }

NextIteration:
    //Determine Cluster Membership Matrix
    // initialize all U[i][j] to 0
    // untill all points have been
    // checked.  Then find group (ig) with

    //page 127

    //smalest distance to point (jg)
    // and set that U[ig][jg]=1
    for (j = 1; j <= NumOfDataPoints; j++) { // Data Loop
        dmin = pow(xxc[j] - ncx[1], 2) + pow(yyc[j] - ncy[1], 2);// Set Min. Val to Start
        ig = 1;
        for (i = 1; i <= c; i++) { // Center's Loop
            U[i][j] = 0;
            dd = pow(xxc[j] - ncx[i], 2) + pow(yyc[j] - ncy[i], 2); //likely to havge i and j reversed because paper did not scan well
            if (dd <= dmin) {
                dmin = dd;
                ig = i;
            }
        }
        U[ig][j] = 1;
    }
    // Compute Cost (K-means clustering?)
    JT = 0;
    for (i = 1; i <= c; i++) {
        J[i] = -;
        for (k = 1; k <= NumOfDataPoints; k++)
            if (U[i][k])J[i] += pow(xxc[k] - ncx[i], 2) + pow(yyc[k] - ncy[i], 2);
        JT += J[i];
    }
    if (!IN) JTFirst = JT;
    if (JTFirst == 0.0 && JT > 0.0) JTFirst = JT;
    IN++;
    for (i = 1; i <= c; i++) {
        G = 0;
        xsum = ysum = 0.0;
        for (j = 1; j <= NumOfDataPoints; j++) {
            G += U[i][j];
            if (U[i][j]) { xsum += xxc[j]; ysum += yyc[j]; }
        }
        if (G == 0) {
            dc.TextOut(10, 100, "ERROR", 5);
            goto end;
        }
        ncx[i] = xsum / (double)G;
        ncy[i] = ysum / (double)G;
    }
    if (IN < 50) goto NextIteration;
end:
    sprintf(txrt, "IN=%i Init. Cost=%15.2f Final Cost JT=%15.2f", IN, JTFirst, JT);
    //dc.TextOut(10, 30, txt, strlen(txt));
    for (i = 1; i <= c; i++) // Draw final centers as white
    {   //De-normalize
        cxx = ncx[i] * xMax / 2 + xo; cyy = ncy[i] * yMax / 2 + yo;
        //Line(dc, cxx - 50, cyy, cxx + 50, cyy, white);
        //Line(dc, cxx, cyy - 50, cxx, cyy + 50, white);
    }

    // page 128
}
*/


int RunWeightSave()
{
    OtherWeights = 1;

    if (bFailedToLearn)
        printf("Failed to Learn How to Balance.\n");
    else
        printf("Succeeded to Learn How to Balance.\n");

    return 1;
    
}

//------------------------------------------------
     // page 129



   //FILENAME=acease.cpp
   // //Neural Ataptive Critic Element (ACE) and Associative Search Element (ASE)
   // Algrotihm:  Bang-Bang Control

   //Initialize Neural ACE ASE variable here

bool InitializeNeuralACEASE()
{
    // page 130
    //int i, j, B, jj;
    int i, j, jj;
    //float s0to1, s1to6, s6to12;
  
    float ThetaBoxSpacgin = ThetaExtreme / NumThetaBoxes;
    float DThetaBoxSpacing = DThetaExtreme / NumDThetaBoxes;
    //float MinDisX, MinDisY, Dx, Dy;
    static int quant[2] = { 10,10 };
    IC = 1;

    tstep = 1 / fs;

    srand((int)time(NULL));  //added 2023CE
        
    NumOfNodes = NumThetaBoxes * NumDThetaBoxes;

    if (OtherWeights == 0) TrialNum = 0;

    for (i = 0; i < 100; i++) trx[i] = 0;
    steps = 0; jj = 1;
    failure = 0;
    t = 0;

    //learning parameters:
    Alpha = 1000; Delta = 0.9;
    Beta = 0.5; Gamma = 0.95; Lamda = 0.8;

    if (OtherWeights == 0) {
        for (i = 1; i <= NumOfNodes; i++) {
            wt[i] = 0;
            vt[i] = 0;
            elg0[i] = 0;
            elg[i] = 0;
            xbar[i] = 0;
            tempelg[i] = 0;
        }
        OtherWeights = 1;
    }

    predlast = 0;  //last prediction

    for (i = 0; i < MAX_STEPS; i++) {
        ang[i] = 0;
        systime[i] = 0;
        force[i] = 0;
        //V_reff[i] = 0;
    }

    //Determine Sigma (Overlap) for all centers

    // page 131

    for (i = 1; i <= NumOfNodes; i++) {
        SigmaTheta[i] = (float)sqrt(-(pow(ThetaBoxSpacing, 2)) / (2.0 * log(Overlap)));
        SigmaDTheta[i] = (float)sqrt(-(pow(DThetaBoxSpacing, 2)) / (2.0 * log(Overlap)));
    }
    FailAng = (float)12 * (float)Ang2Rad; //Failure Angle (by degrees)
    if (NeuralACEASEOptions.Uniform)
        for (i = 0; i < NumThetaBoxes; i++) //Input space normalized to -1 and 1
            for (j = 0; j < NumDThetaBoxes; j++) { // nc = node or box center
                ncx[j + 1 + i * NumDThetaBoxes] =
                    -DThetaExtreme + j * DThetaBoxSpacing * 2 + DThetaBoxSpacing;
                ncy[j + 1 + i * NumDThetaBoxes] = -ThetaExtreme + i * ThetaBoxSpacing * 2 + ThetaBoxSpacing;
            }
    for (i = 1; i <= NumOfNodes; i++) ISNode[i] = 0.0;

    reinf = 0;
    prevt = 0;

    yZ[0] = 0; // State 1 (Angle) theta Initial Condition
    yZ[1] = 0; // State 2 (Angular VElocity) theta prime

    return true;
}

void ace()
{
    double vtsum = 0;
    //int i, j;
    int i;

    // ADAPTIVE CRITIC ELEMENT
    // RETURNS: internal reinforcement (internal_reinf),
    //          weights for ACE(vt), and prediction(pred)
    if (failure) 
        vtsum = 0;
    else
    {
        for (i = 1; i <= NumOfNodes; i++)
            vtsum = vtsum + vt[i] * ISNode[i];
    }

    if(vtsum >= 10000) 
		vtsum = 10000; //set practical saturation limit for pleasure.
    if(vtsum <= -10000) 
		vtsum = -10000; //set practical saturation limit for pain.
    pred = vtsum;
    // Internal Reinforcement (for on-line adaptation)
    internal_reinf = reinf + (Gamma * pred) - predlast;
    // Update Value Function for All Nodes
    for (i = 1; i <= NumOfNodes; i++)
        vt[i] = vt[i] + (Beta * internal_reinf * xbar[i]);
}

// Action Network
// Associative Search Element
void ase()
{
    double noise;
    //int i, j;
    int i;

    //page 132

    double wtsum = 0.0, dom;

    // x=0 means zero input
    //variance =0.01
    //to produce a probability density function value
    //noise = ((double)(random(700) - 300)) / 10000;
    noise = ((double)((int)(700 * rand() / RAND_MAX) - 300)) / 10000;
    if (failure)
        wtsum = 0.0;
    else
        for (i = 1; i <= NumOfNodes; i++) wtsum = wtsum + ISNode[i] * wt[i];
    dom = wtsum + noise;
    if (NeuralACEASEOptions.OutSigmoid)
        action = (float)2 * (float)(1 / (1 + exp(-dom)) - 0.50); // Sigmoidal Function (between +1 and 1)
    else { // Bang Bang Output
        if (dom >= 0) action = 1.0;
        else action = -1.0;
    }

    // Ref is used as a distrubance signal
    //if (NeuralACEASEOptions.DisturbanceYes) {
        //V_reff[steps] = Ref(1) * 10;
        //action += V_reff[steps];
    //action += (float)noise;
    //}
    //update the weights:
    for (i = 1; i <= NumOfNodes; i++)
        wt[i] = wt[i] + Alpha * internal_reinf * elg[i];
}

void decoder()
{
    int i, j, idx;
    //double Theat, DTheta, D;
    double tn, td, dn, dd, et, ed;
    double ThetaBoxSpacing = ThetaExtreme / NumThetaBoxes;
    double DThetaBoxSpacing = DThetaExtreme / NumDThetaBoxes;

    //if((TempISNode=(int *)malloc(NumOfNodes))==NULL) exit(-1);
    // Dynamicaly Allocate Temp Memory

    //Decoder for states, RETURNS: BoxNum
    //Input--2 state vectors from pole system: Normalized
    x[0] = states[0] * Rad2Ang; // angle of the pole with the vertical
    x[1] = states[1] * Rad2Ang; // angular velocity all in degrees
   
    for (i = 1; i <= NumOfNodes; i++) ISNode[i] = 0.0; // Clear Boxes for New State

    if (failure)return;

    //page 133

    if (NeuralACEASEOptions.RBF)
    {
        for (i = 1; i <= NumOfNodes; i++)
        {
            // 2D Guassian, pow(x,y) = x to power of y
            tn = -pow((x[0] - ncy[i]), 2);
            td = (2 * pow(SigmaTheta[i], 2));
            dn = -pow((x[1] - ncx[i]), 2);
            dd = (2 * pow(SigmaDTheta[i], 2));
            et = exp(tn / td); // Radial Basis Function for Theta (angle)
            ed = exp(dn / dd);  // Radial Basis Function for DTheta (angular vel.)
            ISNode[i] = et * ed;
        }
    }
    else if (NeuralACEASEOptions.Uniform)
    {
        // Uniform Binary Grid With No Ovelap (Rectangular)
        for (i = 0; i < NumThetaBoxes; i++)
        {
            // Set Boxes for Extreme Negative Angular Velocity
            if ((x[1] < (ncx[i * NumDThetaBoxes + 1] + DThetaBoxSpacing)) &&
                (x[0] >= (ncy[i * NumDThetaBoxes + 1] - ThetaBoxSpacing)) &&
                (x[0] < (ncy[i * NumDThetaBoxes + 1] + ThetaBoxSpacing)))
                ISNode[i * NumDThetaBoxes + 1] = 1.0; // Binary Output
            else
                ISNode[i * NumDThetaBoxes + 1] = 0.0; // Binary Output
            //Set Boxes Between Extremes
            for (j = 2; j < NumDThetaBoxes; j++)
            {
                idx = j + i * NumDThetaBoxes;
                if ((x[0] >= (ncy[idx] - ThetaBoxSpacing)) &&
                    (x[0] < (ncy[idx]+ThetaBoxSpacing)) &&
                    (x[1] >= (ncx[idx] - DThetaBoxSpacing)) &&
                    (x[1] < (ncx[idx] + DThetaBoxSpacing)))
                    ISNode[idx] = 1.0; // Binary Output
                else
                    ISNode[idx] = 0.0; // Binary Output
            }
            // Set Boxes for Extreme Positive Angular Velocity
            if ((x[1] >= ncx[(i + 1) * NumDThetaBoxes] - DThetaBoxSpacing) &&
                (x[0] >= ncy[(i + 1) * NumDThetaBoxes] - ThetaBoxSpacing) &&
                (x[0] < ncy[(i + 1) * NumDThetaBoxes] + ThetaBoxSpacing))
                ISNode[(i + 1) * NumDThetaBoxes] = 1.0;
            else
                ISNode[(i + 1) * NumDThetaBoxes] = 0.0;
        }
    }
    else if (NeuralACEASEOptions.CMAC)
    {
        for (i = 0; i <= NumOfNodes; i++) TempISNode[i] = 0; // convert to int
        for (i = 0; i < NS; i++) xcmac[i] = (float)((int)1000 * x[i]);
        //cmac_response(cmac_id,xcmac,TempISNode); // x is in degrees
        for (i = 1; i <= NumOfNodes; i++) ISNode[i] = ((float)TempISNode[i - 1]) / 1000;
        //free(TempISNode);
    }
    

}

//page 134


// SOLVE MODEL FOR SIMULATOIN AND/OR TRAINIGN: Returns x[NS]=Model States
void PoleModelSolve()
{
    //function sts=polemod(uf,yZ,t,tstep,method)
    //car-pole simulatoin functoin
    //Solve ODE by using various methods:
    // method = 0 (default) : Euler method
    // method = 1 : Runge-Kutta 2nd order method
    // method = 2 : Runge-Kutta 4th order method
    int i;

    for (i = 0; i < NS; i++) y[i] = yZ[i];
    PoleStateSpaceModel(s1, t, y, force[steps]); // Euler's
    for (i = 0; i < NS; i++) ys2[i] = y[i] + (tstep / 2) * s1[i];
    PoleStateSpaceModel(s2, t + tstep / 2, ys2, force[steps]);

    if (method == 2) { //Runge-Kutta 4th order
        for (i = 0; i < NS; i++) ys3[i] = y[i] + (tstep / 2) * s2[i];
        PoleStateSpaceModel(s3, t + tstep / 2, ys3, force[steps]);
        for (i = 0; i < NS; i++) ys4[i] = y[i] + tstep * s3[i];
        PoleStateSpaceModel(s4, t + tstep, ys3, force[steps]);
    }

    //solution:
    if (method == 0) // Euler's Method
        for (i = 0; i < NS; i++) y[i] = y[i] + tstep * s1[i];
    else if (method == 1)
        for (i = 0; i < NS; i++) y[i] = y[i] + tstep * s2[i];
    else if (method == 2)
        for (i = 0; i < NS; i++) y[i] = y[i] + tstep * s1[i] / 6 + tstep * s2[i] / 3 + tstep * s3[i] / 3 + tstep * s4[i] / 6;
    for (i = 0; i < NS; i++) states[i] = y[i]; // return states

    printf("states[0]=%6.2f deg states[1]=%6.2f deg/sec steps=%i RunNum=%i TrialNum=%i action=%6.2f pred=%6.8f\n",states[0]*Rad2Ang,states[1]*Rad2Ang,steps,RunNum,TrialNum,action,pred);


}

// STATE SPACE MODEL FOR POLE SYSTEM SIMULATIONs: Returns dtdx[NS]
void PoleStateSpaceModel(double dtdx[], double t, double pstate[], float u)
{
    double g, l, md, mr, r, k1, k2, jm;
    double a1, a2, a3, a4;

    // Physical Constants of Inverted Pendulum System
    g = 9.8; // m/sec, gravity
    l = 0.49927; // m, Length of Pole
    md = 0.26164;  // kg, Mass of Disc
    mr = .04240; //kg, Mass of Pole
    r = 1.44;  // Ohms, resistance of motor windings
    k1 = 0.0833;  // Nm/am, Proportionality const. between


    // page 135

   // winding current
    k2 = .0821;     // Vsec/rad, Proportionality const. between 
    //

    //induced emf& angular velocity.
    jm = .000044;  // kgm^2, Armature Inertia


    // us i DC voltage applied to motor (input)

    //states:

    //  x1=angle, dx1dt=angular velocity=x2, dx2dt=angular acceleration
    // constants
    a1 = g * (md * l + mr * l / 2);
    a2 = (k1 * k2) / r;
    a3 = k1 / r;
    a4 = md * l * l + (mr * l * l) / 3 + jm;
    //equations:
    dtdx[0] = pstate[1];
    dtdx[1] = (a1 * sin(pstate[0]) - a2 * pstate[1] + a3 * (double)u) / a4;
}





        // Run program: Ctrl + F5 or Debug > Start Without Debugging menu
        // Debug program: F5 or Debug > Start Debugging menu

        // Tips for Getting Started: 
        //   1. Use the Solution Explorer window to add/manage files
        //   2. Use the Team Explorer window to connect to source control
        //   3. Use the Output window to see build output and other messages
        //   4. Use the Error List window to view errors
        //   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
        //   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
