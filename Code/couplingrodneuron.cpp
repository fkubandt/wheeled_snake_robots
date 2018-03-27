#include "couplingrodneuron.h"
#include <assert.h>

#define DO_EXPAND(VAL)  VAL ## 1
#define EXPAND(VAL)     DO_EXPAND(VAL)

using namespace std;
using namespace matrix;


/** Constructor */
CouplingRod::CouplingRod(const std::string& name, const lpzrobots::OdeConfig& odeconfig, double changek, std::string changeCar, int mode, double friction)
  : AbstractController(name, "1.0"), odeconfig(odeconfig), changek(changek), changeCar(changeCar), mode(mode), friction(friction) { }


/** initialize function of controller to set all parameters
 * and add plot options as well as possibility to change parameters
 * from the console */
void CouplingRod::init(int sensornumber, int motornumber, RandGen* randGen)
{
  nSensors = sensornumber;
  nMotors  = motornumber;

  time = 0;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;

  noise = new WhiteNormalNoise;
  noise->init(1);


  addParameterDef("a",    &a,       1, "mode 0: slope of sigmoidal function");
  addParameterDef("b",    &b,       0, "mode 0: threshold of sigmoidal function");
  addParameterDef("k",    &k,      2, "coupling rod: spring constant"); 
  addParameterDef("mode", &mode,    mode, " sigmoidal (0) or constant force (2) ");
  addParameterDef("A",    &A,     1., "mode 1: amplitude of sinus target");
  addParameterDef("f",    &frequ,   3, "mode 1; rotational frequency in [1/s] of the driving force");



  N.resize( nMotors );
  
  cout << "########" << endl << "Controller internal variables: " << endl;
  cout << "nNeurons =  nMotors: " << nMotors << "   and nSensors: "<< nSensors << "   and stepsize: " << stepSize << endl;

  for( int i=0; i<nMotors; i++) {
    N[i].x=0.;
    N[i].y=0.;
    N[i].k = k*(1+handleChanges(i));
      /** Parameter Gamma is the same for all neurons */
    if(i==0) 
      addParameterDef("Gamma", &N[i].gamma, 20, "mode 0; decay constant of membrane potential");
    addInspectableValue("n"+itos(i)+":x_act", &N[i].x_act, "actual position of left x between [-1,1]");
    addInspectableValue("n"+itos(i)+":x_tar", &N[i].x_tar, "target position of left x between [-1,1]");
    addInspectableValue("n"+itos(i)+":x",     &N[i].x,     "mode 0: membrane potential");
    addInspectableValue("n"+itos(i)+":k", &N[i].k,  "motor power");
  }
}



void CouplingRod::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  /** calculation of dt and adaptation if something was changed online in odeconfig*/
  //static int counter=0;  //counter to add Inspectable value only in first step
  double old_stepSize = stepSize;
  stepSize = odeconfig.simStepSize*odeconfig.controlInterval;
  if( old_stepSize!=stepSize ) cout << "Controller stepsize = " << stepSize << endl;
  /** generating motor values using coupling rod mechanism */
  /** loop over all wheels calculate x_tar and set motor value */
  for( int i=0; i<nMotors; i++)
  {
    N[i].x_act  = cos(sensors[i]+M_PI/2.);
    if(mode==0) { /** adaptive torque */
      N[i].gamma  = N[0].gamma; 
      N[i].x     += N[i].gamma *( N[i].x_act - N[i].x ) *stepSize;
      N[i].x_tar = tanh(a*N[i].x);
    }
    if(mode==1) { /** force angular velocities */
      double frequs [10] = {40.85339643,  43.96589587,  31.85906464,  33.,
        35.09985755,  40.45985665,  0,  51.18593557,
        48.66210024,  0};
      N[i].x_tar  =   A*cos(frequs[i]*time);      
    }
    /** F_tan (multiply with R in wheeledrobot.cpp) */
    motors[i]  = couplingRod( sensors, i )-friction*sensors[i+motornumber]; 
  }
  time += stepSize;
}


double CouplingRod::handleChanges(int i){
  /** adapt individual motor strength of each controller 
    * creates a string to compare to the string given in  main.cpp */
  double changeValue = 0.;
  int nCar = i/2+1;
  std::string thisCar = "";
  if(i%2){
    thisCar = "rk";   
  }
  else{
    thisCar = "lk";   
  }
  thisCar += std::to_string(nCar); 

  if(changeCar.find("knoise")!= std::string::npos){
    changeValue = noise->generate()*changek/100.; 
  }
  else if(changeCar.find(thisCar)!= std::string::npos){
    changeValue = changek/100.;
  }
  return changeValue;
}

double CouplingRod::couplingRod(const sensor* sensors, int i){ 
  /** calculate motor force */
  double phi = sensors[i]+M_PI/2.;
  double F = 0;
  if(mode == 1){
    F = N[i].k* sin(phi) *( N[i].x_tar - N[i].x_act );
    //std::cout << "this mode is not activated, Force will be set 0"<< std::endl;
    //F =0;
  }
  else if(mode == 2){
    #if !defined(INCLUDE_FEATURE_param) || (EXPAND(INCLUDE_FEATURE_param) == 1)
        F = N[i].k*0.162; //calculated from mean angular velocity for mu=0.5
      #else
        F = N[i].k*INCLUDE_FEATURE_force; 
      #endif
  }
  else{
    F = N[i].k * sin(phi) * (N[i].x_tar - N[i].x_act);
  }
  return F;
}


/************************** NOT USED ******************/

void CouplingRod::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) { }


int CouplingRod::getSensorNumber() const {
  return nSensors;
}


int CouplingRod::getMotorNumber() const {
  return nMotors;
}


bool CouplingRod::store(FILE* f) const {
  //  S.store(f); // if S is a matrix::Matrix
  Configurable::print(f,0);
  return true;
}


bool CouplingRod::restore(FILE* f) {
  //  S.restore(f); // if S is a matrix::Matrix
  Configurable::parse(f);
  return true;
}
