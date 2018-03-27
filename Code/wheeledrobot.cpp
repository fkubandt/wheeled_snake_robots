#include "wheeledrobot.h"
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/torquesensor.h>
#include <random>

using namespace osg;
using namespace std;
namespace lpzrobots{


WheeledRob::WheeledRob(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   const lpzrobots::OdeConfig& odeconfig,
                    const WheeledRobConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf), odeconfig(odeconfig){

  noise = new WhiteNormalNoise;
  noise->init(1);
  /** stepsize */
  stepsize = odeconfig.simStepSize*odeconfig.controlInterval;

  motorNo = 2*conf.carNumber;  /** each car has two wheels */
  /************************** Numbering of sensors **********************************
   * Angle for each wheel                 ~ motorNo					                        *
   * opt. Angular velocity                ~ motorNo           if speedSensors       *
   * Angles between the cars horizontal   ~ carNumber-1                             *
   * opt. Angles vertical                 ~ carNumber-1       if no supportWheels   *
   *                                                                                *
   *            They are logged in this order as x[0] to x[sensorNo]                *
   **********************************************************************************/
  sensorNo = (conf.speedSensors ? 2:1)*motorNo + (conf.supportWheels ? 1:2)*(conf.carNumber-1);
  cout << "########" << endl << "# Motor and Sensor number of Wheeled Robot: " << endl;
  cout << "# motorNo: " << motorNo << "   and sensorNo: " << sensorNo << endl;

  /** storage for orientations of the cars, if all 0 they are on a straight line */
  carAngleV.assign(conf.carNumber-1, 0);
  carAngleH.assign(conf.carNumber-1, 0);

  /** Initial wheel orientations */
  InitWPos.assign(motorNo, -5*M_PI/6.);
  if(conf.randomInitWP) {
    random_device rd;
    mt19937 mt(rd());
    uniform_real_distribution<double> distribution(0,2*M_PI); /** half open interval [0; 2*M_PI) */
    for( int i=0; i<motorNo; i++)
    {
      InitWPos[i]= distribution(mt);
      if(InitWPos[i]<M_PI) cout << "  " << InitWPos[i]/M_PI << " pi" ;
      else cout << "  " << (InitWPos[i]-2*M_PI)/M_PI << " pi" ;
    }
    cout << endl;
  }



  addParameter("damping", &this->conf.spD1, "Damping of car joints");
  addParameter("spring", &this->conf.spC1, "Spring constant of car joints");


}


WheeledRob::~WheeledRob() {}


void WheeledRob::placeIntern(const Matrix& pose) {
  /** shifts robot up so it stands on wheels 
  */
  if(conf.changeCar == "noise"){
  assert(2.*conf.wheelRadius*(1-conf.changeCar_percent/100) > conf.bodyHeight);
  }
  else{
  assert(2*conf.wheelRadius*(1+conf.changeCar_percent/100) > conf.bodyHeight);
  }
  Matrix initialPose = pose * Matrix::translate( Vec3(0.,0., conf.wheelRadius) );
  create(initialPose);
}


int WheeledRob::getSensorsIntern( sensor* sensors, int sensornumber) {
  /** Gets the Joint values for wheel orientation and angular velocity
  * of each wheel  */
  int len=0;
  int JPC = conf.supportWheels ? 4:2; /* # Joints per car */
  for( int i=0; i<conf.carNumber; i++ )
  { /* Wheel orientation */
     sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+0])->getPosition1() + InitWPos[len];
     sensors[len] -= (sensors[len]<M_PI) ? 0 : 2.*M_PI;
     len++;
     sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+1])->getPosition1() + InitWPos[len];
     sensors[len] -= (sensors[len]<M_PI) ? 0 : 2.*M_PI;
     len++;
  }
  if( conf.speedSensors ) {
    for( int i=0; i<conf.carNumber; i++)
    { /* Wheel velocities */
       sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+0])->getPosition1Rate();
       len++;
       sensors[len] = dynamic_cast<HingeJoint*>(joints[JPC*i+1])->getPosition1Rate();
       len++;
    }
  }
  for( int i=0; i<conf.carNumber-1; i++)
  {  /** Vertical angles between cars */
     sensors[len] = carAngleV[i];
     len++;
  }
  if( !conf.supportWheels ) {
    for( int i=0; i<conf.carNumber-1; i++)
    {  /** Horizontal angles between cars */
      sensors[len] = carAngleH[i];
      len++;
    }
  }
  return len;
}



void WheeledRob::setMotorsIntern( const double* motors, int motornumber ) {
  /** calculate motors M = r*(tangential force)
      calculate spring force of the connecting joints
  */
  int JPC = conf.supportWheels ? 4:2;
  int m=0;
  for( int i=0; i<conf.carNumber; i++)
  { /* wheel motors */
    dynamic_cast<HingeJoint*>(joints[JPC*i])->addForce1( motors[m]*conf.wheelRadius );
    m++;
    dynamic_cast<HingeJoint*>(joints[JPC*i+1])->addForce1( motors[m]*conf.wheelRadius );
    m++;
  }
  /** if train: loop over all connections between cars to calc spring force */
  int n=0;
  for( int j=JPC*conf.carNumber; j<conf.carNumber*(JPC+1)-1; j++)
  { /* spring force for vertical car angles */
    double angle_new = dynamic_cast<UniversalJoint*>(joints[j])->getPosition1();
    double springForce = -conf.spC1*angle_new - conf.spD1*(angle_new-carAngleV[n])/stepsize;
    dynamic_cast<UniversalJoint*>(joints[j])->addForce1(springForce);
    carAngleV[n] = angle_new;
    n++;
  }
  if( !conf.supportWheels ) {
    int n=0;
    for( int j=JPC*conf.carNumber; j<conf.carNumber*(JPC+1)-1; j++)
    { /* spring force for horizontal car angles*/
      double angle_new = dynamic_cast<UniversalJoint*>(joints[j])->getPosition2();
      double springForce = -conf.spC1*angle_new - conf.spD1*(angle_new-carAngleH[n])/stepsize;
      dynamic_cast<UniversalJoint*>(joints[j])->addForce2(springForce);
      carAngleH[n] = angle_new;
      n++;
    }
  }
  stepsize = odeconfig.simStepSize*odeconfig.controlInterval; //update stepsize
}


void WheeledRob::setColor(int mode){
  Color color = Color(0,0,0);
  Color colorFirst = Color(0,0,0);
  if(mode == 0){
    color = Color(1.,1.2,0);
    colorFirst = Color(1.6,0.8,0);;
  }
  if(mode == 2){
    color = Color(0,1.2,1);
    colorFirst = Color(0,0.8,1.6);
  }
  if(mode == 1){
    color = Color(1.2,0,1);
    colorFirst = Color(0.8,0,1.6);
  }
  vector<Cylinder*>::iterator it;
  int i=0;
  for( it=bodies.begin(); it!=bodies.end(); ++it){
    if(conf.changeCar.find(std::to_string(i+1)) != std::string::npos){
      (*it)->setColor(Color(1,0,0));
    }
    else if(it==bodies.begin()){
      (*it)->setColor(colorFirst);
    }
    else{
      (*it)->setColor(color);
    }
    i++;
  }
}

/** Previous friction method 
void WheeledRob::velocityFriction(double friction) {
  vector<Cylinder*>::iterator it;
    for( it=bodies.begin(); it!=bodies.end(); ++it ) {
      dBodyID b = (*it)->getBody();
      const dReal* vel = dBodyGetLinearVel(b);
      dBodyAddForce( b, -vel[0]*friction,
                         -vel[1]*friction,
                         -vel[2]*friction );
      
    }

}
**/

double WheeledRob::handleChanges(std::string thisCar){
  /** adapt wheel radius for each wheel individually
  */
  double changeValue = 0.;
  if(conf.changeCar.find("wnoise")!= std::string::npos){
  changeValue = noise->generate()*conf.changeCar_percent/100.;
  }
  else if(conf.changeCar.find(thisCar) != std::string::npos){
  changeValue = conf.changeCar_percent/100.;
  }
  return changeValue;
}

void WheeledRob::create(const Matrix& pose) {
  /** Creating new space for the chain with inside collision of all elements */
  odeHandle.createNewSimpleSpace(parentspace, false);
  //odeHandle.createNewHashSpace(parentspace, false);
  OdeHandle* Space = &odeHandle;
  //spaces.resize( conf.carNumber );
  //vector<Cylinder*> bodies;        /** bodies for the cars */
  bodies.resize( conf.carNumber );
  //vector<Cylinder*> wheels;        /** wheels of the cars */
  wheels.resize( 2*conf.carNumber );
  vector<HingeJoint*> wheelJoints; /** joints between wheels and bodies */
  wheelJoints.resize( 2*conf.carNumber );
  vector<Sphere*> supWheels;        /** support wheels of the cars */
  supWheels.resize( 2*conf.carNumber );
  vector<BallJoint*> supWheelJoints; /** joints between support wheels and bodies */
  supWheelJoints.resize( 2*conf.carNumber );
  vector<UniversalJoint*> carJoints;        /** connections between cars */
  carJoints.resize( conf.carNumber-1 );

  double radius;

  /*******************
  N = parts per car    i=0 to i<conf.carNumber  CA= carNumber
  objects[N*i]         -  bodies
  objects[N*i+1]       -  left wheels
  objects[N*i+2]       -  right wheels
  evtl. objects[N*i+3] -  support wheels front
  evtl. objects[N*i+4] -  support wheels back
  joints[N*i]          -  left wheel joint
  joints[N*i+1]        -  right wheel joint
  evtl. joints[N*i+2]  -  support wheel joint front
  evtl. joints[N*i+3]  -  support wheel joint back
  joints[(2o4)*CA + i] -  joints between cars
  joints[(2o4)*CA + CA-1]
  */

  for( int i=0; i<conf.carNumber; i++) {
      /** Creating body */
      bodies[i] = new Cylinder( conf.bodyRadius, conf.bodyHeight);
      bodies[i]->setSubstance( Substance::getPlastic(0.8));
      bodies[i]->init( *Space, conf.bodyMass, osgHandle);
      Pose bodyPos = osg::Matrix::translate(0, -i*conf.carDistance*conf.bodyRadius, 0)*pose;
      bodies[i]->setPose( bodyPos );
      objects.push_back(bodies[i]);


      /** Creating Left Wheels */
      radius = conf.wheelRadius*(1 + handleChanges("lw"+std::to_string(i+1)));
      wheels[2*i] = new Cylinder( radius, conf.wheelHeight );
      wheels[2*i]->setTexture("Images/chess.rgb");
      wheels[2*i]->init( *Space, conf.wheelMass, osgHandle);
      Pose lwPos = osg::Matrix::rotate(M_PI/2.,0.,1.,0.)*
                     osg::Matrix::translate(-(conf.bodyRadius+conf.wheelHeight/2.),0.,0.)*
                     bodyPos;
      wheels[2*i]->setPose( lwPos );
      objects.push_back( wheels[2*i] );
      /** Creating Wheel Joints */
      wheelJoints[2*i] = new HingeJoint( bodies[i], wheels[2*i], wheels[2*i]->getPosition(), Axis(0,0,1)*lwPos );
      wheelJoints[2*i]->init( *Space, osgHandle, false );
      joints.push_back( wheelJoints[2*i] );

      /** Creating Right Wheels */
      radius = conf.wheelRadius*(1 + handleChanges("rw"+std::to_string(i+1)));
      wheels[2*i+1] = new Cylinder( radius, conf.wheelHeight );
      wheels[2*i+1]->setTexture("Images/chess.rgb");
      wheels[2*i+1]->init( *Space, conf.wheelMass, osgHandle );
      Pose rwPos = osg::Matrix::rotate(M_PI/2.,0.,1.,0.)*
                   osg::Matrix::translate( (conf.bodyRadius+conf.wheelHeight/2.),0.,0.)*
                     bodyPos;
      wheels[2*i+1]->setPose( rwPos );
      objects.push_back( wheels[2*i+1] );
      /** Creating Wheel Joints */
      wheelJoints[2*i+1] = new HingeJoint( bodies[i], wheels[2*i+1], wheels[2*i+1]->getPosition(), Axis(0,0,1)*rwPos );
      wheelJoints[2*i+1]->init( *Space, osgHandle, false );
      
      joints.push_back( wheelJoints[2*i+1] );



      if( conf.supportWheels ) {
         /** Creating Left Support Wheels */
         supWheels[2*i] = new Sphere( conf.supWheelRadius );
         supWheels[2*i]->init( *Space, conf.supWheelMass, osgHandle );
         Pose fwPos = osg::Matrix::translate( 0., conf.bodyRadius-2.*conf.supWheelRadius, conf.supWheelAnchor )*
                        bodyPos;
         supWheels[2*i]->setPose( fwPos );
         objects.push_back( supWheels[2*i] );
         /** Creating Joints */
         supWheelJoints[2*i] = new BallJoint( bodies[i], supWheels[2*i], supWheels[2*i]->getPosition() );
         supWheelJoints[2*i]->init( *Space, osgHandle, true, conf.supWheelRadius/2. );
         joints.push_back( supWheelJoints[2*i] );
         /** Creating Right Wheels */
         supWheels[2*i+1] = new Sphere( conf.supWheelRadius );
         supWheels[2*i+1]->init( *Space, conf.supWheelMass, osgHandle );
         Pose bwPos = osg::Matrix::translate( 0., -conf.bodyRadius+2.*conf.supWheelRadius, conf.supWheelAnchor )*
                        bodyPos;
         supWheels[2*i+1]->setPose( bwPos );
         objects.push_back( supWheels[2*i+1] );
         /** Creating Wheel Joints */
         supWheelJoints[2*i+1] = new BallJoint( bodies[i], supWheels[2*i+1], supWheels[2*i+1]->getPosition() );
         supWheelJoints[2*i+1]->init( *Space, osgHandle, true, conf.supWheelRadius/2. );
         joints.push_back( supWheelJoints[2*i+1] );
      }
  }
  /** The Universal Joints are so far best choice to connect the cars.
    * One can add seperate torques on to axes.
    * By confining the maximal angles we prevent from internal collisions
    */
  enum JType  {BallJ, Hinge2J, UniversalJ};
  JType cj = UniversalJ;
  switch (cj){
    case BallJ:
      //for( int i=0; i<conf.carNumber-1; i++) {
      //    /** Creating joints between cars */
      //    Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
      //    carJoints[i] = new BallJoint( bodies[i], bodies[i+1], jointPos );
      //    carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
      //    joints.push_back( carJoints[i] );
      //}
      break;
    case Hinge2J:
      //for( int i=0; i<conf.carNumber-1; i++) {
      //    /** Creating joints between cars */
      //    Vec3 jointPos = Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)*pose;
      //    Matrix jP = Matrix::translate( Vec3(0., -conf.carDistance/2.-i*conf.carDistance*conf.bodyRadius, 0.)) *pose;
      //    carJoints[i] = new Hinge2Joint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );
      //    carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/2.);
      //    carJoints[i]->setParam(dParamLoStop, -0.5);
      //    carJoints[i]->setParam(dParamHiStop, 0.5);
      //    joints.push_back( carJoints[i] );
      //}
      break;
    case UniversalJ:
      for( int i=0; i<conf.carNumber-1; i++) {
          /** Creating joints between cars */
          Vec3 jointPos = Vec3(0., -(i+0.5)*conf.carDistance*conf.bodyRadius, 0)*pose;
          Matrix jP = Matrix::translate( Vec3(0., -(i+0.5)*conf.carDistance*conf.bodyRadius, conf.wheelRadius))*pose;
          carJoints[i] = new UniversalJoint( bodies[i], bodies[i+1], jointPos, Axis(0,0,1)*jP, Axis(1,0,0)*jP );  //Axis relative to jP
          carJoints[i]->init( odeHandle, osgHandle, true, conf.bodyHeight/1.5);
          carJoints[i]->setParam(dParamLoStop, -M_PI/4. );
          carJoints[i]->setParam(dParamHiStop, M_PI/4. );
          joints.push_back( carJoints[i] );
      }
      break;
  }
}



}


