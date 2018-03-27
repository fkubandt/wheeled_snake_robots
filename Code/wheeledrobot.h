#ifndef __WHEELED_ROB
#define __WHEELED_ROB

#include <ode_robots/oderobot.h>
#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/angularmotor.h>

#include <selforg/noisegenerator.h>

#include <ode_robots/odeconfig.h>
#include <vector>

namespace lpzrobots{

/** structure to hold configuration of the robot */
typedef struct{
  int carNumber;              
  std::string changeCar;
  double changeCar_percent;     
  double carDistance;
  double bodyRadius;          
  double bodyHeight;          
  double bodyMass;            
  double wheelRadius;
  double wheelHeight;
  double wheelMass;
  bool randomInitWP;
  bool supportWheels;
  double supWheelMass;
  double supWheelRadius;
  double supWheelAnchor;
  bool speedSensors;
  double spC1;                 //spring constant of car Joints around Z-axis (yaw)
  double spC2;                 //spring constant of car Joints around pitch axis
  double spD1;                 //damping of car Joints around Z-Axis (yaw)
  double spD2;                 //damping of car Joints around pitch axis
} WheeledRobConf;



/** WheeledRob robot:
 * Each car has two independent wheels
 * The cars are connected by angle joints
 * Inherit from OdeRobot */
class WheeledRob: public OdeRobot {
public:
  WheeledRobConf conf;
  WheeledRobConf confChanges;


  /** Contrustructor */
  WheeledRob(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const lpzrobots::OdeConfig& odeconfig,
           const WheeledRobConf &conf = getDefaultConf(),
           const std::string& name = "WheeledRobot");

  /** Default configuration of the robot */
  static WheeledRobConf getDefaultConf(){
    WheeledRobConf conf;
    conf.carNumber          = 5;
    conf.changeCar          = "none";
    conf.changeCar_percent  = 0.;
    conf.carDistance        = 2.2;                  /* will later be multiplied by bodyRadius */
    conf.bodyRadius         = 0.08;
    conf.bodyHeight         = 0.06;
    conf.bodyMass           = 1.;
    conf.wheelRadius        = 0.04;
    conf.wheelHeight        = 0.01;
    conf.wheelMass          = 0.1;
    conf.randomInitWP       = false; 		            /* initWP = -5*M_PI/6. */
    conf.supportWheels      = (conf.carNumber == 1) ? false:true;
    conf.supWheelMass       = 0.00001;
    conf.supWheelRadius     = conf.wheelRadius/4.; // uses average. perhaps has to be adapted for sides?
    conf.supWheelAnchor     = -conf.wheelRadius+conf.supWheelRadius; /* y of the anchor */
    conf.spC1               = 1.;
    conf.spC2               = 1.;
    conf.spD1               = 0.005;
    conf.spD2               = 0.005;
    conf.speedSensors       = true;
    return conf;
  }

  /** Destructor */
  virtual ~WheeledRob();

  /** Place the robot in the desired pose
   * @param pose desired 4x4 pose matrix */
  virtual void placeIntern(const osg::Matrix& pose) override;

  /** Create the robot in the desired pose
   * @param pose desired 4x4 pose matrix */
  virtual void create(const osg::Matrix& pose);

  int getSensorNumberIntern(){ return sensorNo; };
  int getSensorsIntern( sensor* sensors, int sensornumber );

  virtual int getMotorNumberIntern(){ return motorNo; };
  virtual void setMotorsIntern( const double* motors, int motornumber );

  void setColor(int mode);
  void velocityFriction(double friction);
  double handleChanges(std::string which="none");


private:
  int sensorNo;
  int motorNo;
  double stepsize;
  const lpzrobots::OdeConfig& odeconfig;
  NoiseGenerator* noise;


  std::vector<bool>   carChanges; /*to make changes on specific cars*/
  std::vector<double> carAngleH;
  std::vector<double> carAngleV;
  std::vector<double> InitWPos;
  std::vector<Cylinder*> bodies; /** to apply fricion on all cars in a train */
  std::vector<Cylinder*> wheels;


  std::vector<OdeHandle> spaces;


};

}


#endif
