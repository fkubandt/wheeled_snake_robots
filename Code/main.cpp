#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>

// additional sensors
#include <ode_robots/speedsensor.h>
#include <ode_robots/relativepositionsensor.h>

// noise on sensor and motor values
#include <selforg/one2onewiring.h>

// Environment:
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/randomobstacles.h>
#include <ode_robots/terrainground.h>

#include <ode_robots/base.h>
#include <selforg/agent.h>

#include "wheeledrobot.h"
#include "couplingrodneuron.h"

#include <string>

// definitions for shellscript parameter sweeps
#define DO_EXPAND(VAL)  VAL ## 1
#define EXPAND(VAL)     DO_EXPAND(VAL)

//***************************************//
using namespace lpzrobots;
using namespace std;

class ThisSim : public Simulation
{

public:
/******************** MAIN SETTINGS *******************/

  /** define the environment */
  double friction = 0.002;      
  double noise = 0.;
  bool compare = false;               /* second car for live comparison */
  std::string env = "slopes";         /* "slopes", "wall", "boxes", "playground", "halfpipe" or "no" */
  int gravity = 1;                    /* flying vehicle for synchronization tests */
  bool randObstacles = false;
  /** define the agents*/
  int carNum = 5;                     /* if 1: support wheels will automatically be added */
  bool randomInitWP = false;
  int mode1 = 0;                      /* if not given as command line arg: mode0 = neuron, mode2 = constant torque */      
  int mode2 = 2;                      /* mode or second car for live comparison */       
  std::string changeCar = "no";       /* "none", "wnoise" for noise on all wheels, e.g. "lk5" for left k fifth Car.*/
  double changeWheel_percent = +5.;   /*enter strength of whished change or noise in percent*/
  double my_changeK_percent = 50;     /*if not given as command line arg */

  bool track_rob = true;
  Pose initPose;
  Pose initPose2;
  WheeledRobConf conf = WheeledRob::getDefaultConf();
/***************** END MAIN SETTINGS ******************/

  /** handle command line arguments ****/
  #if !defined(INCLUDE_FEATURE_param) || (EXPAND(INCLUDE_FEATURE_param) == 1)
    int mode = mode1;             
  #else
    int mode = INCLUDE_FEATURE_mode;
  #endif
  #if !defined(INCLUDE_FEATURE_param) || (EXPAND(INCLUDE_FEATURE_param) == 1)
    double changeK_percent = my_changeK_percent;
  #else
    double changeK_percent = INCLUDE_FEATURE_kvar;
  #endif
  /*************************************/

  ThisSim() {
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");
  }

  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    /******** GLOBAL SETTINGS **********/
    global.odeConfig.setParam("simstepsize", 0.001);
    global.odeConfig.setParam("controlinterval", 1);
    global.odeConfig.setParam("gravity", -9.8*gravity);
    global.odeConfig.setParam("noise", noise);
    global.odeConfig.addParameterDef("friction", &friction, friction, "parameter for velocity depending friction");
    /******* END GLOBAL SETTINGS *********/


    /************** ENVIRONMENT **********/
    if( env == "wall" ){
      OdeHandle wallHandle = odeHandle;
      wallHandle.substance.toPlastic(0.8);
      //wallHandle.substance.toRubber(25);
      //wallHandle.substance.toFoam(1);
      auto* wall1 = new Box(8,0.3,1);
      wall1->setTexture("Images/wall.jpg");
      wall1->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall1->setSubstance(  Substance::getPlastic(0.8) );
      wall1->setPosition( Pos(0,3,0) );
      auto* wall2 = new Box(10,0.3,1.5);
      wall2->setTexture("Images/wall.jpg");
      wall2->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall2->setSubstance(  Substance::getPlastic(0.8) );
      //wall2->setPose(osg::Matrix::rotate(M_PI/8,0,0,1));
      wall2->setPosition( Pos(0,-6,0) );
      initPose = osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(-1,2-4*2.2*0.08,0);
      initPose2 = osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(1,2-4*2.2*0.08,0);
      initPose= osg::Matrix::rotate(8*M_PI/8., 0,0,1)*osg::Matrix::translate(0,-1-4*2.2*0.08,0);
      setCameraHomePos(Pos(0,8.7,5), Pos(180, -25, 0 ));
      //setCameraHomePos(Pos(-7,0,3), Pos(180,-90,0));
      setCameraMode( Static );
    }
    if( env == "boxes" ){
	    initPose = osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(-1.5,2,0);
      initPose2 = osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(1.5,2,0);
	    PassiveBox* obstacle1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(0.2,0.4,0.2), 0.3);
      obstacle1->setPose(osg::Matrix::rotate(M_PI/3., 0, 0, 1)*osg::Matrix::translate(-3,-2,0));
      obstacle1->setPosition(Pos(-1.5,-4,0));
      global.obstacles.push_back(obstacle1);
      PassiveBox* obstacle2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(0.2,0.4,0.2), 0.3);
      obstacle2->setPose(osg::Matrix::rotate(M_PI/3., 0, 0, 1)*osg::Matrix::translate(3,-2,0));
      obstacle2->setPosition(Pos(1.5,-4,0));
      global.obstacles.push_back(obstacle2);
      setCameraHomePos(Pos(0,8,6), Pos(180, -40, 0));
      setCameraMode( Static );
	  }
    if(env == "hills"){
      TerrainGround* hill1 = new TerrainGround( odeHandle, osgHandle, "terrains/hills.ppm",
                                               "terrains/hills_texture.ppm", 50, 50, .75,
                                               OSGHeightField::Red);
      hill1->setPose( osg::Matrix::translate(0,0,0.2) );
      global.obstacles.push_back( hill1 );

      initPose = osg::Matrix::rotate(7*M_PI/12, 0,0,1)*osg::Matrix::translate(15,3,2);
      initPose2 = osg::Matrix::rotate(7*M_PI/12, 0,0,1)*osg::Matrix::translate(15,3,2);
      setCameraHomePos (Pos(17,-8,2.5), Pos(-2,-5,0));
      setCameraMode( TV );

	  }
    if( env == "halfpipe" )
    {
      TerrainGround* pipe = new TerrainGround( odeHandle, osgHandle, "terrains/dip128.ppm",
                                               "terrains/dip128_texture.ppm", 10, 1000, 1,
                                               OSGHeightField::Red);
      pipe->setSubstance( Substance::getPlastic(0.8));
      pipe->setPose( osg::Matrix::translate(0,0,0) );
      global.obstacles.push_back( pipe );

      initPose = osg::Matrix::translate(0,4,0.3);
      initPose2 = osg::Matrix::translate(0,2,0.25);
      //setCameraHomePos (Pos(0, -11.4115, 2),  Pos(-1, -20, 0));
      setCameraMode( Follow);
    }
    if( env == "slopes" )
    {
      double dist = 7;
      double slope=1.3;
      double hight = 5.;
      double width = dist*1.4;
      OdeHandle wallHandle = odeHandle;
      wallHandle.substance.toPlastic(0.8);

      /** create four intersecting slopes*/
      auto* wall1 = new Box(width,0.3,hight);
      wall1->setTexture("Images/wood.jpg");
      wall1->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall1->setSubstance(  Substance::getPlastic(0.8) );
      wall1->setPose( osg::Matrix::rotate(-slope,1.,0.,0.) * osg::Matrix::translate(0,dist/2.,0) );
      auto* wall2 = new Box(width,0.3,hight);
      wall2->setTexture("Images/wood.jpg");
      wall2->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall2->setSubstance(  Substance::getPlastic(0.8) );
      wall2->setPose( osg::Matrix::rotate(slope,1.,0.,0.) * osg::Matrix::translate(0,-dist/2.,0) );
      auto* wall3 = new Box(0.3,width,hight);
      wall3->setTexture("Images/wood.jpg");
      wall3->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall3->setSubstance(  Substance::getPlastic(0.8) );
      wall3->setPose( osg::Matrix::rotate(slope,0.,1.,0.) * osg::Matrix::translate(dist/2.,0.,0) );
      auto* wall4 = new Box(0.3,width,hight);
      wall4->setTexture("Images/wood.jpg");
      wall4->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall4->setSubstance(  Substance::getPlastic(0.8) );
      wall4->setPose( osg::Matrix::rotate(-slope,0.,1.,0.) * osg::Matrix::translate(-dist/2.,0.,0) );
      /** add movable box*/
      PassiveBox* obstacle1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(0.2,0.6,0.2), 0.7);
      obstacle1->setPose(osg::Matrix::rotate(M_PI/2., 0, 0, 1)*osg::Matrix::translate(-3,-2,0));
      obstacle1->setPosition(Pos(0,0.5,0));
      global.obstacles.push_back(obstacle1);
      /** add wall */
      auto* wall5 = new Box(0.3,width/2-1,1.5);
      wall5->setTexture("Images/wall.jpg");
      wall5->init( wallHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall5->setSubstance(  Substance::getPlastic(0.8) );
      wall5->setPosition( Pos(dist/2-1,1,0) );


      //initPose = osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(-1.5,-2.2*0.08*4,0);
      //initPose2 = osg::Matrix::rotate(M_PI, 0,0,1)*osg::Matrix::translate(1.5,-0.08*8,0);
      initPose = osg::Matrix::rotate(23*M_PI/16, 0,0,1)*osg::Matrix::translate(-2,-2-2.2*0.08*4,0);
      initPose2 = osg::Matrix::rotate(23*M_PI/16, 0,0,1)*osg::Matrix::translate(-0.5,-2-0.08*8,0);
      setCameraHomePos(Pos(-5.51429, -1.73605, 1),  Pos(-111.004, -7.3396, 0));
      setCameraMode( Static );
    }
    if( env == "playground" ){
      OdeHandle wallHandle = odeHandle;
      //wallHandle.substance.toPlastic(0.8);
      wallHandle.substance.toRubber(25);
      OctaPlayground* world = new OctaPlayground( wallHandle, osgHandle,
      									Pos(5,0.2,0.5), 8, false);
      world->setPose(osg::Matrix::rotate(M_PI/2,0,0,1)*osg::Matrix::translate(0,0,0) );
      global.obstacles.push_back( world );

      initPose = osg::Matrix::translate(-1.5,0,0);
      initPose2 = osg::Matrix::translate(1.5,0,0);
      setCameraHomePos(Pos(22.9623, 0.306804, 12.942),  Pos(90.5044, -31.6968, 0));
      setCameraMode( Static );
    }
    if( env == "inclined"){
      double width = 20000;
      double length = 40000;
      double height = 0.01;
      #if !defined(INCLUDE_FEATURE_param) || (EXPAND(INCLUDE_FEATURE_param) == 1)
        double slope = atan(0.02); //enter percentage of slope into atan
      #else
        double slope = atan(INCLUDE_FEATURE_slope);  
      #endif
      auto* wall1 = new Box(width,length,height);
      wall1->setTexture(TextureDescr("Images/wood.jpg", -2.,-2.));
      wall1->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
      wall1->setSubstance(  ThisSim::getGroundSubstance() );

      /** for slope!=0, the plane is moved such that the car can start motion on even ground*/
      if(slope==0){
        wall1->setPose( osg::Matrix::rotate(slope,1.,0.,0.) * osg::Matrix::translate(0,length/2.*cos(slope) -2 ,length/2. * sin(slope)));
      }
      else{
        wall1->setPose( osg::Matrix::rotate(slope,1.,0.,0.) * osg::Matrix::translate(0,length/2.*cos(slope) -1 + 5,length/2. * sin(slope) -height));
        wall1->setPose( osg::Matrix::rotate(slope,1.,0.,0.) * osg::Matrix::translate(0,length/2.*cos(slope)-5000 ,length/2. * sin(slope) -height));             
      }
      initPose=osg::Matrix::translate(0,0,0.);
      initPose = osg::Matrix::rotate(slope, 1.,0,0)*osg::Matrix::translate(0,0,(5000+carNum*conf.bodyRadius*conf.carDistance )* tan(slope));
      setCameraHomePos (Pos(-6, 0., 51),  Pos(-90,-10, 0));
      setCameraMode( Follow );
      global.odeConfig.addParameterDef("slope", &slope, slope, "slope of inclined plane");
    }
    if( env == "no" )
    {
      if(!gravity){
        /** lift the car off the ground for measurement of the uncoupled system */
        initPose = osg::Matrix::rotate(0*M_PI/2. ,0,0,1)*osg::Matrix::translate(0.5,0,2);
      }
      initPose = osg::Matrix::translate(0,0,0);
      if(gravity==0){
        initPose = osg::Matrix::translate(0,0,1);
      }
      setCameraHomePos (Pos(-2.5, -0.5, 1.5),  Pos(-89, -32, 0));
      setCameraMode( Follow );
    }
    if( randObstacles ) {
      OdeHandle obstHandle = odeHandle;
      obstHandle.substance.toRubber(25);
      RandomObstaclesConf randConf = RandomObstacles::getDefaultConf();
      randConf.pose = osg::Matrix::translate(0,0,0);
      randConf.area = Pos(4,4,0.5);
      randConf.minSize = Pos(0.001,0.001,0.001);
      randConf.maxSize = Pos(0.01, 0.01,0.04);
      randConf.minDensity = 9000;
      randConf.maxDensity = 10000;
      //randConf.area = Pos(5,5,2);
      //randConf.minSize = Pos(0.1,0.1,0.4);
      //randConf.maxSize = Pos(0.7,0.7,0.4);
      //randConf.minDensity = 2;
      //randConf.maxDensity = 5;
      RandomObstacles* RandObstacle = new RandomObstacles(obstHandle, osgHandle, randConf);
      if(randObstacles == true){  /** Generation and placing Objects */
        int num_randObs = 160;
        for (int i=0; i< num_randObs; i++){
          RandObstacle->spawn(RandomObstacles::Box, RandomObstacles::Foam);
          global.obstacles.push_back( RandObstacle );
        }
      }
    }

    /********* END ENVIRONMENT ***********/


    /********* CAR CHAIN *****************/
    conf.changeCar = changeCar;
    conf.changeCar_percent = changeWheel_percent;
    conf.carNumber     = carNum;
    conf.randomInitWP = randomInitWP;

    /** if only 1 car install support wheels */
    (conf.carNumber==1) ? conf.supportWheels=true : conf.supportWheels=false; //already in wheeledrobot.h?? part of standard conf
    auto robot = new WheeledRob(odeHandle, osgHandle, global.odeConfig, conf, "Car");
    /**Add Sensors. Attachment(-1):Main Primitive, Attachment(n) attaches to nth Primitive
    	   n = 3*k:    Body
    	   n = 3*k+1:  right Wheel
    	   n = 3*k+2:  left Wheel
    */
    robot->addSensor(std::make_shared<SpeedSensor>(1,SpeedSensor::Translational,SpeedSensor::XYZ), Attachment(-1));
    robot->addSensor(std::make_shared<RelativePositionSensor>(1,1,RelativePositionSensor::XY), Attachment(0));
    if(carNum==5){
    robot->addSensor(std::make_shared<RelativePositionSensor>(1,1,RelativePositionSensor::XY), Attachment(12));
    robot->addSensor(std::make_shared<RelativePositionSensor>(1,1,RelativePositionSensor::XY), Attachment(0));
    }
    robot->place( initPose );
    robot->setColor(mode);
    auto controller = new CouplingRod("CouplingRod", global.odeConfig, changeK_percent, changeCar, mode, friction);
    auto wiring = new One2OneWiring(new WhiteNormalNoise());
    auto agent = new OdeAgent(global);
    //auto agent = new OdeAgent( PlotOption(File) );
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);

    /** add second agent for comparison at whish */
    if(compare){
    	auto robot2 = new WheeledRob(odeHandle, osgHandle, global.odeConfig, conf, "Car2");
    	robot2->addSensor(std::make_shared<SpeedSensor>(1,SpeedSensor::Translational,SpeedSensor::XYZ), Attachment(-1));
      robot2->addSensor(std::make_shared<RelativePositionSensor>(1,1,RelativePositionSensor::XY), Attachment(0));
        if(carNum==5){
         robot2->addSensor(std::make_shared<RelativePositionSensor>(1,1,RelativePositionSensor::XY), Attachment(12));
        }
        robot2->place( initPose2 );
    	robot2->setColor(mode2);
    	auto controller2 = new CouplingRod("CouplingRod2", global.odeConfig, changeK_percent, changeCar, mode2, friction);
    	auto wiring2 = new One2OneWiring(new WhiteNormalNoise());
    	auto agent2 = new OdeAgent(global);
    	agent2->init(controller2, robot2, wiring2);
    	global.agents.push_back(agent2);
    	global.configs.push_back(agent2);
    };
    /** track options */
    TrackRobot* TrackOpt = new TrackRobot(false,false,false, track_rob);
    TrackOpt->conf.displayTraceDur = 20;
    TrackOpt->conf.displayTraceThickness = 0.01; // 0.01 or 0.
    agent->setTrackOptions( *TrackOpt );
    /*********** END CAR CHAIN **********/

  }

/** old friction method
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(!pause and friction_mode == 0) {
      OdeRobot* rob = globalData.agents[0]->getRobot();
      dynamic_cast<WheeledRob*>(rob)->velocityFriction(friction);
      if(compare){
		  OdeRobot* rob2 = globalData.agents[1]->getRobot();
      dynamic_cast<WheeledRob*>(rob2)->velocityFriction(friction);
      }
    }
  }
**/
                       

  virtual void bindingDescription(osg::ApplicationUsage & au) const { }
};




int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
