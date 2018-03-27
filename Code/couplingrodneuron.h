#ifndef __COUPLING_ROD_NEURON
#define __COUPLING_ROD_NEURON

#include <selforg/abstractcontroller.h>
#include <ode_robots/odeconfig.h>
#include <selforg/noisegenerator.h>

#include <vector>





class CouplingRod : public AbstractController{

public:

  CouplingRod(const std::string& name, const lpzrobots::OdeConfig& odeconfig,
                double changek = 0, std::string changeCar = "no", int mode=0, double friction=0.5); 

  /** initialisation of the controller with the given sensor/ motornumber
    Must be called before use. The random generator is optional.  */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0)  override;

  /** Set different k or noise in main.cpp */
  double handleChanges(int i);

  /** Calculates tangential force resulting from the spring force */
  double couplingRod(const sensor* sensors, int i);

/************************** NOT USED ******************/

  /** @return Number of sensors the controller
    was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const override;

  /** @return Number of motors the controller
    was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const override;

  /** performs one step.
    Calculates motor commands from sensor inputs.
    @param sensors sensors inputs scaled to [-1,1]
    @param sensornumber length of the sensor array
    @param motors motors outputs. MUST have enough space for motor values!
    @param motornumber length of the provided motor array */
  virtual void step(const sensor* sensors, int sensornumber,
                        motor* motors, int motornumber) override;

  /** performs one step without learning.
    @see step */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors) override;

  /** stores the object to the given file stream (binary). */
  virtual bool store(FILE* f) const override;

  /** loads the object from the given file stream (binary). */
  virtual bool restore(FILE* f) override;

/********************************************************/



protected:
  double nSensors;
  double nMotors;

private:
  const lpzrobots::OdeConfig& odeconfig;
  double time;
  double stepSize;



  /** sinus */
  double frequ;
  double A;

  /** sigmoidal */
  double a;
  double b;

  /**Changes*/
  double changek;
  double changeGamma;
  std::string changeCar;
  NoiseGenerator* noise;

    /** coupling rod */
  double k;
  double k_changeit;
  int mode;
  double friction;

  struct Neuron {
    double x;
    double gamma;
    double y;
    double x_act;
    double x_tar;
    double k;
  };
  std::vector<Neuron> N;
};

#endif // Header guard
