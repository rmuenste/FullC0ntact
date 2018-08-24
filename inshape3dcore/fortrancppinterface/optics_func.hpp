#include <cppinterface.h>

#ifdef OPTIC_FORCES
#include <LightSrc.h>
#include <linse.h>
#include <ellipsoid.h>
#include <surface.h>
#include <raytrace.h>
#include <parsexml.h>


Form **O;
LightSrc **L;                     // Da stehen die Lichtquellen drin
int N = 301;                      // Anzahl Strahlen  
double r0 = 1000;                 // Groesse der "Weltkugel" (=Berechnungsbereich)
Vector<double> StartPos(0,0,-40); // Position der Lichtquelle
double wvl=0.532;                   // Wellenlaenge
double w0=0.01;                   // (fiktiver) Strahldurchmesser im Fokus  
Vector<double> focuspos=zero;     // Fokusposition (zero = Nullvektor)     
double R=2;                      // Radius des Objekts  
complex<double> n=1.59;            // Brechungsindex des Objekts   


void init_optical_tweezers()
{

  int nLS, nObj;
  loadXML("test.xml",nLS,L, nObj,O);

  double rho = 2.0e-15; 
  double m = L[0]->Ein[0]->Volume() * rho;
  Matrix<double> I = (computeInertia(L[0]->getObject(0)) * m);
  if(myWorld.parInfo_.getId() == 1)
  {

    std::cout << "====================" << std::endl;
    std::cout << "    Object-Prop     " << std::endl;
    std::cout << "====================" << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > m[kg]: " << m  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > Inertia Tensor[kg * m^2]: " << std::endl << I << termcolor::reset << std::endl;;

    std::cout << termcolor::bold << termcolor::red << myWorld.parInfo_.getId() <<  
                " > Inertia Tensor[mm^2 * microgram]: " << std::endl << myWorld.rigidBodies_[0]->invInertiaTensor_ << termcolor::reset << std::endl;;

  }

}

extern "C" void get_optic_forces()
{
  Vector<double> *F=new Vector<double> [1]; // Liste mit Kräften (1 wegen nur einem Objekt)
  Vector<double> *D=new Vector<double> [1]; // Liste mit Drehmomenten ( " )

  RigidBody *body = myWorld.rigidBodies_[0];
  Vector<double> pos; 
  pos.data[0] = 1e3 * body->com_.x; 
  pos.data[1] = 1e3 * body->com_.y;
  pos.data[2] = 1e3 * body->com_.z;
  L[0]->reset();
  L[0]->Ein[0]->P=pos;
  // The force
  F[0]=zero; 
  // The torque
  D[0]=zero;   
  trace (1,L,F,D); // eigentliche Strahlverfolgung
  
  // Get the force vector
  Vec3 forcex(F[0].data[0],F[0].data[1],F[0].data[2]);

  // F is force in Newton = kg * m/s^2
  // We need force in microgram * mm/s^2
  // Conversion to microgram * mm/s^2:
  forcex *= 1.00e12;

  // Get the torque vector
  Vec3 taux(D[0].data[0],D[0].data[1],D[0].data[2]);

  // T is torque in Newton * m = kg * m^2/s^2
  // We need torque in microgram * mm^2/s^2
  // Conversion to microgram * mm^2/s^2:
  taux *= 1.00e15;
  body->laserForce_ = forcex;
  body->laserTorque_ = taux;

  if(myWorld.parInfo_.getId()==1)
  {

    std::cout << "====================" << std::endl;
    std::cout << "    Laser-Force     " << std::endl;
    std::cout << "====================" << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > laser Force[microgram*mm/s^2]: " <<  body->laserForce_ << termcolor::reset;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > laser torque[microgram*mm^2/s^2]: " <<  body->laserTorque_ << termcolor::reset;

  }

  myPipeline.integrator_->applyExternalForce(body, forcex, taux);

  delete[] F;
  delete[] D;
}
#endif
