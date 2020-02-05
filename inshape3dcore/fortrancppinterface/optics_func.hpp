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
  O = new Form*[1];          // Da stehen die Objekte drin (hier nur eins)
  L = new LightSrc*[1];  // Da stehen die Lichtquellen drin

  O[0] = new FormEllipsoid(zero, Vector<double>(R, R, R), n, r0); // Objekt (Ellipsoid) erstellen

  L[0] = new LightSrcGauss(StartPos, N, wvl, w0, focuspos, 100.0); // Lichtquelle erstellen (Gausstrahl)
  L[0]->setN0(1.33); // Brechungsindex Umgebungsmedium setzen
  L[0]->ObjectList(1, O); // Objektliste an Lichtquelle uebergeben
  // Leistung
  L[0]->P0 = 0.04; // Objektliste an Lichtquelle uebergeben
  ((LightSrcGauss*)L[0])->setNA(1.2); // numerische Apertur des Strahls �ndern (=sin (Oeffnungswinkel))
//  ((LightSrcGauss*)L[0])->setNA(0.99); // numerische Apertur des Strahls �ndern (=sin (Oeffnungswinkel))
//  cout << "w0=" <<    ((LightSrcGauss*)L[0])->w0 << endl; // Durchmesser im Fokus hat sich ge�ndert
//  cout << "P0=" <<    ((LightSrcGauss*)L[0])->P0 << endl; // Durchmesser im Fokus hat sich ge�ndert

  double rho = 1.18e-15; 
  double vol = L[0]->Ein[0]->Volume(); 
  double m = vol * rho;
  Matrix<double> I = (computeInertia(L[0]->getObject(0)) * m);
  if(myWorld.parInfo_.getId() == 1)
  {

    std::cout << "====================" << std::endl;
    std::cout << "    Object-Prop     " << std::endl;
    std::cout << "====================" << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > m[kg]: " << m  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > m[microgram]: " <<  1.0/myWorld.rigidBodies_[0]->invMass_  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
                " > vol[micrometer^3]: " << vol  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
                " > vol[micrometer^3]: " << myWorld.rigidBodies_[0]->volume_  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > Inertia Tensor[kg * m^2]: " << std::endl << I << termcolor::reset << std::endl;;

    std::cout << termcolor::bold << termcolor::red << myWorld.parInfo_.getId() <<  
                " > Inertia Tensor[mm^2 * microgram]: " << std::endl << myWorld.rigidBodies_[0]->invInertiaTensor_ << termcolor::reset << std::endl;;

  }
}

void init_optical_tweezers_xml()
{

  int nLS, nObj;
  loadXML("rotor.xml",nLS,L, nObj,O);

  L[0]->P0 = 0.3; 
  double power = L[0]->P0; 
  double rho = 1.0e-15; 
  double vol = L[0]->Ein[0]->Volume(); 
  double m = vol * rho;
  Matrix<double> I = (computeInertia(L[0]->getObject(0)) * m);
  if(myWorld.parInfo_.getId() == 1)
  {

    std::cout << "====================" << std::endl;
    std::cout << "    Object-Prop     " << std::endl;
    std::cout << "====================" << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > m[kg]: " << m  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > m[microgram]: " <<  1.0/myWorld.rigidBodies_[0]->invMass_  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
                " > vol[micrometer^3]: " << vol  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
                " > vol[mm^3]: " << myWorld.rigidBodies_[0]->volume_  << termcolor::reset << std::endl;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > Inertia Tensor[kg * m^2]: " << std::endl << I << termcolor::reset << std::endl;;

    std::cout << termcolor::bold << termcolor::red << myWorld.parInfo_.getId() <<  
                " > Inertia Tensor[mm^2 * microgram]: " << std::endl << myWorld.rigidBodies_[0]->invInertiaTensor_ << termcolor::reset << std::endl;;

    std::cout << termcolor::bold << termcolor::red << myWorld.parInfo_.getId() <<  
                " > Power: " << power  << termcolor::reset << std::endl;

  }

}

void update_configuration()
{

  for(auto &body : myWorld.rigidBodies_)
  { 
    if(body->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;

    L[0]->Ein[0]->P.data[0] = 1e3 * body->com_.x;
    L[0]->Ein[0]->P.data[1] = 1e3 * body->com_.y;
    L[0]->Ein[0]->P.data[2] = 1e3 * body->com_.z;

    Vec3 eulerAngles = body->quat_.convertToEuler();
    L[0]->Ein[0]->setMatrix(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    if(myWorld.parInfo_.getId()==1)
    {
      std::cout << myWorld.parInfo_.getId() <<  "> Loaded position OT:  " <<
                    L[0]->Ein[0]->P.data[0] << " " << L[0]->Ein[0]->P.data[1] << " " << L[0]->Ein[0]->P.data[2] <<  std::endl;

      std::cout << myWorld.parInfo_.getId() <<  "> Loaded position FC:  " <<
                    body->com_;
    }
  }
}

extern "C" void get_optic_forces()
{
  Vector<double> *F=new Vector<double> [1]; // Liste mit Kraeften (1 wegen nur einem Objekt)
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
  //Vec3 forcex(F[0].data[0],F[0].data[1],F[0].data[2]);
  Vec3 forcex(0,0,0);

  // F is force in Newton = kg * m/s^2
  // We need force in microgram * mm/s^2
  // Conversion to microgram * mm/s^2:
  forcex *= 1.00e12;

  // Get the torque vector
  //Vec3 taux(D[0].data[0],D[0].data[1],D[0].data[2]);
  Vec3 taux(0,0,D[0].data[2]);

  // T is torque in Newton * m = kg * m^2/s^2
  // We need torque in microgram * mm^2/s^2
  // Conversion to microgram * mm^2/s^2:
//  Vec3 taux(0,0,0);
  taux *= 1.00e15;
  //taux *= 1.00e9;
//  taux.z = 0.516;
//  taux.z = 0.129;
//  taux.z = 0.129;

  body->oldLaserForce_ = body->laserForce_;
  body->oldLaserTorque_ = body->laserTorque_;

  body->laserForce_ = forcex;
  body->laserTorque_ = taux;

//  body->laserForce_ = forcex;
//  body->laserTorque_ = taux;

  myPipeline.integrator_->applyExternalForce(body, forcex, taux);

  if(myWorld.parInfo_.getId()==1)
  {

    std::cout << "====================" << std::endl;
    std::cout << "    Laser-Force     " << std::endl;
    std::cout << "====================" << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > laser Force[microgram*mm/s^2]: " <<  body->laserForce_ << termcolor::reset;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > laser torque[kg*m^2/s^2]: " <<  Vec3(0,0,D[0].data[2]) << termcolor::reset;
    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > laser torque[mug*mm^2/s^2]: " <<  body->laserTorque_ << termcolor::reset;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > laser update: " <<  body->laserUpdate << termcolor::reset;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > fluid update: " <<  body->fluidUpdate << termcolor::reset;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > Angvelocity: " <<  body->getAngVel() << termcolor::reset;

  }

  delete[] F;
  delete[] D;
}

#endif

void init_external_ot() {

#ifdef OPTIC_FORCES
  init_optical_tweezers_xml();
  //init_optical_tweezers();
#endif 

  if(myParameters.startType_ == 1)
  {
    Reader myReader;

    if(myWorld.parInfo_.getId()==1)
    {
      std::string folder("_sol_rb");

      std::ostringstream nameRigidBodies;
      nameRigidBodies << folder << "/" << solIdx << "/rb.dmp";

      std::string n(nameRigidBodies.str());

      std::cout << "Loading dmp file: " << n << std::endl;
    }
    
    std::vector<BodyStorage> dumpSol = myReader.read_sol_rb(solIdx);

    for(unsigned i(0); i < dumpSol.size(); ++i)
    {
      BodyStorage &dumpedBody = dumpSol[i]; 
      int idx = dumpedBody.id_;
      myWorld.rigidBodies_[idx]->com_ = dumpedBody.com_;
      myWorld.rigidBodies_[idx]->velocity_ = dumpedBody.velocity_;
      myWorld.rigidBodies_[idx]->setAngVel(dumpedBody.angVel_);
      myWorld.rigidBodies_[idx]->setOrientation(dumpedBody.quat_);

#ifdef OPTIC_FORCES
      L[0]->Ein[0]->P.data[0] = 1e3 * dumpedBody.com_.x;
      L[0]->Ein[0]->P.data[1] = 1e3 * dumpedBody.com_.y;
      L[0]->Ein[0]->P.data[2] = 1e3 * dumpedBody.com_.z;

      Vec3 eulerAngles = myWorld.rigidBodies_[idx]->quat_.convertToEuler();
      L[0]->Ein[0]->setMatrix(eulerAngles.x, eulerAngles.y, eulerAngles.z);

      if(myWorld.parInfo_.getId()==1)
      {
        std::cout << myWorld.parInfo_.getId() <<  "> Loaded position OT:  " <<
                     L[0]->Ein[0]->P.data[0] << " " << L[0]->Ein[0]->P.data[1] << " " << L[0]->Ein[0]->P.data[2] <<  std::endl;

        std::cout << myWorld.parInfo_.getId() <<  "> Loaded position FC:  " <<
                     dumpedBody.com_;
      }
#endif 
    }

  }

}

