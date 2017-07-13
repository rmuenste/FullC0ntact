#include <cppinterface.h>

#ifdef OPTIC_FORCES
#include <LightSrc.h>
#include <linse.h>
#include <ellipsoid.h>
#include <surface.h>
#include <trace.h>

Form **O;
LightSrc **L;                     // Da stehen die Lichtquellen drin
int N = 301;                      // Anzahl Strahlen  
double r0 = 1000;                 // Groesse der "Weltkugel" (=Berechnungsbereich)
Vector<double> StartPos(0,0,-40); // Position der Lichtquelle
double wvl=1.0;                   // Wellenlaenge
double w0=0.01;                   // (fiktiver) Strahldurchmesser im Fokus  
Vector<double> focuspos=zero;     // Fokusposition (zero = Nullvektor)     
double R=30;                      // Radius des Objekts  
complex<double> n=1.5;            // Brechungsindex des Objekts   


void init_optical_tweezers()
{
  O = new Form*[1];          // Da stehen die Objekte drin (hier nur eins)
  L = new LightSrc*[1];  // Da stehen die Lichtquellen drin

  O[0] = new FormEllipsoid(zero, Vector<double>(R, R, R), n, r0); // Objekt (Ellipsoid) erstellen

  L[0] = new LightSrcGauss(StartPos, N, wvl, w0, focuspos, 100.0); // Lichtquelle erstellen (Gausstrahl)
  L[0]->setN0(1.33); // Brechungsindex Umgebungsmedium setzen
  L[0]->ObjectList(1, O); // Objektliste an Lichtquelle uebergeben
  ((LightSrcGauss*)L[0])->setNA(0.99); // numerische Apertur des Strahls ändern (=sin (Oeffnungswinkel))
  cout << "w0=" <<    ((LightSrcGauss*)L[0])->w0 << endl; // Durchmesser im Fokus hat sich geändert
  cout << "P0=" <<    ((LightSrcGauss*)L[0])->P0 << endl; // Durchmesser im Fokus hat sich geändert
}

extern "C" void get_optic_forces()
{
  Vector<double> *F=new Vector<double> [1]; // Liste mit Kräften (1 wegen nur einem Objekt)
  Vector<double> *D=new Vector<double> [1]; // Liste mit Drehmomenten ( " )

  RigidBody *body = myWorld.rigidBodies_[0];
  Vector<double> pos; 
  pos.data[0] = 1e3 * body->com_.x; 
  pos.data[1] = 0;
  pos.data[2] = 0;
  L[0]->reset();
  L[0]->Ein[0]->P=pos;
  if(myWorld.parInfo_.getId()==1)
  {
    std::cout << "object pos[microns]: " << L[0]->Ein[0]->P;
  }
  F[0]=zero; 
  D[0]=zero;   
  trace (1,L,F,D); // eigentliche Strahlverfolgung
  // F is force in Newton = kg * m/s^2
  // we need force in microgram * mm/s^2
  //Vec3 forcex(F[0].data[0],F[0].data[1],F[0].data[2]);
  Vec3 forcex(F[0].data[0],0,0);
  forcex *= 2.75e12;
  forcex.y = F[0].data[1];
  Vec3 taux(0,0,0);
  body->laserForce_ = forcex;

  //myPipeline.integrator_->applyForce(body, forcex, taux);

  delete[] F;
  delete[] D;
}
#endif
