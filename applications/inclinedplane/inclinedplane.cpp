// /***************************************************************************
//  *   Copyright (C) 2006-2010 by Raphael Muenster   *
//  *   raphael@Cortez   *
//  *                                                                         *
//  *   This program is free software; you can redistribute it and/or modify  *
//  *   it under the terms of the GNU General Public License as published by  *
//  *   the Free Software Foundation; either version 2 of the License, or     *
//  *   (at your option) any later version.                                   *
//  *                                                                         *
//  *   This program is distributed in the hope that it will be useful,       *
//  *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
//  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
//  *   GNU General Public License for more details.                          *
//  *                                                                         *
//  *   You should have received a copy of the GNU General Public License     *
//  *   along with this program; if not, write to the                         *
//  *   Free Software Foundation, Inc.,                                       *
//  *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
//  ***************************************************************************/
// 
// #include <string>
// #include <aabb3.h>
// #include <iostream>
// #include <genericloader.h>
// #include <unstructuredgrid.h>
// #include <distops3.h>
// #include <triangulator.h>
// #include <iomanip>
// #include <sstream>
// #include <intersectorray3tri3.h>
// #include <vtkwriter.h>
// #include <world.h>
// #include <particlefactory.h>
// #include <collisionpipeline.h>
// #include <rigidbodymotionmesh.h>
// #include <mymath.h>
// #include <distancetriangle.h>
// #include <boundingvolumetree3.h>
// #include <subdivisioncreator.h>
// #include <traits.h>
// #include <boundarybox.h>
// #include <timecontrol.h>
// #include <matrixnxn.h>
// #include <vectorn.h>
// #include <linearsolvergauss.h>
// #include <reader.h>
// #include <worldparameters.h>
// #include <globalheader.h>
// #include <meshobject.h>
// #include <subdivisioncreator.h>
// #include <boundingvolumetree3.h>
// #include <hspatialhash.h>
// #include <broadphasestrategy.h>
// #include <objloader.h>
// #include <motionintegratorsi.h>
// 
// using namespace i3d;
// 
// Real a = CMath<Real>::MAXREAL;
// CUnstrGrid myGrid;
// World myWorld;
// CollisionPipeline myPipeline;
// MotionIntegratorSI myMotion;
// CSubdivisionCreator subdivider;
// CBoundaryBoxr myBoundary;
// TimeControl myTimeControl;
// WorldParameters myParameters;
// Real startTime=0.0;
// 
// int perrowx;
// int perrowy;
// int perrowz;
// 
// double xmin=-100;
// double ymin=-100;
// double zmin=-100;
// double xmax=100.0f;
// //double ymax=0.35f;
// double ymax=100.0f;
// double zmax=100.0f;
// Real radius = Real(0.05);
// int iReadGridFromFile = 0;
// int *islots=NULL;
// 
// void addboundary()
// {
//   //initialize the box shaped boundary
//   myWorld.rigidBodies_.push_back(new RigidBody());
//   RigidBody *body = myWorld.rigidBodies_.back();
//   body->affectedByGravity_ = false;
//   body->density_  = 0;
//   body->volume_   = 0;
//   body->invMass_     = 0;
//   body->angle_    = VECTOR3(0,0,0);
//   body->setAngVel(VECTOR3(0,0,0));
//   body->velocity_ = VECTOR3(0,0,0);
//   body->friction_ = 0.0;
//   body->shape_    = RigidBody::BOUNDARYBOX;
//   CBoundaryBoxr *box = new CBoundaryBoxr();
//   box->rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
//   box->CalcValues();
//   body->com_      = box->rBox.GetCenter();
//   body->shape_      = box;
//   body->invInertiaTensor_.SetZero();
//   body->restitution_ = 0.0;
//   body->setOrientation(body->angle_);
//   CRectangle3<Real> rec0(VECTOR3(0.26,-0.1505,0),VECTOR3(1,0,0),VECTOR3(0,0,-1),0.13,0.0155); //front
//   CRectangle3<Real> rec1(VECTOR3(0.26,0.4005,0),VECTOR3(1,0,0),VECTOR3(0,0,1),0.13,0.0155);//back
//   CRectangle3<Real> rec2(VECTOR3(0.26,0.125,-0.0155),VECTOR3(1,0,0),VECTOR3(0,1,0),0.13,0.2755);//bottom
//   CRectangle3<Real> rec3(VECTOR3(0.26,0.125,0.0155),VECTOR3(1,0,0),VECTOR3(0,-1,0),0.13,0.2755);//top
//   CRectangle3<Real> rec4(VECTOR3(0.39,0.125,0),VECTOR3(0,1,0),VECTOR3(0,0,-1),0.2755,0.0155);//right
//   CRectangle3<Real> rec5(VECTOR3(0.13,0.21325,0),VECTOR3(0,1,0),VECTOR3(0,0,1),0.18725,0.0155); //left1
//   CRectangle3<Real> rec6(VECTOR3(0.13,-0.08825,0.0),VECTOR3(0,1,0),VECTOR3(0,0,1),0.06225,0.0155);//left2
//   CRectangle3<Real> rec7(VECTOR3(0.065,-0.026,0.0),VECTOR3(1,0,0),VECTOR3(0,0,-1),0.065,0.0155);//front2
//   CRectangle3<Real> rec8(VECTOR3(0.065,0.026,0.0),VECTOR3(1,0,0),VECTOR3(0,0,1),0.065,0.0155);//back2
//   CRectangle3<Real> rec9(VECTOR3(0.065,0.0,0.0155),VECTOR3(1,0,0),VECTOR3(0,-1,0),0.065,0.026);//top2
//   CRectangle3<Real> rec10(VECTOR3(0.065,0.0,-0.0155),VECTOR3(1,0,0),VECTOR3(0,1,0),0.065,0.026);//bottom2
//   CRectangle3<Real> rec11(VECTOR3(0.0,0.0,0.0),VECTOR3(0,1,0),VECTOR3(0,0,1),0.026,0.0155);//left3
//   box->m_vBorders.push_back(rec0);
//   box->m_vBorders.push_back(rec1);
//   box->m_vBorders.push_back(rec2);
//   box->m_vBorders.push_back(rec3);
//   box->m_vBorders.push_back(rec4);
//   box->m_vBorders.push_back(rec5);
//   box->m_vBorders.push_back(rec6);
//   box->m_vBorders.push_back(rec7);
//   box->m_vBorders.push_back(rec8);
//   box->m_vBorders.push_back(rec9);
//   box->m_vBorders.push_back(rec10);
//   box->m_vBorders.push_back(rec11);
// }
// 
// void cleanup()
// {
//   std::vector<RigidBody*>::iterator vIter;
//   for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
//   {
//     RigidBody *body    = *vIter;
//     delete body;
//   }
//   if(islots !=NULL)
//   {
//     delete []islots;
//     islots=NULL;
//   }
// }
// 
// void initphysicalparameters()
// {
// 
//   std::vector<RigidBody*>::iterator vIter;
// 
//   for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
//   {
//     RigidBody *body    = *vIter;
//     if(!body->affectedByGravity_)
//       continue;
//     body->density_    = myParameters.defaultDensity_;
//     body->volume_     = body->shape_->Volume();
//     Real dmass          = body->density_ * body->volume_;
//     body->invMass_    = 1.0/(body->density_ * body->volume_);
//     body->angle_      = VECTOR3(0,0,0);
//     body->setAngVel(VECTOR3(0,0,0));
//     body->velocity_   = VECTOR3(0,0,0);
//     body->com_        = VECTOR3(0,0,0);
//     body->force_      = VECTOR3(0,0,0);
//     body->torque_     = VECTOR3(0,0,0);
//     body->restitution_ = 0.0;
//     body->setOrientation(body->angle_);
//     body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//     //calculate the inertia tensor
//     //Get the inertia tensor
//     body->generateInvInertiaTensor();
//   }
// 
// }
// 
// void pyramidtest()
// {
// 	ParticleFactory myFactory;
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
// 	Real drad = extends[0];
// 	Real d    = 2.0 * drad;
// 	Real distbetween = drad * 0.05;
//   Real delta = d+distbetween;
//   Real deltaz = d;
// 
//   int towerheight=9;
// 
//   int layers=18;
//   int iboxes = (layers*(layers+1))/2.0;
//   myFactory.addBoxes(myWorld.rigidBodies_,iboxes,extends);
//   
// 	//assign the physical parameters of the rigid bodies
// 	initphysicalparameters();
//   Real length = Real(layers-1) * delta + d;
//   Real ystart = (myGrid.m_vMax.y/2.0)-(length/2.0);
//   VECTOR3 pos(myGrid.m_vMax.x/2.0, ystart, extends[2]);
//   int index = 0;
//   for(int i=0;i<layers;i++)
//   {
//     pos.y=ystart+Real(i) * (drad+distbetween/2.0);
//     for(int j=i;j<layers;j++)
//     {
//       myWorld.rigidBodies_[index]->translateTo(pos);
//       pos.y+=delta;
//       index++;
//     }
//     pos.z+=deltaz;
//   }
// 
//   myFactory.addBoxes(myWorld.rigidBodies_,towerheight,extends);
//   pos = VECTOR3(myGrid.m_vMax.x/2.0+5.0*d, myGrid.m_vMax.y/2.0, extends[2]);
// 
//   for(int j=iboxes;j<myWorld.rigidBodies_.size();j++)
//   {
//     RigidBody *body    = myWorld.rigidBodies_[j];
//     if(!body->affectedByGravity_)
//       continue;
//     body->density_    = myParameters.defaultDensity_;
//     body->volume_     = body->shape_->Volume();
//     Real dmass          = body->density_ * body->volume_;
//     body->invMass_    = 1.0/(body->density_ * body->volume_);
//     body->angle_      = VECTOR3(0,0,0);
//     body->setAngVel(VECTOR3(0,0,0));
//     body->velocity_   = VECTOR3(0,0,0);
//     body->com_        = VECTOR3(0,0,0);
//     body->force_      = VECTOR3(0,0,0);
//     body->torque_     = VECTOR3(0,0,0);
//     body->restitution_ = 0.0;
//     body->setOrientation(body->angle_);
//     body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//     //calculate the inertia tensor
//     //Get the inertia tensor
//     body->generateInvInertiaTensor();
//     body->translateTo(pos);
//     pos.z+=d;
//   }
// 
//   iboxes = myWorld.rigidBodies_.size();
//   myFactory.addBoxes(myWorld.rigidBodies_,towerheight,extends);
//   pos = VECTOR3(myGrid.m_vMax.x/2.0+10.0*d, myGrid.m_vMax.y/2.0, extends[2]);
// 
//   for(int j=iboxes;j<myWorld.rigidBodies_.size();j++)
//   {
//     RigidBody *body    = myWorld.rigidBodies_[j];
//     if(!body->affectedByGravity_)
//       continue;
//     body->density_    = myParameters.defaultDensity_;
//     body->volume_     = body->shape_->Volume();
//     Real dmass          = body->density_ * body->volume_;
//     body->invMass_    = 1.0/(body->density_ * body->volume_);
//     body->angle_      = VECTOR3(0,0,0);
//     body->setAngVel(VECTOR3(0,0,0));
//     body->velocity_   = VECTOR3(0,0,0);
//     body->com_        = VECTOR3(0,0,0);
//     body->force_      = VECTOR3(0,0,0);
//     body->torque_     = VECTOR3(0,0,0);
//     body->restitution_ = 0.0;
//     body->setOrientation(body->angle_);
//     body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//     //calculate the inertia tensor
//     //Get the inertia tensor
//     body->generateInvInertiaTensor();
//     body->translateTo(pos);
//     pos.z+=d;
//   }
//   
//   iboxes = myWorld.rigidBodies_.size();
//   myFactory.addBoxes(myWorld.rigidBodies_,1,extends);
//   pos = VECTOR3(myGrid.m_vMax.x/2.0-4.0*d, myGrid.m_vMax.y/2.0, 7.25 * extends[2]);
// 
//   RigidBody *body    = myWorld.rigidBodies_.back();
//   body->density_    = myParameters.defaultDensity_;
//   body->volume_     = body->shape_->Volume();
//   Real dmass          = body->density_ * body->volume_;
//   body->invMass_    = 1.0/(body->density_ * body->volume_);
//   body->angle_      = VECTOR3(0,0,0);
//   body->setAngVel(VECTOR3(0,0,0));
//   body->velocity_   = VECTOR3(5.0,0,0);
//   body->com_        = VECTOR3(0,0,0);
//   body->force_      = VECTOR3(0,0,0);
//   body->torque_     = VECTOR3(0,0,0);
//   body->restitution_ = 0.0;
//   body->setOrientation(body->angle_);
//   body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//   //calculate the inertia tensor
//   //Get the inertia tensor
//   body->generateInvInertiaTensor();
//   body->translateTo(pos);
//   pos.z+=d;
// 
// 
// }
// 
// void addobstacle()
// {
// 
//   ObjLoader Loader;
// 
//   RigidBody *body = new RigidBody();
//   CMeshObject<Real> *pMeshObject= new CMeshObject<Real>();
// 
//   Loader.readMultiMeshFromFile(&pMeshObject->m_Model,"meshes/fritten_final_mili.obj");
// 
//   pMeshObject->m_Model.GenerateBoundingBox();
// 
//   pMeshObject->SetFileName("meshes/fritten_final_mili.obj");
// 
//   body->shape_ = pMeshObject;
//   body->shape_ = RigidBody::MESH;
//   myWorld.rigidBodies_.push_back(body);
// 
//   //initialize the simulation with some useful physical parameters
//   //initialize the box shaped boundary
// 
//   body->affectedByGravity_ = false;
//   body->density_  = 0;
//   body->volume_   = 0;
//   body->invMass_     = 0;
//   body->angle_    = VECTOR3(1.57,0,0);
//   body->setAngVel(VECTOR3(0,0,0));
//   body->velocity_ = VECTOR3(0,0,0);
//   body->shape_    = RigidBody::MESH;
// 
//   body->com_      = VECTOR3(0,0,0);
// 
//   body->invInertiaTensor_.SetZero();
// 
//   body->restitution_ = 0.0;
// 
//   body->setOrientation(body->angle_);
//   body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//   body->translateTo(VECTOR3(0.0,myGrid.m_vMax.y/2.0+0.15,0.0));
// 
//   C3DModel model_out(pMeshObject->m_Model);
//   model_out.GenerateBoundingBox();
//   for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
//   {
//     model_out.m_vMeshes[i].m_matTransform =body->getTransformationMatrix();
//     model_out.m_vMeshes[i].m_vOrigin =body->com_;
//     model_out.m_vMeshes[i].TransformModelWorld();
//     model_out.m_vMeshes[i].GenerateBoundingBox();
//   }
// 
//   std::vector<Triangle3r> pTriangles = model_out.GenTriangleVector();
//   CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
//   subdivider = CSubdivisionCreator(&myRessources);
//   pMeshObject->m_BVH.InitTree(&subdivider);
//   pMeshObject->m_BVH.GenTreeStatistics();
// 
// }
// 
// void createlineuptest()
// {
//   ParticleFactory myFactory;
//   int offset = myWorld.rigidBodies_.size();
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
//   myFactory.addSpheres(myWorld.rigidBodies_,1,myParameters.defaultRadius_);
//   initphysicalparameters();
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.25 * drad;
//   int perrow = myGrid.m_vMax.x/(distbetween+d);
//   VECTOR3 pos(myGrid.m_vMin.x+1.0*drad, myGrid.m_vMax.y/2.0, myGrid.m_vMin.z+6.0*drad);
//   myWorld.rigidBodies_[0]->translateTo(pos);
//   distbetween = 0.0 * drad;
//   pos.z+=d;
//   distbetween = 0.0 * drad;
//   for(int i=1;i<myWorld.rigidBodies_.size();i++)
//   {
//     myWorld.rigidBodies_[i]->translateTo(pos);
//     pos.z+=d+distbetween;
//   }
// 
// 
// /*  myFactory.AddSpheres(myWorld.m_vRigidBodies,1,myParameters.m_dDefaultRadius);
//   CRigidBody *body    = myWorld.m_vRigidBodies.back();
//   body->m_dDensity    = myParameters.m_dDefaultDensity;
//   body->m_dVolume     = body->m_pShape->Volume();
//   Real dmass          = body->m_dDensity * body->m_dVolume;
//   body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
//   body->m_vAngle      = VECTOR3(0,0,0);
//   body->SetAngVel(VECTOR3(0,0,0));
//   body->m_vVelocity   = VECTOR3(0.0,0,-0.25);
//   body->m_vCOM        = VECTOR3(0,0,0);
//   body->m_vForce      = VECTOR3(0,0,0);
//   body->m_vTorque     = VECTOR3(0,0,0);
//   body->m_Restitution = 0.0;
//   body->SetOrientation(body->m_vAngle);
//   body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
//   body->GenerateInvInertiaTensor();
//   distbetween = 9.0 * drad;
//   pos.z+=d+distbetween;
//   body->TranslateTo(pos);*/
//   
//   addobstacle();
// 
// }
// 
// void createstackingtest()
// {
//   ParticleFactory myFactory;
//   Real extends[3] = {myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
//   //myWorld = myFactory.ProduceSpheres(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
//   myWorld = myFactory.produceCylinders(myParameters.bodies_,extends);
//   initphysicalparameters();
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.25 * drad;
//   int perrow = myGrid.m_vMax.x/(distbetween+d);
//   VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween, (myGrid.m_vMax.z/2.0)-d);
//   myWorld.rigidBodies_[0]->translateTo(pos);
//   pos.x+=d+distbetween;
//   bool even=(perrow%2==0) ? true : false;
//   Real ynoise = 0.005;
//   for(int i=1;i<myWorld.rigidBodies_.size();i++)
//   {
//     if((i)%(perrow) == 0)
//     {
//       //advance in y and reset x
//       pos.x = myGrid.m_vMin.x+drad+distbetween;
//       pos.z -= d+distbetween;
//       if(even)
//       {
//         ynoise = -ynoise;
//       }
//     }
//     VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
//     myWorld.rigidBodies_[i]->translateTo(bodypos);
//     pos.x+=d+distbetween;
//     ynoise = -ynoise;
//   }
// }
// 
// void randposition()
// {
// bool finish=false;
// int x,y;
// ParticleFactory myFactory;
// 
//   while(!finish)
//   {
//     x = rand()%perrowx;
//     y = rand()%perrowy;
//     //check if there is a sphere in this slot
//     if(islots[x*perrowy+y]==0)
//     {
//       myFactory.addSpheres(myWorld.rigidBodies_,1,myParameters.defaultRadius_);
//       //move to center of grid cell 
//       islots[x*perrowy+y]=1;
//       finish=true;
//     }
//   }
// }
// 
// void addobject()
// {
// 
//   Real timeElapsed = myTimeControl.GetTime() - startTime;
//   if((timeElapsed > 0.1) && (myWorld.rigidBodies_.size() < 256))
//   {
//     //add a few spheres
//     for(int i=0;i<4;i++)
//     {
//       randposition();
//     }//end for
//   }//end if
// 
//   for(int x=0;x<perrowx;x++)
//     for(int y=0;y<perrowy;y++)
//       islots[x*perrowy+y]=0;
// 
// }
// 
// void addsphere_dt(int istep)
// {
// 
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad + 0.01;
// 
// 
//   std::vector<VECTOR3> vPos;
//   VECTOR3 pos(0.0,0.0,7.7);
//   vPos.push_back(pos);
//   double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand()/(double)RAND_MAX);
//   pos = VECTOR3(d*cos(radian),d*sin(radian),7.7);
//   vPos.push_back(pos);
//   radian += CMath<double>::SYS_PI/2.0;
//   pos = VECTOR3(d*cos(radian),d*sin(radian),7.7);
//   vPos.push_back(pos);
//   radian += CMath<double>::SYS_PI/2.0;
//   pos = VECTOR3(d*cos(radian),d*sin(radian),7.7);
//   vPos.push_back(pos);
//   radian += CMath<double>::SYS_PI/2.0;
//   pos = VECTOR3(d*cos(radian),d*sin(radian),7.7);
//   vPos.push_back(pos);  
// 
//   
//   int iadd = 5;
//   int iStart = istep;
//   int iSeed = 1;
// 
//   if(istep == 0)
//     return;
// 
//   if(istep%36 != 0)
//     return;
//     
//   Real noise = 0.0005;
//   
//   if(myWorld.rigidBodies_.size() < 1000)
//   {
//     ParticleFactory myFactory;
// 
//     int offset = myWorld.rigidBodies_.size();
// 
//     Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
// 
//     myFactory.addSpheres(myWorld.rigidBodies_,iadd,myParameters.defaultRadius_);
//     
//     for(int i=0;i<iadd;i++)
//     {
//       RigidBody *body    = myWorld.rigidBodies_[offset+i];
//       body->density_    = myParameters.defaultDensity_;
//       body->volume_     = body->shape_->Volume();
//       Real dmass          = body->density_ * body->volume_;
//       body->invMass_    = 1.0/(body->density_ * body->volume_);
//       body->angle_      = VECTOR3(0,0,0);
//       body->setAngVel(VECTOR3(0,0,0));
//       body->velocity_   = VECTOR3(0,0,-1.05);
//       body->com_        = VECTOR3(0,0,0);
//       body->force_      = VECTOR3(0,0,0);
//       body->torque_     = VECTOR3(0,0,0);
//       body->restitution_ = 0.0;
//       body->setOrientation(body->angle_);
//       body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//       
//       //calculate the inertia tensor
//       //Get the inertia tensor
//       body->generateInvInertiaTensor();
//       pos = vPos[i];
//       body->translateTo(pos);      
//     }
//   }//end if
// 
//   myPipeline.graph_->clear();
// 
//   //assign the rigid body ids
//   for(int j=0;j<myWorld.rigidBodies_.size();j++)
//     myWorld.rigidBodies_[j]->iID_ = j;
// 
//   std::cout<<"Added body, number of particles: "<<myWorld.rigidBodies_.size()<<std::endl;
// 
// }
// 
// //-------------------------------------------------------------------------------------------------------
// 
// void drivcav()
// {
//   
//   ParticleFactory myFactory;
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};
// 
//   Real myxmin = 2.0;  
//   Real myymin = 0.0;  
//   Real myzmin = 0.0;  
// 
//   Real myxmax = 6.0;  
//   Real myymax = 2.0;  
//   Real myzmax = 1.0;  
// 
// 
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real dz    = 4.0 * drad;
//   Real distbetween = 1.0 * drad;
//   Real distbetweenz = 0.5 * drad;
// 
//   Real extendX = myxmax - myxmin;  
//   Real extendY = myymax - myymin;  
//   Real extendZ = myzmax - myzmin;  
// 
//   int perrowx = 1; //extendX/(distbetween+d);
//   int perrowy = extendY/(distbetween+d);  
//   
//   int numPerLayer = perrowx * perrowy;
//   int layers = 4;
//   int nTotal = numPerLayer * layers;
// 
//   //add the desired number of particles
//   myFactory.addSpheres(myWorld.rigidBodies_,numPerLayer*layers,myParameters.defaultRadius_);  
//   initphysicalparameters();
//   
//   VECTOR3 pos(myxmin+drad+distbetween , myymin+drad+distbetween+0.0025, (myzmin+drad));
//   
//   Real ynoise = 0.0025;
//   int count=0;
//     
//   for(int z=0;z<layers;z++)
//   {
//     for(int j=0;j<perrowy;j++)
//     {
//       for(int i=0;i<perrowx;i++,count++)
//       {
//         //one row in x
//         VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
//         myWorld.rigidBodies_[count]->translateTo(bodypos);
//         pos.x+=d+distbetween;
//       }
//       pos.x=myxmin+drad+distbetween;
//       pos.y+=d+distbetween;    
//     }
//     ynoise = -ynoise;        
//     pos.z+=d;
//     pos.y=myymin+drad+distbetween+0.0025;        
//   }
// 
// }
// 
// //-------------------------------------------------------------------------------------------------------
// 
// void spherestack()
// {
//   
//   ParticleFactory myFactory;
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};
// 
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real dz    = 4.0 * drad;
//   Real distbetween = 0.25 * drad;
//   Real distbetweenz = 0.5 * drad;
//   int perrowx = myGrid.m_vMax.x/(distbetween+d);
//   int perrowy = myGrid.m_vMax.y/(distbetween+d);  
//   
//   int numPerLayer = perrowx * perrowy;
//   int layers = 5;
//   int nTotal = numPerLayer * layers;
// 
//   //add the desired number of particles
//   myFactory.addSpheres(myWorld.rigidBodies_,numPerLayer*layers,myParameters.defaultRadius_);  
//   initphysicalparameters();
//   
//   //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
//   VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+0.0025, (1.5));
//   
//   //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
//   Real ynoise = 0.0025;
//   int count=0;
//     
//   for(int z=0;z<layers;z++)
//   {
//     for(int j=0;j<perrowy;j++)
//     {
//       for(int i=0;i<perrowx;i++,count++)
//       {
//         //one row in x
//         VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
//         myWorld.rigidBodies_[count]->translateTo(bodypos);
//         pos.x+=d+distbetween;
//       }
//       pos.x=myGrid.m_vMin.x+drad+distbetween;
//       pos.y+=d+distbetween;    
//     }
//     ynoise = -ynoise;        
//     pos.z-=d;
//     pos.y=myGrid.m_vMin.y+drad+distbetween+0.0025;        
//   }
// 
// }
//  
// //-------------------------------------------------------------------------------------------------------
// 
// void sphericalstack()
// {
//   
//   ParticleFactory myFactory;
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};
// 
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.5 * drad;
//   int perrowx = 0.5/(distbetween+d);
//   int perrowy = perrowx-2;
//   Real z=7.7;
//   Real x=-0.25+drad+distbetween;
//   Real y=-0.25+drad+distbetween;
//   int layers = 1;
//   std::vector<VECTOR3> vPos;
// 
//   for(int layer=0;layer<layers;layer++)
//   {
//     double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand()/(double)RAND_MAX);
//     // make an x-row and rotate
//     for(int i=0;i<perrowx;i++)
//     {
//       VECTOR3 pos(x,0,0);
//       MATRIX3X3 rotmat;
//       rotmat.MatrixFromAngles(VECTOR3(0,0,radian));
//       pos = rotmat * pos;
//       pos.z=z;
//       //pos = VECTOR3(x+fabs(x)*cos(radian),fabs(x)*sin(radian),z);
//       vPos.push_back(pos);
//       x+=d+distbetween;
//     }
//     //yrow
//     for(int i=0;i<perrowy;i++)
//     {
//       VECTOR3 pos(0,y,0);
//       MATRIX3X3 rotmat;
//       rotmat.MatrixFromAngles(VECTOR3(0,0,radian));
//       pos = rotmat * pos;
//       pos.z=z;
//       //pos = VECTOR3(x+fabs(x)*cos(radian),fabs(x)*sin(radian),z);
//       vPos.push_back(pos);
//       if(i==0)
//         y=(d+distbetween);
//       else
//         y+=d+distbetween;
//     }
//     y=-0.25+drad+distbetween;
//     x=-0.25+drad+distbetween;
//     z-=d+1.5*distbetween;
//   }
// 
//   int numPerLayer = perrowx + perrowy;
//   myFactory.addSpheres(myWorld.rigidBodies_,numPerLayer*layers,myParameters.defaultRadius_);  
//   initphysicalparameters();
// 
//   std::vector<RigidBody*>::iterator vIter;
//   std::vector<VECTOR3>::iterator i;
// 
//   for(vIter=myWorld.rigidBodies_.begin(),i=vPos.begin();vIter!=myWorld.rigidBodies_.end();vIter++,i++)
//   {
//     RigidBody *body    = *vIter;
//     VECTOR3 pos         = *i;
//     body->translateTo(pos);
//   }
// 
// }
// 
// 
// void addsphere_dt()
// {
//   bool addsphere=false;
//   if(myTimeControl.GetTimeStep() == 0)
//     addsphere=false;
//   else if(myTimeControl.GetTimeStep()%100 == 0)
//     addsphere=true;
// 
//   if((addsphere) && (myWorld.rigidBodies_.size() < 256))
//   {
//     ParticleFactory myFactory;
// 
//     int offset = myWorld.rigidBodies_.size();
// 
//     Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
// 
//     myFactory.addSpheres(myWorld.rigidBodies_,1,myParameters.defaultRadius_);
// 
//     RigidBody *body    = myWorld.rigidBodies_.back();
//     body->density_    = myParameters.defaultDensity_;
//     body->volume_     = body->shape_->Volume();
//     Real dmass          = body->density_ * body->volume_;
//     body->invMass_    = 1.0/(body->density_ * body->volume_);
//     body->angle_      = VECTOR3(0,0,0);
//     body->setAngVel(VECTOR3(0,0,0));
//     body->velocity_   = VECTOR3(0,0,0);
//     body->com_        = VECTOR3(0,0,0);
//     body->force_      = VECTOR3(0,0,0);
//     body->torque_     = VECTOR3(0,0,0);
//     body->restitution_ = 0.0;
//     body->setOrientation(body->angle_);
//     body->setTransformationMatrix(body->getQuaternion().GetMatrix());
// 
// 
//     VECTOR3 pos(myGrid.m_vMin.x+myParameters.defaultRadius_, myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-2.0*myParameters.defaultRadius_);
// 
//     body->translateTo(pos);
// 
//   }//end if
// 
//   myPipeline.graph_->clear();
// 
//   //assign the rigid body ids
//   for(int j=0;j<myWorld.rigidBodies_.size();j++)
//     myWorld.rigidBodies_[j]->iID_ = j;
// 
// 
// }
// 
// void addspheres()
// {
//   
//   ParticleFactory myFactory;
//   int offset = myWorld.rigidBodies_.size();
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
//   //myFactory.AddSpheres(myWorld.m_vRigidBodies,175,myParameters.m_dDefaultRadius);
//   //myFactory.AddCylinders(myWorld.m_vRigidBodies,24,extends);
//   //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
//   //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
//   //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
//   myFactory.addSpheres(myWorld.rigidBodies_,512,myParameters.defaultRadius_);
//   //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
//   //myFactory.AddSpheres(myWorld.m_vRigidBodies,175,myParameters.m_dDefaultRadius);
//   //myFactory.AddCylinders(myWorld.m_vRigidBodies,24,extends);
//   //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
//   //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
//   //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
//   initphysicalparameters();
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.25 * drad;
//   int perrow = myGrid.m_vMax.x/(distbetween+d);
//   //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
//   VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween, (myGrid.m_vMax.z/1.0)-d);
//   //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
//   myWorld.rigidBodies_[offset]->translateTo(pos);
//   pos.x+=d+distbetween;
//   bool even=(perrow%2==0) ? true : false;
//   Real ynoise = 0.005;
//   int count=0;
//   for(int i=offset+1;i<myWorld.rigidBodies_.size();i++)
//   {
//     if((i)%(perrow) == 0)
//     {
//       //advance in y and reset x
//       pos.x = myGrid.m_vMin.x+drad+distbetween;
//       pos.y += d+distbetween;
//       if(even)
//       {
//         ynoise = -ynoise;
//       }
//       if(++count==3)
//       {
//         pos.z -= d+distbetween;
//         pos.y=myGrid.m_vMin.y+drad+distbetween;
//         count=0;
//       }
//     }
//     VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
//     myWorld.rigidBodies_[i]->translateTo(bodypos);
//     pos.x+=d+distbetween;
//     ynoise = -ynoise;
//   }
// 
// } 
//  
// void meshtorus()
// {
//   ParticleFactory myFactory;
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
//   myWorld = myFactory.produceMesh("meshes/cup_small_high.obj");
//   Real extentBox[3]={0.25, 0.25, 0.025};
//   myFactory.addBoxes(myWorld.rigidBodies_,1,extentBox);
//   myFactory.addSpheres(myWorld.rigidBodies_,20,myParameters.defaultRadius_);
// 
//   //assign the physical parameters of the rigid bodies
//   initphysicalparameters();
// 
//   myWorld.rigidBodies_[0]->translateTo(VECTOR3(0.49,0.25,0.378));
//   myWorld.rigidBodies_[1]->translateTo(VECTOR3(0.75, 0.25, 0.28));
//   myWorld.rigidBodies_[1]->affectedByGravity_=false;
//   myWorld.rigidBodies_[1]->invMass_=0;
//   myWorld.rigidBodies_[1]->invInertiaTensor_.SetZero();
//   CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.rigidBodies_[0]->shape_);
// 
//   C3DModel model_out(pMeshObject->m_Model);
//   model_out.m_vMeshes[0].m_matTransform =myWorld.rigidBodies_[0]->getTransformationMatrix();
//   model_out.m_vMeshes[0].m_vOrigin =myWorld.rigidBodies_[0]->com_;
//   model_out.m_vMeshes[0].TransformModelWorld();
//   model_out.GenerateBoundingBox();
//   model_out.m_vMeshes[0].GenerateBoundingBox();
//   std::vector<Triangle3r> pTriangles = model_out.GenTriangleVector();
//   CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
//   subdivider = CSubdivisionCreator(&myRessources);
//   pMeshObject->m_BVH.InitTree(&subdivider);
//   pMeshObject->m_BVH.GenTreeStatistics();
//   
//   int offset = 2;
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.25 * drad;
//   int perrow = 7;//myGrid.m_vMax.x/(distbetween+d);
//   //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
//   Real xstart=myGrid.m_vMin.x + (myGrid.m_vMax.x/2.5) - (drad+distbetween);
//   Real ystart=myGrid.m_vMin.y+drad+distbetween+myGrid.m_vMax.y/3.0;  
//   VECTOR3 pos(xstart , ystart, (myGrid.m_vMax.z/1.7)-d);
//   //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
//   myWorld.rigidBodies_[offset]->translateTo(pos);
//   pos.x+=d+distbetween;
//   bool even=(perrow%2==0) ? true : false;
//   Real ynoise = 0.0015;
//   int count=0;
//   for(int i=offset+1;i<myWorld.rigidBodies_.size();i++)
//   {
//     if((i)%(perrow) == 0)
//     {
//       //advance in y and reset x
//       pos.x = xstart;
//       pos.y += d+distbetween;
//       if(even)
//       {
//         ynoise = -ynoise;
//       }
//       if(++count==6)
//       {
//         pos.z -= d+distbetween;
//         pos.y=ystart;
//         count=0;
//       }
//     }
//     VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
//     myWorld.rigidBodies_[i]->translateTo(bodypos);
//     pos.x+=d+distbetween;
//     ynoise = -ynoise;
//   }  
// 
// /*  myWorld.m_vRigidBodies.back()->TranslateTo(VECTOR3(0.5,0.25,0.08));
// 
//   Real drad = myParameters.m_dDefaultRadius;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.25 * drad;
// 
//   perrowx = myGrid.m_vMax.x/(distbetween+d);
//   perrowy = myGrid.m_vMax.y/(distbetween+d);
// 
//   islots = new int[perrowx*perrowy];
//   for(int x=0;x<perrowx;x++)
//     for(int y=0;y<perrowy;y++)
//       islots[x*perrowy+y]=0;*/
//      
//   
// }
// 
// void createrestingtest()
// {
//   Real drad = myWorld.rigidBodies_[0]->shape_->GetAABB().m_Extends[0];
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.5 * drad;
//   int perrow = myGrid.m_vMax.x/(distbetween+d);
//   VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMin.z)+drad);
//   myWorld.rigidBodies_[0]->translateTo(pos);
//   pos.z+=d;//+distbetween;
//   for(int i=1;i<myWorld.rigidBodies_.size();i++)
//   {
//     if((i)%(perrow) == 0)
//     {
//       //advance in y and reset x
//       pos.x = myGrid.m_vMin.x+drad+distbetween;
//       pos.z += d;
//     }
//     myWorld.rigidBodies_[i]->translateTo(pos);
//     pos.z+=d;//+distbetween;
//   }
// }
// 
// void add()
// {
//   
//   ParticleFactory myFactory;
//   int offset = myWorld.rigidBodies_.size();
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};
//   //myFactory.AddSpheres(myWorld.m_vRigidBodies,175,myParameters.m_dDefaultRadius);
//   myFactory.addCylinders(myWorld.rigidBodies_,24,extends);
// 
//   //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
//   //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
//   //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
//   //myFactory.AddSpheres(myWorld.m_vRigidBodies,124,myParameters.m_dDefaultRadius);
//   //myFactory.AddSpheres(myWorld.m_vRigidBodies,4,0.05);
//   initphysicalparameters();
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real dz    = 4.0 * drad;
//   Real distbetween = 0.25 * drad;
//   Real distbetweenz = 0.5 * drad;
//   int perrow = myGrid.m_vMax.x/(distbetween+d);
//   //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
//   VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+0.002, (myGrid.m_vMax.z/2.0)-dz);
//   //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
//   myWorld.rigidBodies_[offset]->translateTo(pos);
//   pos.x+=d+distbetween;
//   bool even=(perrow%2==0) ? true : false;
//   Real ynoise = 0.0025;
//   int count=0;
//   for(int i=offset+1;i<myWorld.rigidBodies_.size();i++)
//   {
//     if((i)%(perrow) == 0)
//     {
//       //advance in y and reset x
//       pos.x = myGrid.m_vMin.x+drad+distbetween;
//       pos.y += d+distbetween;
//       if(even)
//       {
//         ynoise = -ynoise;
//       }
//       if(++count==3)
//       {
//         pos.z -= dz+distbetweenz;
//         pos.y=myGrid.m_vMin.y+drad+distbetween+0.002;
//         count=0;
//       }
//     }
//     VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
//     myWorld.rigidBodies_[i]->translateTo(bodypos);
//     pos.x+=d+distbetween;
//     ynoise = -ynoise;
//   }
// 
// }
// 
// void reactor()
// {
//   ParticleFactory myFactory;
//   int offset = myWorld.rigidBodies_.size();
//   Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
// 
//   myFactory.addSpheres(myWorld.rigidBodies_,1,myParameters.defaultRadius_);
//   
//   RigidBody *body    = myWorld.rigidBodies_.back();
//   body->density_    = myParameters.defaultDensity_;
//   body->volume_     = body->shape_->Volume();
//   Real dmass          = body->density_ * body->volume_;
//   body->invMass_    = 1.0/(body->density_ * body->volume_);
//   body->angle_      = VECTOR3(0,0,0);
//   body->setAngVel(VECTOR3(0,0,0));
//   body->velocity_   = VECTOR3(0,0,0);
//   body->com_        = VECTOR3(0,0,0);
//   body->force_      = VECTOR3(0,0,0);
//   body->torque_     = VECTOR3(0,0,0);
//   body->restitution_ = 0.0;
//   body->generateInvInertiaTensor();  
//   body->setOrientation(body->angle_);
//   body->setTransformationMatrix(body->getQuaternion().GetMatrix());
// 
//   Real drad = myParameters.defaultRadius_;
//   Real d    = 2.0 * drad;
//   Real distbetween = 0.25 * drad;
// 
//   VECTOR3 pos(0.0+distbetween+drad, 0, 0);
//   body->translateTo(pos);
//   body->velocity_=VECTOR3(0.075,0,0);
// 
//   //addobstacle();
// 
// }
// 
// void initrigidbodies()
// {
//   ParticleFactory myFactory;
// 
//   //Produces a domain
//   //it is a bit unsafe, because the domain at this point is
//   //not fully useable, because of non initialized values in it
//   //myWorld = myFactory.ProduceSphericalWithObstacles(iPart);
//   //myWorld = myFactory.ProduceSpherical(iPart);
// 
//   if(myParameters.bodyInit_ == 0)
//   {
//     myWorld = myFactory.produceFromParameters(myParameters);
//   }
//   else if(myParameters.bodyInit_ == 1)
//   {
//     myWorld = myFactory.produceFromFile(myParameters.bodyConfigurationFile_.c_str(),myTimeControl);
//   }
//   else
//   {
//     if(myParameters.bodyInit_ == 2)
//     {
//       meshtorus();
//     }
// 
//     if(myParameters.bodyInit_ == 3)
//     {
//       pyramidtest();
//     }
// 
//     if(myParameters.bodyInit_ == 4)
//     {
//       myWorld = myFactory.produceFromParameters(myParameters);      
//       reactor();
//     }
// 
//     if(myParameters.bodyInit_ == 5)
//     {
//       spherestack();
//     }
// 
//     if(myParameters.bodyInit_ == 6)
//     {
//       drivcav();
//     }
//     
//   }
// 
//   //initialize the box shaped boundary
//   myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
//   myBoundary.CalcValues();
// 
//   //add the boundary as a rigid body
//   addboundary();
//   
// }
// 
// void initsimulation()
// {
// 
//   //first of all initialize the rigid bodies
//   initrigidbodies();
// 
//   //assign the rigid body ids
//   for(int j=0;j<myWorld.rigidBodies_.size();j++)
//   {
//     myWorld.rigidBodies_[j]->iID_ = j;
//     if(myWorld.rigidBodies_[j]->getShape() != RigidBody::BOUNDARYBOX)
//     {
//       myWorld.rigidBodies_[j]->friction_ = 0.45;
//     }
//   }
// 
//   //set the timestep
//   myTimeControl.SetDeltaT(myParameters.timeStep_);
//   myTimeControl.SetTime(0.0);
//   myTimeControl.SetCautiousTimeStep(0.005);
//   myTimeControl.SetPreferredTimeStep(0.005);
//   myTimeControl.SetReducedTimeStep(0.0001);
//   myTimeControl.SetTimeStep(0);
// 
//   //link the boundary to the world
//   myWorld.setBoundary(&myBoundary);
// 
//   //set the time control
//   myWorld.setTimeControl(&myTimeControl);
// 
//   //set the gravity
//   myWorld.setGravity(myParameters.gravity_);
// 
//   //Set the collision epsilon
//   myPipeline.setEPS(0.02);
// 
//   //initialize the collision pipeline 
//   myPipeline.init(&myWorld,myParameters.m_iSolverType,myParameters.maxIterations_,myParameters.pipelineIterations_);
// 
//   //set the broad phase to simple spatialhashing
//   myPipeline.setBroadPhaseHSpatialHash();
//   //myPipeline.SetBroadPhaseNaive();
//   //myPipeline.SetBroadPhaseSpatialHash();
// 
//   //set which type of rigid motion we are dealing with
//   myMotion=MotionIntegratorSI(&myWorld);
// 
//   //set the integrator in the pipeline
//   myPipeline.integrator_ = &myMotion;
//  
//   myWorld.densityMedium_ = myParameters.densityMedium_;
//   
//   myPipeline.response_->m_pGraph = myPipeline.graph_;  
// 
// }
// 
// void continuesimulation()
// {
//   
//   ParticleFactory myFactory;
// 
//   //Produces a domain
//   //it is a bit unsafe, because the domain at this point is
//   //not fully useable, because of non initialized values in it
//   //string = ssolution
//   myWorld = myFactory.produceFromFile(myParameters.solutionFile_.c_str(),myTimeControl);
// 
//   //initialize the box shaped boundary
//   myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
//   myBoundary.CalcValues();
//   
//   //set the timestep
//   myTimeControl.SetCautiousTimeStep(0.005);
//   myTimeControl.SetPreferredTimeStep(0.005);
//   myTimeControl.SetReducedTimeStep(0.0001);
//   myParameters.nTimesteps_+=myTimeControl.GetTimeStep();
// 
//   //link the boundary to the world
//   myWorld.setBoundary(&myBoundary);
// 
//   //set the time control
//   myWorld.setTimeControl(&myTimeControl);
// 
//   //set the gravity
//   myWorld.setGravity(myParameters.gravity_);
// 
//   //Set the collision epsilon
//   myPipeline.setEPS(0.02);
// 
//   //initialize the collision pipeline 
//   myPipeline.init(&myWorld,myParameters.m_iSolverType,myParameters.maxIterations_,myParameters.pipelineIterations_);
// 
//   //set the broad phase to simple spatialhashing
//   myPipeline.setBroadPhaseHSpatialHash();
//   //myPipeline.SetBroadPhaseNaive();
//   //myPipeline.SetBroadPhaseSpatialHash();
// 
//   //set which type of rigid motion we are dealing with
//   myMotion=MotionIntegratorSI(&myWorld);
// 
//   //set the integrator in the pipeline
//   myPipeline.integrator_ = &myMotion;
// 
//   myWorld.densityMedium_ = myParameters.densityMedium_;
//   
//   myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;
//   
//   myPipeline.response_->m_pGraph = myPipeline.graph_;  
// 
//   RigidBody *body    = myWorld.rigidBodies_[4];
//   //body->m_InvInertiaTensor.SetZero();
//   body->setAngVel(VECTOR3(0,0,0));
// 
// }
// 
// void writetimestep(int iout)
// {
//   std::ostringstream sName,sNameParticles,sContacts;
//   std::string sModel("output/model.vtk");
//   std::string sParticle("solution/particles.i3d");
//   CVtkWriter writer;
//   int iTimestep=iout;
//   sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//   sNameParticles<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//   sModel.append(sName.str());
//   sParticle.append(sNameParticles.str());
//   sContacts<<"output/contacts.vtk."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//   //Write the grid to a file and measure the time
//   writer.WriteRigidBodies(myWorld.rigidBodies_,sModel.c_str());
//   RigidBodyIO rbwriter;
//   myWorld.output_ = iTimestep;
//   std::vector<int> indices;
//   indices.push_back(12);
//   indices.push_back(13);
//   indices.push_back(15);
//   rbwriter.write(myWorld,indices,sParticle.c_str());
//   rbwriter.write(myWorld,sParticle.c_str());
//   writer.WriteContacts(myPipeline.contacts_,sContacts.str().c_str());
// 
//   std::ostringstream sNameHGrid;
//   std::string sHGrid("output/hgrid.vtk");
//   sNameHGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//   sHGrid.append(sNameHGrid.str());
//   
//   //iterate through the used cells of spatial hash
//   SpatialHashHierarchy *pHash = dynamic_cast<SpatialHashHierarchy*>(myPipeline.broadPhase_->strategy_->implicitGrid_->getSpatialHash());  
//   
//   CUnstrGridr hgrid;
//   pHash->convertToUnstructuredGrid(hgrid);
// 
//   writer.WriteUnstr(hgrid,sHGrid.c_str());  
//   
//   
//   if(iout==0)
//   {
//     std::ostringstream sNameGrid;
//     std::string sGrid("output/grid.vtk");
//     sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
//     sGrid.append(sNameGrid.str());
//     writer.WriteUnstr(myGrid,sGrid.c_str());
//   }
// }
// 
// int main()
// {
//   using namespace std;
//   int iOut=0;
//   Real dTimePassed=1;
//   Real energy0=0.0;
//   Real energy1=0.0;
//   Reader reader;
//   std::string meshFile=std::string("meshes/mesh.tri");
//   //read the user defined configuration file
//   reader.readParameters(string("start/data.TXT"),myParameters);
// 
//   //initialize the grid
//   if(iReadGridFromFile == 1)
//   {
//     myGrid.InitMeshFromFile(meshFile.c_str());
//     //refine grid: Parameter iMaxRefinement
//   }
//   else
//   {
//     myGrid.InitCube(xmin,ymin,zmin,xmax,ymax,zmax);
//   }
// 
//   //initialize a start from zero or
//   //continue a simulation
//   if(myParameters.startType_ == 0)
//   {
//     initsimulation();
//   }
//   else
//   {
//     continuesimulation();
//   }
// 
//   //start the main simulation loop
//   for(;myWorld.timeControl_->m_iTimeStep<=myParameters.nTimesteps_;myWorld.timeControl_->m_iTimeStep++)
//   {
//     Real simTime = myTimeControl.GetTime();
//     energy0=myWorld.getTotalEnergy();
//     cout<<"------------------------------------------------------------------------"<<endl;
//     cout<<"## Timestep Nr.: "<<myWorld.timeControl_->m_iTimeStep<<" | Simulation time: "<<myTimeControl.GetTime()
//       <<" | time step: "<<myTimeControl.GetDeltaT() <<endl;
//     cout<<"Energy: "<<energy0<<endl;
//     cout<<"------------------------------------------------------------------------"<<endl;
//     cout<<endl;
//     //addsphere_dt();
//     myPipeline.startPipeline();
//     energy1=myWorld.getTotalEnergy();
//     cout<<"Energy after collision: "<<energy1<<endl;
//     cout<<"Energy difference: "<<energy0-energy1<<endl;
//     //addsphere_dt(myWorld.m_pTimeControl->m_iTimeStep);
//     //if(dTimePassed >= myTimeControl.GetPreferredTimeStep())
//     //{
//       std::cout<<"Timestep finished... writing vtk."<<std::endl;
//       writetimestep(iOut);
//       std::cout<<"Finished writing vtk."<<std::endl;
//       iOut++;
//       dTimePassed = 0.f;
// //    }
//     myTimeControl.SetTime(simTime+myTimeControl.GetDeltaT());
//     dTimePassed += myTimeControl.GetDeltaT();
//   }//end for
// 
//   cleanup();
// 
//   return 0;
// }
#include <iostream>
#include <application.h>

namespace i3d {

  class InclinedPlane : public Application<> {

  public:

    InclinedPlane() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void run() {

      unsigned nOut = 0;
      //start the main simulation loop
      for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
      {
        Real simTime = myTimeControl_.GetTime();
        Real energy0 = myWorld_.getTotalEnergy();
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
        std::cout << "Energy: " << energy0 << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
        myPipeline_.startPipeline();
        Real energy1 = myWorld_.getTotalEnergy();
        std::cout << "Energy after collision: " << energy1 << std::endl;
        std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
        Application::writeOutput(nOut,true,true);
        std::cout << "Finished writing vtk." << std::endl;
        nOut++;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }//end for

    }

  };

}

using namespace i3d;

int main()
{
  
  InclinedPlane myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
