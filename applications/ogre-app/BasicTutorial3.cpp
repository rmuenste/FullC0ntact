/*-------------------------------------------------------------------------
This source file is a part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/


Copyright (c) 2000-2013 Torus Knot Software Ltd
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:


The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE
-------------------------------------------------------------------------*/

//! [starter]

#include <exception>
#include <iostream>

#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <OgreInput.h>
#include <OgreRTShaderSystem.h>
#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreManualObject.h>

#include "softbody.hpp"

using namespace Ogre;
using namespace OgreBites;

class TutorialApplication
        : public ApplicationContext
        , public InputListener
        , public FrameListener
{
public:
    TutorialApplication();
    virtual ~TutorialApplication();

    void setup();
    bool keyPressed(const KeyboardEvent& evt);

    bool frameRenderingQueued(const FrameEvent& evt);

    double simTime;

    ManualObject* manual;

    SceneManager* scnMgr;

    CameraMan* camMan;

    std::shared_ptr<SoftBody4> rope;

};


TutorialApplication::TutorialApplication()
    : ApplicationContext("OgreTutorialApp"), simTime(0.0)
{
}


TutorialApplication::~TutorialApplication()
{
}


void TutorialApplication::setup()
{

  //! [ropephysics]
  SpringConfiguration springConf(9, -0.2, 10.0, 8.0, 8.0);
  rope = std::make_shared<SoftBody4>(SoftBody4(springConf));
  InitSpringMesh meshInit(springConf, rope.get()->springs_, rope.get()->geom_, rope.get()->u_, rope.get()->force_, rope.get()->externalForce_);

  meshInit.init();
  std::cout << rope.get()->a0_ << std::endl;
  std::cout << rope.get()->ks_ << std::endl;
  //! [ropephysics]

  // do not forget to call the base first
  ApplicationContext::setup();
  addInputListener(this);

  // get a pointer to the already created root
  Root* root = getRoot();
  SceneManager* scnMgr = root->createSceneManager();

  // register our scene with the RTSS
  RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
  shadergen->addSceneManager(scnMgr);

  // -- tutorial section start --
  //! [cameracreate]
  SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
  Camera* cam = scnMgr->createCamera("myCam");
  //! [cameracreate]

//    //! [cameraposition]
  camNode->setPosition(0, 300, 500);
  camNode->lookAt(Vector3(0, 0, -100), Node::TransformSpace::TS_WORLD);
  //    //! [cameraposition]

      //! [cameralaststep]
  cam->setNearClipDistance(5);
  camNode->attachObject(cam);
  //! [cameralaststep]

  camMan = new CameraMan(camNode);

  // set our camera to orbit around the origin and show cursor
  //camMan->setStyle(CS_ORBIT);
  //camMan->setYawPitchDist(Degree(0), Degree(0), 250);

  //! [addviewport]
  Viewport* vp = getRenderWindow()->addViewport(cam);
  //! [addviewport]

  //! [viewportback]
  vp->setBackgroundColour(ColourValue(0, 0, 0));
  //! [viewportback]

  //! [cameraratio]
  cam->setAspectRatio(Real(vp->getActualWidth()) / Real(vp->getActualHeight()));
  //! [cameraratio]

  //! [createskydome]
  scnMgr->setSkyDome(true, "Examples/CloudySky", 10, 8);
  //! [createskydome]

  //! [plane]
  Plane plane(Vector3::UNIT_Y, 0);
  //! [plane]

  //! [planedefine]
  MeshManager::getSingleton().createPlane(
    "ground",
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    plane,
    1500, 1500, 20, 20,
    true,
    1, 5, 5,
    Vector3::UNIT_Z);
  //! [planedefine]

  //! [planecreate]
  Entity* groundEntity = scnMgr->createEntity("ground");
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(groundEntity);
  //! [planecreate]

  //! [planenoshadow]
  groundEntity->setCastShadows(false);
  //! [planenoshadow]


  //! [planesetmat]
  groundEntity->setMaterialName("Examples/BumpyMetal");
  //! [planesetmat]

  manual = scnMgr->createManualObject("manual");
  manual->setDynamic(true);

  Vector3 vstart(-100.0f, 200.0f, 0);

  int row = 9;
  int col = 9;
  float gridSize = 200.0f / float(10);

  manual->begin("BaseWhiteNoLighting", RenderOperation::OT_LINE_STRIP);

  for (int i = 0; i < rope.get()->N_-1; ++i)
  {
    std::cout << rope.get()->geom_[i].x << " " << rope.get()->geom_[i].y << " " << rope.get()->geom_[i].z << std::endl;
    manual->position(rope.get()->geom_[i]);
    manual->position(rope.get()->geom_[i+1]);
  }

  manual->end();

//  for (int j = 0; j <= row; ++j)
//  {
//    manual->begin("BaseWhiteNoLighting", RenderOperation::OT_LINE_STRIP);

//    for (int i = 0; i < col; ++i)
//    {
//      manual->position(vstart.x + i * gridSize, vstart.y - j * gridSize, 0.0);
//      manual->position(vstart.x + (i + 1) * gridSize, vstart.y - j * gridSize, 0.0);
//    }

//    manual->end();
//  }

//  for (int i = 0; i <= col; ++i)
//  {
//    manual->begin("BaseWhiteNoLighting", RenderOperation::OT_LINE_STRIP);

//    for (int j = 0; j < row; ++j)
//    {
//      manual->position(vstart.x + i * gridSize, vstart.y - j * gridSize, 0.0);
//      manual->position(vstart.x + i * gridSize, vstart.y - (j + 1) * gridSize, 0.0);
//    }

//    manual->end();
//  }

  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);

  //! [lightingsset]
  scnMgr->setAmbientLight(ColourValue(0.3, 0.3, 0.3));
  scnMgr->setShadowTechnique(ShadowTechnique::SHADOWTYPE_STENCIL_MODULATIVE);
  //! [lightingsset]

  //! [spotlight]
  Light* spotLight = scnMgr->createLight("SpotLight");
  //! [spotlight]

  //! [spotlightcolor]
  spotLight->setDiffuseColour(0, 0, 1.0);
  spotLight->setSpecularColour(0, 0, 1.0);
  //! [spotlightcolor]

  //! [spotlighttype]
  spotLight->setType(Light::LT_SPOTLIGHT);
  //! [spotlighttype]

  //! [spotlightposrot]
  SceneNode* spotLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
  spotLightNode->attachObject(spotLight);
  spotLightNode->setDirection(-1, -1, 0);
  spotLightNode->setPosition(Vector3(200, 200, 0));
  //! [spotlightposrot]

  //! [spotlightrange]
  spotLight->setSpotlightRange(Degree(35), Degree(50));
  //! [spotlightrange]

  //! [directlight]
  Light* directionalLight = scnMgr->createLight("DirectionalLight");
  directionalLight->setType(Light::LT_DIRECTIONAL);
  //! [directlight]

  //! [directlightcolor]
  directionalLight->setDiffuseColour(ColourValue(0.4, 0, 0));
  directionalLight->setSpecularColour(ColourValue(0.4, 0, 0));
  //! [directlightcolor]

  //! [directlightdir]
  SceneNode* directionalLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
  directionalLightNode->attachObject(directionalLight);
  directionalLightNode->setDirection(Vector3(0, -1, 1));
  //! [directlightdir]

  //! [pointlight]
  Light* pointLight = scnMgr->createLight("PointLight");
  pointLight->setType(Light::LT_POINT);
  //! [pointlight]

  //! [pointlightcolor]
  pointLight->setDiffuseColour(0.3, 0.3, 0.3);
  pointLight->setSpecularColour(0.3, 0.3, 0.3);
  //! [pointlightcolor]

  //! [pointlightpos]
  SceneNode* pointLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
  pointLightNode->attachObject(pointLight);
  pointLightNode->setPosition(Vector3(0, 150, 250));
  //! [pointlightpos]
  // -- tutorial section end --
}

bool TutorialApplication::frameRenderingQueued(const FrameEvent& evt)
{

  Ogre::Real dt = evt.timeSinceLastFrame;
  std::cout << "===================================================================" << std::endl;
  std::cout << "| Simulation time: " << simTime << std::endl;
  std::cout << "| Time since last frame: " << evt.timeSinceLastFrame << std::endl;
  std::cout << "===================================================================" << std::endl;
  simTime += evt.timeSinceLastFrame;

  Vector3 vstart(-100.0f, 200.0f, 0);

  int row = 9;
  int col = 9;
  float gridSize = 200.0f / float(10);

  double speed = -1.0;

  rope.get()->step(simTime, dt, 0);

  int sectionCounter = 0;

  manual->beginUpdate(sectionCounter);

  for (int i = 0; i < rope.get()->N_-1; ++i)
  {
    manual->position(rope.get()->geom_[i]);
    manual->position(rope.get()->geom_[i+1]);
  }

  manual->end();

//  for (int j = 0; j <= row; ++j)
//  {
//    manual->beginUpdate(sectionCounter);

//    for (int i = 0; i < col; ++i)
//    {
//      manual->position(vstart.x + i * gridSize, vstart.y - j * gridSize, 0.0 + simTime * speed);
//      manual->position(vstart.x + (i + 1) * gridSize, vstart.y - j * gridSize, 0.0 + simTime * speed);
//    }

//    manual->end();
//    sectionCounter++;
//  }

//  for (int i = 0; i <= col; ++i)
//  {
//    manual->beginUpdate(sectionCounter);

//    for (int j = 0; j < row; ++j)
//    {
//      manual->position(vstart.x + i * gridSize, vstart.y - j * gridSize, 0.0 + simTime * speed);
//      manual->position(vstart.x + i * gridSize, vstart.y - (j + 1) * gridSize, 0.0 + simTime * speed);
//    }

//    manual->end();
//    sectionCounter++;
//  }

  return true;
}

bool TutorialApplication::keyPressed(const KeyboardEvent& evt)
{
  if (evt.keysym.sym == SDLK_ESCAPE)
  {
    getRoot()->queueEndRendering();
  }
  return true;
}


int main(int argc, char **argv)
{
  try
  {
    TutorialApplication app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error occurred during execution: " << e.what() << '\n';
    return 1;
  }

  return 0;
}

//! [starter]
