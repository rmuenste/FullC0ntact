#include <ode_rb_writer.hpp>
#include <ode_config.hpp>
#include <json.hpp>

#ifdef WITH_ODE
namespace i3d {

void ODERigidBodyWriter::write(const World& world, const std::string fileName) {

  nlohmann::json array_explicit = nlohmann::json::array();

  std::ofstream out(fileName, std::ios::out);

  if (!out) {
    std::cout << "Cannot open file: " << fileName << endl;
    std::exit(EXIT_FAILURE);
  }
  
  for (const BodyODE& b : world.bodies_) {

    const RigidBody &body = *world.rigidBodies_[b._index];

    std::string myType;
    if (b._type == "Sphere") {

      myType = std::string("Sphere");

      Real rad = Real(dGeomSphereGetRadius(b._geomId));
      Real dim = 2. * rad;

      Quaternionr quat = body.getQuaternion();

      Vec3 eulerAngles = quat.convertToEuler();
      
      const double *avel = dBodyGetAngularVel(b._bodyId);

      nlohmann::json j2 = {
        {"Type", myType},
        {"IsDynamic", "1"},
        {"Pos", {body.com_.x, body.com_.y, body.com_.z}},
        {"Rot", {quat.x, quat.y, quat.z}},
        {"Vel", {body.velocity_.x, body.velocity_.y, body.velocity_.z}},
        {"Norm", {1, 0, 0}},
        {"Dim", {dim, dim, dim}},
        {"AngVel", {avel[0], avel[1], avel[2]}}
      };

      array_explicit.push_back(j2);
    }
    else if (b._type == "TriMesh") {

      myType = std::string("TriMesh");

      Quaternionr quat = body.getQuaternion();

      Vec3 eulerAngles = quat.convertToEuler();
      
      const double *avel = dBodyGetAngularVel(b._bodyId);

      nlohmann::json j2 = {
        {"Type", myType},
        {"IsDynamic", "1"},
        {"Pos", {body.com_.x, body.com_.y, body.com_.z}},
        {"Rot", {quat.x, quat.y, quat.z}},
        {"Vel", {body.velocity_.x, body.velocity_.y, body.velocity_.z}},
        {"Norm", {1, 0, 0}},
        {"Dim", {1., 1., 1.}},
        {"AngVel", {0, 0, 0}}
      };

      array_explicit.push_back(j2);
    }
  }

  for (const BodyODE& b : world.boundaryGeometries_) {

    std::string myType = b._type;
    if (b._type == "TriMesh") {

      Model3D* modelPointer = b._meshObject.get();

      std::string meshName = modelPointer->meshFiles_[0];

      nlohmann::json j2 = {
        {"Type", myType},
        {"IsDynamic", "0"},
        {"Pos", {0.0, 0.0, 0.0}},
        {"Rot", {0.0, 0.0, 0.0}},
        {"Vel", {0.0, 0.0, 0.0}},
        {"Norm", {1, 0, 0}},
        {"Dim", {1.0, 1.0, 1.0}},
        {"MeshFile", meshName},
        {"AngVel", {0.0, 0.0, 0.0}}
      };

      array_explicit.push_back(j2);
    }
  }

  out << std::setw(4) << array_explicit << std::endl;
  out.close();
}

};
#endif