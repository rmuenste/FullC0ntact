#ifndef SPHERETREEGENERATOR_HPP_IQOGSQJE
#define SPHERETREEGENERATOR_HPP_IQOGSQJE

#include<vector>
#include<distancemap.h>
#include<sphere.h>
#include <queue>

namespace i3d {


  template <typename T>
    class ISTGenerator 
    {
      public:

        ISTGenerator() = default;
        virtual ~ISTGenerator() = default;

        void generate(std::vector<Sphere<T>> &spheres, DistanceMap<T,cpu> &map)
        {
          
          for (int z(0); z < map.cells_[2]; ++z)
          {
            for (int y(0); y < map.cells_[1]; ++y)
            {
              for (int x(0); x < map.cells_[0]; ++x)
              {
                int indices[8];
                map.vertexIndices(x, y, z, indices);
                Vector3<T> center(0, 0, 0);
                int in = 1;
                for (int k = 0; k < 8; ++k)
                {
                  center += map.vertexCoords_[indices[k]];
                  in = in & map.stateFBM_[indices[k]];
                }
                if (in)
                {
                  center *= 0.125;
                  Real rad = (map.vertexCoords_[indices[0]] - center).mag();
                  Spherer sphere(center, rad);
                  spheres.push_back(sphere);
                }
              }
            }
          }
        }
    };

  template <typename T, class Generator>
    class SphereTreeGenerator
    {
      public:

        Generator gen_;

        DistanceMap<T,0> *map_;

        std::vector<Sphere<T>> *spheres_;

        SphereTreeGenerator() = default;

        SphereTreeGenerator(DistanceMap<T,cpu> *map, std::vector<Sphere<T>> *spheres) :
          spheres_(spheres), map_(map)
        {
           
        }

        virtual ~SphereTreeGenerator() = default; 

        void generateIST()
        {
          std::vector<Sphere<T>> &spheres = *spheres_;
          DistanceMap<T,cpu> &map = *map_;
          gen_.generate(spheres, map);
        }  

    };


}


#endif /* end of include guard: SPHERETREEGENERATOR_HPP_IQOGSQJE */
