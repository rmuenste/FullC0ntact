#ifndef SPHERETREEGENERATOR_HPP_IQOGSQJE
#define SPHERETREEGENERATOR_HPP_IQOGSQJE

#include<vector>
#include<distancemap.h>
#include<sphere.h>
#include <queue>
#include <algorithm>

namespace i3d {

  template <typename T>
    class ISTQueueEntry
    {
      public:

        T dist;

        Vector3<T> voxelCenter;
        Vector3<T> nearest;

        int index;

        ISTQueueEntry() = default;

        ~ISTQueueEntry() = default;

        ISTQueueEntry(T d, int ind, const Vector3<T> &c, const Vector3<T> &cp) : dist(d), index(ind), voxelCenter(c), nearest(cp)
        {

        }

        ISTQueueEntry(T d, int ind, const Vector3<T> &c) : dist(d), index(ind), voxelCenter(c)
        {

        }

    };

  template <typename T>
    class ISTGenerator 
    {
      public:

        ISTGenerator() = default;
        virtual ~ISTGenerator() = default;

        void generate(std::vector<Sphere<T>> &spheres, DistanceMap<T,cpu> &map)
        {

          std::deque<ISTQueueEntry<T>> pqueue;
          
          for (int z(0); z < map.cells_[2]; ++z)
          {
            for (int y(0); y < map.cells_[1]; ++y)
            {
              for (int x(0); x < map.cells_[0]; ++x)
              {

                //int indices[8];
                //map.vertexIndices(x, y, z, indices);
                //Vector3<T> center(0, 0, 0);
                //int in = 1;
                //for (int k = 0; k < 8; ++k)
                //{
                //  center += map.vertexCoords_[indices[k]];
                //  in = in & map.stateFBM_[indices[k]];
                //}
                //if (in)
                //{
                //  center *= 0.125;
                //  Real rad = (map.vertexCoords_[indices[0]] - center).mag();
                //  Spherer sphere(center, rad);
                //  spheres.push_back(sphere);
                //}

                int indices[8];
                map.vertexIndices(x, y, z, indices);
                int baseIndex = z*map.dim_[1] + y*map.dim_[0] + x;
                Vector3<T> center(0, 0, 0);
                int in = 1;
                T dist = 0;

                for (int k = 0; k < 8; ++k)
                {
                  center += map.vertexCoords_[indices[k]];
                  in = in & map.stateFBM_[indices[k]];
                  dist += std::abs(map.distance_[indices[k]]);
                }
                if (in)
                {
                  // compute voxel center
                  center *= 0.125;
                  dist   *= 0.125;
                  pqueue.push_back(ISTQueueEntry<T>(dist, baseIndex, center));
                }

              }
            }
          }

          //while (!pqueue.empty()) {
          //  std::cout << pqueue.top().dist << " ";
          //  pqueue.pop();
          //}
          //std::cout << '\n';

          std::sort(pqueue.begin(), pqueue.end(),
          [](const ISTQueueEntry<T> &l, const ISTQueueEntry<T> &r) { return (l.dist > r.dist); }
            );

          //for (auto &i : pqueue)
          //{
          //  std::cout << i.dist << " ";
          //}
          //std::cout << '\n';

          while (!pqueue.empty())
          {
            ISTQueueEntry<T> e = pqueue.front();
            Sphere<T> s(e.voxelCenter, e.dist);
            spheres.push_back(s); 
            pqueue.pop_front();

            auto i = pqueue.begin();
            while (i!=pqueue.end())
            {
              if (s.isPointInside(i->voxelCenter))
              {
                i = pqueue.erase(i);
              }
              else
                ++i;
            }
            std::cout << "size: " << pqueue.size() << std::endl;
          }
          std::cout << "Number of elements in spheres: " << spheres.size() << std::endl;
          //std::exit(EXIT_FAILURE);
        }
    };

    template <typename T>
    class ISTGeneratorC
    {
    public:

      ISTGeneratorC() = default;
      virtual ~ISTGeneratorC() = default;

      void generate(std::vector<Sphere<T>> &spheres, DistanceMap<T, cpu> &map)
      {

        std::deque<ISTQueueEntry<T>> pqueue;
        std::deque<Sphere<T>> sspheres;
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
                sspheres.push_back(sphere);
              }
            }
          }
        }

        for (int z(0); z < map.cells_[2]; ++z)
        {
          for (int y(0); y < map.cells_[1]; ++y)
          {
            for (int x(0); x < map.cells_[0]; ++x)
            {

              //int indices[8];
              //map.vertexIndices(x, y, z, indices);
              //Vector3<T> center(0, 0, 0);
              //int in = 1;
              //for (int k = 0; k < 8; ++k)
              //{
              //  center += map.vertexCoords_[indices[k]];
              //  in = in & map.stateFBM_[indices[k]];
              //}
              //if (in)
              //{
              //  center *= 0.125;
              //  Real rad = (map.vertexCoords_[indices[0]] - center).mag();
              //  Spherer sphere(center, rad);
              //  spheres.push_back(sphere);
              //}

              int indices[8];
              map.vertexIndices(x, y, z, indices);
              int baseIndex = z*map.dim_[1] + y*map.dim_[0] + x;
              Vector3<T> center(0, 0, 0);
              int in = 1;
              T dist = 0;

              for (int k = 0; k < 8; ++k)
              {
                center += map.vertexCoords_[indices[k]];
                in = in & map.stateFBM_[indices[k]];
                dist += std::abs(map.distance_[indices[k]]);
              }
              if (in)
              {
                // compute voxel center
                center *= 0.125;
                dist *= 0.125;
                pqueue.push_back(ISTQueueEntry<T>(dist, baseIndex, center));
              }

            }
          }
        }

        //while (!pqueue.empty()) {
        //  std::cout << pqueue.top().dist << " ";
        //  pqueue.pop();
        //}
        //std::cout << '\n';

        std::sort(pqueue.begin(), pqueue.end(),
          [](const ISTQueueEntry<T> &l, const ISTQueueEntry<T> &r) { return (l.dist > r.dist); }
        );

        //for (auto &i : pqueue)
        //{
        //  std::cout << i.dist << " ";
        //}
        //std::cout << '\n';

        while (!pqueue.empty())
        {
          ISTQueueEntry<T> e = pqueue.front();
          Sphere<T> s(e.voxelCenter, e.dist);
          spheres.push_back(s);
          pqueue.pop_front();

          auto i = pqueue.begin();
          while (i != pqueue.end())
          {
            if (s.isPointInside(i->voxelCenter))
            {
              i = pqueue.erase(i);
            }
            else
              ++i;
          }

          auto j = sspheres.begin();
          while (j != sspheres.end())
          {
            Vector3<T> diff = (s.center_ - j->getCenter());
            T length = diff.mag();
            if (length + j->getRadius() < s.getRadius())
            {
              j = sspheres.erase(j);
            }
            else
              ++j;
          }

          std::cout << "size: " << pqueue.size() << std::endl;
        }

        for (auto i : sspheres)
        {
          spheres.push_back(i);
        }

        std::cout << "Number of elements in spheres: " << spheres.size() << std::endl;
        std::cout << "Number of elements in sspheres: " << sspheres.size() << std::endl;
        //std::exit(EXIT_FAILURE);
      }
    };

    template <typename T>
    class ISTGeneratorS
    {
    public:

      ISTGeneratorS() = default;
      virtual ~ISTGeneratorS() = default;

      void generate(std::vector<Sphere<T>> &spheres, DistanceMap<T, cpu> &map)
      {

        std::deque<ISTQueueEntry<T>> pqueue;

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

    template <typename T>
    class ISTGeneratorV
    {
    public:

      ISTGeneratorV() = default;
      virtual ~ISTGeneratorV() = default;

      void generate(std::vector<Sphere<T>> &spheres, DistanceMap<T, cpu> &map)
      {

        int size = map.dim_[0] * map.dim_[1];

        std::deque<ISTQueueEntry<T>> pqueue;

        for (int i(0); i < size; ++i)
        {

          int baseIndex = i;

          Vector3<T> center   = map.vertexCoords_[i];
          int in              = map.stateFBM_[i];
          T dist              = std::abs(map.distance_[i]);

          if (in)
          {
            pqueue.push_back(ISTQueueEntry<T>(dist, baseIndex, center, map.contactPoints_[i]));

          }

        }

        std::sort(pqueue.begin(), pqueue.end(),
          [](const ISTQueueEntry<T> &l, const ISTQueueEntry<T> &r) { return (l.dist > r.dist); }
        );

        //for (auto &i : pqueue)
        //{
        //  std::cout << i.dist << " ";
        //}
        //std::cout << '\n';

        while (!pqueue.empty())
        {
          ISTQueueEntry<T> e = pqueue.front();
          Sphere<T> s(e.voxelCenter, e.dist);
          spheres.push_back(s);
          //std::cout << "ID: " << e.index << std::endl;
          //std::cout << "dist: " << e.dist << std::endl;
          //std::cout << "center: " << e.voxelCenter << std::endl;
          //std::cout << "point: " << e.nearest << std::endl;
          //pqueue.pop_front();
          //break;
          auto i = pqueue.begin();
          while (i != pqueue.end())
          {
            //Vector3<T> diff = (s.center_ - i->voxelCenter);
            //s.d
            //T mydist = diff * diff <=  radius_ * radius_);

            if (s.isPointInside(i->voxelCenter))
            {
              i = pqueue.erase(i);
            }
            else
              ++i;
          }
          std::cout << "size: " << pqueue.size() << std::endl;
        }
        std::cout << "Number of elements in spheres: " << spheres.size() << std::endl;

        //std::exit(EXIT_FAILURE);
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
