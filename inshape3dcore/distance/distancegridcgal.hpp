
#ifndef DISTANCEGRIDCGAL_HPP
#define DISTANCEGRIDCGAL_HPP

//===================================================
//                     INCLUDES
//===================================================
#include <distancemap.h>
#include <meshobject.h>

#ifdef WITH_CGAL

namespace i3d {

template<typename T>
class DistanceMapMesh
{
public:

    DistanceMap<T> *grid_;

    MeshObject<T, cgalKernel> *meshObj_;

	DistanceMapMesh(void) {

    }

	DistanceMapMesh(DistanceMap<T> *grid, MeshObject<T, cgalKernel> *meshObj) : grid_(grid), meshObj_(meshObj) {

    }

	virtual ~DistanceMapMesh(void) {

    }

	virtual void ComputeDistance() {

        auto total = grid_->getVertexCount();

        for(unsigned i=0; i < total; i++) {

            Vector3<T> vQuery = grid_->vertexCoords_[i]; 

            if (meshObj_->isPointInside(vQuery)) {
                grid_->stateFBM_[i]=1;    
            } else {
                grid_->stateFBM_[i]=0;          
            }

            std::pair<T, Vector3<T>> res = meshObj_->getClosestPoint(vQuery);

            grid_->distance_[i] = res.first; 

            if(grid_->stateFBM_[i])
                grid_->distance_[i] *= -1.0;

            grid_->normals_[i] = vQuery - res.second; 
            grid_->contactPoints_[i] = res.second;

            if(i%1000 == 0) {
                double percent = (double(i) / total) * 100.0;
                std::cout << "> Progress: " << static_cast<int>(percent) << "%" << std::flush;
                std::cout << "\r";
            }

        }

        std::cout << "> Progress: " << 100 << "%" << std::flush;
        std::cout << std::endl;

    }

};

template<typename T>
class DistanceGridMesh
{
public:
    UnstructuredGrid<T,DTraits> *grid_;

    MeshObject<T, cgalKernel> *meshObj_;

	DistanceGridMesh(void) {

    }

	DistanceGridMesh(UnstructuredGrid<T,DTraits> *grid, MeshObject<T, cgalKernel> *meshObj) : grid_(grid), meshObj_(meshObj) {

    }

	virtual ~DistanceGridMesh(void) {

    }

	void intersectAndDecimate() {

    }

	virtual void ComputeDistance() {

        auto total = grid_->nvt_;

        for(unsigned i=0; i < total; i++) {

            Vector3<T> vQuery = grid_->vertexCoords_[i]; 

            if (meshObj_->isPointInside(vQuery)) {
                grid_->m_myTraits[i].iTag=1;    
            } else {
                grid_->m_myTraits[i].iTag=0;          
            }

            std::pair<T, Vector3<T>> res = meshObj_->getClosestPoint(vQuery);

            grid_->m_myTraits[i].distance = res.first; 

            if(grid_->m_myTraits[i].iTag)
                grid_->m_myTraits[i].distance *= -1.0;

            if(i%1000 == 0) {
                double percent = (double(i) / total) * 100.0;
                std::cout << "> Vertex distance Progress: " << static_cast<int>(percent) << "%" << std::flush;
                std::cout << "\r";
            }

        }

        std::cout << "> Vertex distance Progress: " << 100 << "%" << std::flush;
        std::cout << std::endl;

    }

	virtual void ComputeElementDistance() {

        auto total = grid_->nel_;

        for(auto ive(0); ive < total; ++ive) {
            Hexa &hex = grid_->hexas_[ive];

            Vector3<T> mid = Vector3<T>(0,0,0);

            for(auto vidx(0); vidx < 8; ++vidx) {
                int idx = hex.hexaVertexIndices_[vidx];
                mid += grid_->vertexCoords_[idx];
            }

            mid *= T(0.125);

            std::pair<T, Vector3<T>> res = meshObj_->getClosestPoint(mid);

            grid_->elementDist_.push_back(res.first);

            if(ive%1000 == 0) {
                double percent = (double(ive) / total) * 100.0;
                std::cout << "> Element distance progress: " << static_cast<int>(percent) << "%" << std::flush;
                std::cout << "\r";
            }

        }

        std::cout << "> Element distance progress: " << 100 << "%" << std::flush;
        std::cout << std::endl;

    }

};

}

#endif

#endif