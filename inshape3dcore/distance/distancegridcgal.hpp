
#ifndef DISTANCEGRIDCGAL_HPP
#define DISTANCEGRIDCGAL_HPP

//===================================================
//                     INCLUDES
//===================================================
#include <distancemap.h>
#include <meshobject.h>

namespace i3d {


template<typename T>
class DistanceMapMesh
{
public:

    //std::shared_ptr<DistanceMap<T>> grid_;
    DistanceMap<T> *grid_;

    MeshObject<T, cgalKernel> *meshObj_;

	DistanceMapMesh(void) {

    }

	DistanceMapMesh(DistanceMap<T> *grid, MeshObject<T, cgalKernel> *meshObj) : grid_(grid), meshObj_(meshObj) {

    }

	virtual ~DistanceMapMesh(void) {

    }

	virtual void ComputeDistance() {

        //auto total = grid_->getVertexCount();

        int total = (grid_->cells_[0]+1)*(grid_->cells_[1]+1)*(grid_->cells_[2]+1);

        for(int i=0;i<total;i++) {

            Vec3 vQuery=grid_->vertexCoords_[i];

//        for(unsigned i = 0; i < total; ++i) {
//
//            Vector3<T> vQuery = grid_->vertexCoords_[i]; 

            if (meshObj_->isPointInside(vQuery)) {
                grid_->stateFBM_[i]=1;    
            } else {
                grid_->stateFBM_[i]=0;          
            }

//            std::pair<T, Vector3<T>> res = meshObj_->getClosestPoint(vQuery);
//
//            grid_->distance_[i] = res.first; 
//
//            if(grid_->stateFBM_[i])
//                grid_->distance_[i] *= -1.0;
//
//            grid_->normals_[i] = vQuery - res.second; 
//            grid_->contactPoints_[i] = res.second;

            if(i%1000 == 0) {
                double percent = (double(i) / total) * 100.0;
                std::cout << "> Progress: " << static_cast<int>(percent) << "%" << std::flush;
                std::cout << "\r";
            }

            i++;
            
        }

        std::cout << "> Progress: " << 100 << "%" << std::flush;
        std::cout << std::endl;

    }

};




}

#endif