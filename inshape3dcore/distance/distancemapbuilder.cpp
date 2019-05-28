#include <distancemapbuilder.hpp>
#include <distancegridcgal.hpp>

namespace i3d {

template <typename T>
void DistanceMapBuilder<T>::buildDistanceMap(void)
{

    Real size = body_->getBoundingSphereRadius();

    Real extraX = extraDomainSize_[0];
    Real extraY = extraDomainSize_[1];
    Real extra =  extraDomainSize_[2];
    // The max size of the box domain is the size of the longest axis plus 
    // an additional 10% of the bounding sphere size 
    //Real size2 = body_->shape_->getAABB().extents_[body_->shape_->getAABB().longestAxis()] + extra * size;
    Real size2 = body_->shape_->getAABB().extents_[2];

    // The size of a cell of the regular grid is 1/64 of the longest axis
    // We use this as a uniform cell size
    //Real cellSize = 2.0 * size2 / 64.0f;
    Real cellSize = 2.0 * size2 / 64.0f;
    //Real cellSize = 2.0 * size2 / 128.0f;

    //Real cellSizeArray[3] =  {1.0 * cellSize, 1.0 * cellSize, 1.0 * cellSize};
    Real cellSizeArray[3] = {cellSizeArray_[0], cellSizeArray_[1], cellSizeArray_[2]};

    // Get the center of the domain
    boxCenter = body_->shape_->getAABB().center_;

    // Extend of the bounding box from the center
    _x = body_->shape_->getAABB().extents_[0];
    _y = body_->shape_->getAABB().extents_[1];
    _z = body_->shape_->getAABB().extents_[2];

    modifyBoundingBoxUniform(0, extraX * size);
    modifyBoundingBoxUniform(1, extraY * size);
    modifyBoundingBoxUniform(2, extra * size);

    // -x
    //modifyBoundingBoxNonUniform(2, -40.0);
    //boxCenter.z += 40.0;

    // Compute the x,y,z size of the domain 
    _x *= 2.0;
    _y *= 2.0;
    _z *= 2.0;

    // Cells in x, y, z
    // Select enough cells to cover the object
    int nCells[3] = {int(std::ceil(_x/cellSizeArray[0])), int(std::ceil(_y/cellSizeArray[1])), int(std::ceil(_z/cellSizeArray[2]))};

    // Update the domain size
    _x = nCells[0] * cellSizeArray[0]; 
    _y = nCells[1] * cellSizeArray[1];
    _z = nCells[2] * cellSizeArray[2];

    std::cout << "> Domain size: [" << _x << "," << _y << "," << _z << "]" << std::endl;
    std::cout << "> Domain center: " << boxCenter;
    std::cout << "> Uniform cell size: " << cellSize << std::endl;
    std::cout << "> Cells [" << nCells[0] << "," << nCells[1] << "," << nCells[2] << "]" << std::endl;
    std::cout << "size2: " << size2 << std::endl;

    Real ex[3] =  {0.5 * _x, 0.5 * _y, 0.5 * _z};

    AABB3r myBox(boxCenter, ex); 

//    Vec3 newMinVertex(myBox.vertices_[0]);
//    newMinVertex.y = myBox.vertices_[0].y - 0.1 * size;
//
//	AABB3r newBoundingBox = AABB3<Real>(newMinVertex, myBox.vertices_[1]);

    std::cout << "> Domain box vertex0: " << myBox.vertices_[0];
    std::cout << "> Domain box vertex1: " << myBox.vertices_[1];

    //body_->map_ = new DistanceMap<Real>(newBoundingBox,nCells, cellSize);
    body_->map_ = new DistanceMap<Real>(myBox,nCells, cellSizeArray);

    Transformationr worldTransform = body_->getTransformation();

#ifdef WITH_CGAL
    MeshObject<Real, cgalKernel> *object = dynamic_cast< MeshObject<Real, cgalKernel> *>(body_->shape_);

    DistanceMapMesh<Real> distance(body_->map_, object);
   
    std::cout << "Total1: " << body_->map_->getVertexCount() << std::endl;

    distance.ComputeDistance();
#endif

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class DistanceMapBuilder<float>;
template class DistanceMapBuilder<double>;

}