#include <distancemapbuilder.hpp>

namespace i3d {

template <typename T>
void DistanceMapBuilder<T>::buildDistanceMap(void)
{

    Real size = getBoundingSphereRadius();

    Real extra = 0.00;
    Real extraX = 0.00;
    Real extraY = 0.1;
    // The max size of the box domain is the size of the longest axis plus 
    // an additional 10% of the bounding sphere size 
    Real size2 = shape_->getAABB().extents_[shape_->getAABB().longestAxis()] + extra * size;

    // The size of a cell of the regular grid is 1/64 of the longest axis
    // We use this as a uniform cell size
    //Real cellSize = 2.0 * size2 / 64.0f;
    Real cellSize = 2.0 * size2 / 64.0f;
    //Real cellSize = 2.0 * size2 / 128.0f;

    // Compute the x,y,z size of the domain 
    Real _x = 2.0 * (shape_->getAABB().extents_[0] + extra * size);
    Real _y = 2.0 * (shape_->getAABB().extents_[1] + extraY * size);
    Real _z = 2.0 * (shape_->getAABB().extents_[2] + extra * size);

    // Get the center of the domain
    VECTOR3 boxCenter = shape_->getAABB().center_;

    // Cells in x, y, z
    // Select enough cells to cover the object
    int nCells[3] = {int(std::ceil(_x/cellSize)), int(std::ceil(_y/cellSize)), int(std::ceil(_z/cellSize))};

    // Update the domain size
    _x = nCells[0] * cellSize; 
    _y = nCells[1] * cellSize;
    _z = nCells[2] * cellSize;

    std::cout << "> Domain size: [" << _x << "," << _y << "," << _z << "]" << std::endl;
    std::cout << "> Domain center: " << boxCenter;
    std::cout << "> Uniform cell size: " << cellSize << std::endl;
    std::cout << "> Cells [" << nCells[0] << "," << nCells[1] << "," << nCells[2] << "]" << std::endl;

    Real ex[3] =  {0.5 * _x, 0.5 * _y, 0.5 * _z};

    AABB3r myBox(boxCenter, ex); 
    std::cout << "> Domain box vertex0: " << myBox.vertices_[0];
    std::cout << "> Domain box vertex1: " << myBox.vertices_[1];

    map_ = new DistanceMap<Real>(myBox,nCells, cellSize);

    Transformationr worldTransform = getTransformation();

    MeshObject<Real, cgalKernel> *object = dynamic_cast< MeshObject<Real, cgalKernel> *>(shape_);

    DistanceMapMesh<Real> distance(map_, object);
   
    std::cout << "Total1: " << map_->getVertexCount() << std::endl;

    distance.ComputeDistance();

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class DistanceMapBuilder<float>;
template class DistanceMapBuilder<double>;

}