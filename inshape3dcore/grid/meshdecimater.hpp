#ifndef MESHDECIMATER_HPP
#define MESHDECIMATER_HPP

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include <unstructuredgrid.h> 
#include <meshobject.h>

namespace i3d {

template<typename T>
class MeshDecimater
{
public:

    UnstructuredGrid<T,DTraits> *grid_;

    MeshObject<T, cgalKernel> *meshObj_;

	MeshDecimater(void) {

    }

	MeshDecimater(UnstructuredGrid<T,DTraits> *grid, MeshObject<T, cgalKernel> *meshObj) : grid_(grid), meshObj_(meshObj) {

    }

	virtual ~MeshDecimater(void) {

    }

	void decimate() {

        // first mark undesired hexas
        for(auto ive = 0; ive < grid_->nel_; ++ive)
        {

            const Hexa &hex = grid_->hexas_[ive];
            int cnt = 0;

            Vector3<T> mid = Vector3<T>(0,0,0);

            // first criterion for removal:
            // all vertices of the hexahedron are 
            // outside of the immersed geometry
            for(auto vidx = 0; vidx < 8; ++vidx) {
            
                int idx = hex.hexaVertexIndices_[vidx]; 

                mid += grid_->vertexCoords_[hex.hexaVertexIndices_[vidx]];
                if(grid_->m_myTraits[idx].iTag)
                    cnt++;

            }

            mid *= T(0.125);

            if(cnt)
              grid_->elemVol_[ive] = 1;
            else {
              checkDistanceCriterion(hex, mid, ive); 
            }

        }//end for  

        removeAndRenumber();

    }

    private:

        inline void checkDistanceCriterion(const Hexa &hex, const Vector3<T> &mid, int ive) {

            bool checkMore = false;
            grid_->elemVol_[ive] = 0;

            // second criterion: if the distance of the center point to 
            // the immersed geometry is less than the radius of the bs
            // of the hexahedral element then we do not remove this element
            for(auto vidx = 0; vidx < 8; ++vidx) {
                
                Vector3<T> &v = grid_->vertexCoords_[hex.hexaVertexIndices_[vidx]]; 

                T distMid = (mid - v).mag();

                if( 1.0 * distMid > grid_->elementDist_[ive]) {
                    
                    int intersections;

                    // Triangulate the hexahedron faces into two triangles
                    for (auto fidx(0); fidx < 6; ++fidx) {

                        intersections = 0;
                        int iface = hex.hexaFaceIndices_[fidx];
                        int iv0 = grid_->verticesAtFace_[iface].faceVertexIndices_[0];
                        int iv1 = grid_->verticesAtFace_[iface].faceVertexIndices_[1];
                        int iv2 = grid_->verticesAtFace_[iface].faceVertexIndices_[2];
                        int iv3 = grid_->verticesAtFace_[iface].faceVertexIndices_[3];

                        Vector3<T> v0 = grid_->vertexCoords_[iv0];
                        Vector3<T> v1 = grid_->vertexCoords_[iv1];
                        Vector3<T> v2 = grid_->vertexCoords_[iv2];
                        Vector3<T> v3 = grid_->vertexCoords_[iv3];

                        intersections += meshObj_->triangleIntersectionQuery(v0, v1, v3);
                        intersections += meshObj_->triangleIntersectionQuery(v1, v2, v3);

                        // If this face intersects the object
                        if(intersections) {
                            grid_->elemVol_[ive] = 1;
                            return;
                        }

                    }

                }

            }

        }

	    inline void removeAndRenumber() {

            auto newNEL = 0;
            auto newNVT = 0;

            std::set<int> newVertices;
            std::vector<int> newElements;
            std::vector<int> vertexIndices(grid_->nvt_, -1);

            for(auto ive = 0; ive < grid_->nel_; ++ive)
            {
                Hexa &hex = grid_->hexas_[ive];
                if(grid_->elemVol_[ive] > 0) {
                newElements.push_back(ive);
                for(auto vidx = 0; vidx < 8; ++vidx) {
                    int idx = hex.hexaVertexIndices_[vidx]; 
                    newVertices.insert(idx);
                }
                newNEL++;
                }
            }

            std::cout << "New nel: " << newNEL << std::endl;
            std::cout << "New nvt: " << newVertices.size() << std::endl;

            Vector3<T> *pVertexCoordsNew = new Vector3<T>[newVertices.size()];

            int idx = 0;
            for(auto iter = newVertices.begin(); iter != newVertices.end(); ++iter) {

                auto vidx = *iter; 
                pVertexCoordsNew[idx] = grid_->vertexCoords_[vidx];  
                vertexIndices[vidx] = idx;
                idx++;

            }

            Hexa *pHexaNew = new Hexa[newNEL];

            for(auto ive = 0; ive < newNEL; ++ive)
            {
                pHexaNew[ive] = grid_->hexas_[newElements[ive]];
            }

            
            Hexa *oldHexas = grid_->hexas_;
            grid_->hexas_ = pHexaNew;
            delete[] oldHexas;

            grid_->nel_ = newNEL;
            grid_->nvt_ = newVertices.size();

            for(auto ive = 0; ive < grid_->nel_; ++ive)
            {
                Hexa &hex = grid_->hexas_[ive];
                for(auto vidx = 0; vidx < 8; ++vidx) {
                    int idx = hex.hexaVertexIndices_[vidx]; 
                    int newIdx = vertexIndices[idx];
                    hex.hexaVertexIndices_[vidx] = newIdx;
                }
            }

            Vector3<T> *oldVertices = grid_->vertexCoords_;
            grid_->vertexCoords_ = pVertexCoordsNew;
            delete[] oldVertices;

        }

};


}

#endif