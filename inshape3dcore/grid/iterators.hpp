#ifndef ITERATORS_HPP_QNTY5KOL
#define ITERATORS_HPP_QNTY5KOL

#include <cstddef>
#include <vector3.h>
#include <cstring>
#include <Eigen/Sparse>
#include <Eigen/Dense>

namespace i3d {

  class DeformationAxis {

    public:
      
    Vec3 axis_;

    Vec3 intersecPoints_[2];

    std::pair<Real, Real> coeffs_[2];

    int faceIdx[2];

    void toString() {

      std::cout << axis_;
      std::cout << intersecPoints_[0];
      std::cout << intersecPoints_[1];
      std::cout << coeffs_[0].first << ";" << coeffs_[0].second << std::endl;
      std::cout << coeffs_[1].first << ";" << coeffs_[1].second << std::endl;
      std::cout << faceIdx[0] << std::endl;
      std::cout << faceIdx[1] << std::endl;

    }

  };

  /**
  * @brief Class that represents a hexahedron
  *
  */
  class Hexa
  {
  public:
    typedef Eigen::Matrix<Real, 8, 6> MatrixXd;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Hexa()
    {
      std::memset(hexaNeighborIndices_, -1, 6 * sizeof(int));
      memset(hexaFaceIndices_, -1, 6 * sizeof(int));
      memset(hexaVertexIndices_, -1, 8 * sizeof(int));
      memset(hexaEdgeIndices_, -1, 12 * sizeof(int));
    };

    Hexa& operator=(const Hexa &hex)
    {
      memcpy(hexaVertexIndices_, hex.hexaVertexIndices_, 8 * sizeof(int));
      memcpy(hexaEdgeIndices_, hex.hexaEdgeIndices_, 12 * sizeof(int));
      memcpy(hexaFaceIndices_, hex.hexaFaceIndices_, 6 * sizeof(int));
      memcpy(hexaNeighborIndices_, hex.hexaNeighborIndices_, 6 * sizeof(int));
      axes_ = hex.axes_;
      return *this;
    };

    Hexa(const Hexa &hex)
    {
      memcpy(hexaVertexIndices_, hex.hexaVertexIndices_, 8 * sizeof(int));
      memcpy(hexaEdgeIndices_, hex.hexaEdgeIndices_, 12 * sizeof(int));
      memcpy(hexaFaceIndices_, hex.hexaFaceIndices_, 6 * sizeof(int));
      memcpy(hexaNeighborIndices_, hex.hexaNeighborIndices_, 6 * sizeof(int));
      axes_ = hex.axes_;
    };

    /**
    * The array maps the indices of the 8 local nodes to the
    * the global vertex array, so that the vertex coordinates
    * can be accessed.
    **/
    int hexaVertexIndices_[8];
    /**
    * The array maps the indices of the vertices of the local 12 edges
    * to the global vertex array, so that the coordinates of the edge
    * vertices can be accessed
    */
    int hexaEdgeIndices_[12];
    /**
    * The array maps the local vertex indices of the 6 faces to
    * the global vertex array, so that the coordinates of the vertices
    * can be accessed
    */
    int hexaFaceIndices_[6];
    /**
    * The array stores the element id of the neighbouring hexas
    * at the 6 faces of the hexa. If there is no neighbour at a
    * face the array stores a -1 at this position.
    */
    int hexaNeighborIndices_[6];

    Eigen::SparseMatrix<Real> shapeMatrix;
    MatrixXd denseMatrix;

    std::vector<DeformationAxis> axes_;

    std::vector<Real> evalShapeFuncFace(int faceIdx, Real xi, Real eta) {

      int facesHex[6][4]=
      {
        {0,1,2,3},
        {0,4,5,1},
        {1,5,6,2},
        {2,6,7,3},
        {0,3,7,4},
        {4,7,6,5}
      };

      std::vector<Real> values;

      for(int i = 0; i < 8; ++i)
        values.push_back(0.0);

      values[facesHex[faceIdx][0]] = (1.0 - xi) * (1.0 - eta); 
      values[facesHex[faceIdx][1]] = xi * (1.0 - eta); 
      values[facesHex[faceIdx][2]] = xi * eta; 
      values[facesHex[faceIdx][3]] = (1.0 - xi) * eta; 

      return values;

    }

  };

  /**
  * @brief Class that an edge between to points in 3d
  *
  */
  class HexaEdge
  {
  public:
    int edgeVertexIndices_[2];
  };

  /**
  * @brief Class that represents a quadrilateral face in 3d
  *
  */
  class HexaFace
  {
  public:
    int faceVertexIndices_[4];
  };

  class VertexVertex
  {
  public:
    VertexVertex()
    {
      std::memset(m_iVertInd,-1,6*sizeof(int));
      m_iNeighbors = 0;
    };

    int m_iVertInd[6];
    int m_iNeighbors;
  };

  //VertexVertexIterator
  /**
  * @brief An VertexVertexIter iterates over the vertices attached to a vertex
  *
  */          
  class VertexVertexIter
  {
    public:

      typedef int  value_type;
      typedef int* pointer;
      typedef int& reference;
      VertexVertexIter(int curpos = 0,VertexVertex *_vv=nullptr, int iVert=0) : _curpos(curpos),vv_(_vv),m_iVert(iVert) {};

      reference operator*() {return  vv_->m_iVertInd[_curpos];};

      int idx() {return vv_->m_iVertInd[_curpos];};

      VertexVertexIter& operator++()
      {
        _curpos=_curpos+1;
        return *this;
      }

      VertexVertexIter operator++(int)
      {
        VertexVertexIter old_it = *this;
        ++(*this);
        return old_it;
      }

      bool operator !=(VertexVertexIter rhs){return _curpos!=rhs._curpos;};

    protected:
      int _curpos;
      VertexVertex *vv_;
      int m_iVert;
  };

  class HFaceIter
  {
  public:

    typedef HexaFace  value_type;
    typedef HexaFace* pointer;
    typedef HexaFace& reference;
    //Contructor initializes the edgeiter with NULL
    HFaceIter(HexaFace* curpos = nullptr, int ipos = 0) : _curpos(curpos), _ipos(ipos) {};

    reference operator*() { return *_curpos; }

    pointer Get() { return _curpos; }

    int idx() { return _ipos; };

    HFaceIter& operator++()
    {
      _curpos = _curpos + 1;
      _ipos++;
      return *this;
    }

    HFaceIter operator++(int)
    {
      HFaceIter old_it = *this;
      ++(*this);
      return old_it;
    }

    bool operator !=(HFaceIter rhs) { return _curpos != rhs._curpos; };

  protected:
    HexaFace* _curpos;
    int    _ipos;
  };


  //----------------------------------------------------------------------------
  //Class VertexIter: the vertex iterator iterates over all vertices of the mesh
  /**
  * @brief A VertexIter iterates over vertices of a CUnstructuredGrid
  *
  */
  template<typename T>
  class VertexIter
  {
  public:

    typedef Vector3<T>  value_type;
    typedef Vector3<T>* pointer;
    typedef Vector3<T>& reference;
    VertexIter(Vector3<T>* curpos = NULL, int ipos = 0) : _curpos(curpos), _pos(ipos) {};

    reference operator*() { return *_curpos; }

    pointer Get() { return _curpos; }

    int idx() { return _pos; };

    VertexIter& operator++()
    {
      _curpos = _curpos + 1;
      _pos++;
      return *this;
    }

    VertexIter operator++(int)
    {
      VertexIter old_it = *this;
      ++(*this);
      return old_it;
    }

    bool operator !=(VertexIter rhs) { return _curpos != rhs._curpos; };

  protected:
    Vector3<T>* _curpos;
    int _pos;
  };

  /**
  * @brief An ElementIter iterates over elements of a grid
  *
  */
  class ElementIter
  {
  public:

    typedef Hexa  value_type;
    typedef Hexa* pointer;
    typedef Hexa& reference;
    ElementIter(Hexa* curpos = nullptr, int ipos = 0) : _curpos(curpos), _ipos(ipos) {};

    reference operator*() { return *_curpos; }

    ElementIter& operator++()
    {
      _curpos = _curpos + 1;
      _ipos++;
      return *this;
    }

    pointer Get() { return _curpos; };

    int idx() { return _ipos; };

    ElementIter operator++(int)
    {
      ElementIter old_it = *this;
      ++(*this);
      return old_it;
    }

    bool operator !=(ElementIter rhs) { return _curpos != rhs._curpos; };



  protected:
    Hexa* _curpos;
    int    _ipos;
  };

  //----------------------------------------------------------------------------
  /**
  * @brief An EdgeIter iterates over the edges of a CUnstrucuredGrid
  *
  */
  class EdgeIter
  {
  public:

    typedef HexaEdge  value_type;
    typedef HexaEdge* pointer;
    typedef HexaEdge& reference;
    //Contructor initializes the edgeiter with NULL
    EdgeIter(HexaEdge* curpos = nullptr, int ipos = 0) : _curpos(curpos), _ipos(ipos) {};

    reference operator*() { return *_curpos; }

    pointer Get() { return _curpos; }

    int idx() { return _ipos; };

    EdgeIter& operator++()
    {
      _curpos = _curpos + 1;
      _ipos++;
      return *this;
    }

    EdgeIter operator++(int)
    {
      EdgeIter old_it = *this;
      ++(*this);
      return old_it;
    }

    bool operator !=(EdgeIter rhs) { return _curpos != rhs._curpos; };

  protected:
    HexaEdge* _curpos;
    int    _ipos;
  };



};

#endif /* end of include guard: ITERATORS_HPP_QNTY5KOL */
