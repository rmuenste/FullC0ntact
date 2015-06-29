
template <typename T>
class BVHNode
{
public:

  //the node's bounding volume
  AABB3f bv_;

  int nTriangles_;

  int *indices_;

};
