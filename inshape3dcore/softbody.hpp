#ifndef SOFTBODY_HPP_NRX1JCK5
#define SOFTBODY_HPP_NRX1JCK5

namespace i3d
{

  template <typename T> 
  class BasicSoftBody
  {
  public:
    BasicSoftBody ();

    virtual ~BasicSoftBody ();

    virtual void internalForce(T dt); 

    virtual void applyForce(T dt); 
  
  private:
    /* data */
  };

  template<typename T, typename Geometry>
  class SoftBody : public BasicSoftBody<T>
  {
  public:
    SoftBody ();

    virtual ~SoftBody ();
  
  private:
    /* data */
  };
  
} /* i3d */ 


#endif /* end of include guard: SOFTBODY_HPP_NRX1JCK5 */
