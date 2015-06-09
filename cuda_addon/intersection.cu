__device__ bool intersection(const vector3 &orig, const vector3 &dir, triangle *tri, vector3* vertices)
{

  // compute the offset origin, edges, and normal
  vector3 &v0 = vertices[tri->idx0];
  vector3 &v1 = vertices[tri->idx1];
  vector3 &v2 = vertices[tri->idx2];

  vector3 kDiff = orig - v0;
  vector3 kEdge1 = v1 - v0;
  vector3 kEdge2 = v2 - v0;
  vector3 kNormal = vector3::Cross(kEdge1, kEdge2);

  // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
  // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
  //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
  //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
  //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
  real fDdN = dir * kNormal;
  real fSign;
  if (fDdN > 0.0000005)
  {
    fSign = 1.0;
  }
  else if (fDdN < 0.0000005)
  {
    fSign = -1.0f;
    fDdN = -fDdN;
  }
  else
  {
    // Ray and triangle are parallel, call it a "no intersection"
    // even if the ray does intersect.
    return false;
  }

  real fDdQxE2 = (dir * vector3::Cross(kDiff, kEdge2)) * fSign;

  if (fDdQxE2 >= 0.0)
  {
    Real fDdE1xQ = (dir * vector3::Cross(kEdge1, kDiff)) * fSign;
    if (fDdE1xQ >= 0.0)
    {
      if (fDdQxE2 + fDdE1xQ <= fDdN)
      {
        // line intersects triangle, check if ray does
        Real fQdN = (kDiff * kNormal) * -fSign;
        if (fQdN >= 0.0)
        {
          // ray intersects triangle
          return true;
        }
        // else: t < 0, no intersection
      }
      // else: b1+b2 > 1, no intersection
    }
    // else: b2 < 0, no intersection
  }
  // else: b1 < 0, no intersection

  return false;

}//end Intersection


//#define TESTING
//#define DEBUG_IDX 38057
//#define DEBUG_IVT 65618
__device__ bool intersection_tri(const vector3 &orig, const vector3 &dir, const vector3 &v0, const vector3 &v1, const vector3 &v2, int idx)
{

  vector3 kDiff = orig - v0;
  vector3 kEdge1 = v1 - v0;
  vector3 kEdge2 = v2 - v0;
  vector3 kNormal = vector3::Cross(kEdge1, kEdge2);

  // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
  // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
  //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
  //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
  //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
  float fDdN = dir * kNormal;
  float fSign;
  if (fDdN > FLT_EPSILON)
  {
    fSign = 1.0;
  }
  else if (fDdN < -FLT_EPSILON)
  {
    fSign = -1.0f;
    fDdN = -fDdN;
  }
  else
  {
    // Ray and triangle are parallel, call it a "no intersection"
    // even if the ray does intersect.
#ifdef TESTING
    if (idx == DEBUG_IDX)
      printf("parallel\n");
#endif
    return false;
  }

  float fDdQxE2 = (dir * vector3::Cross(kDiff, kEdge2)) * fSign;

  if (fDdQxE2 >= DBL_EPSILON)// FLT_EPSILON) //FLT_EPSILON    
  {
    Real fDdE1xQ = (dir * vector3::Cross(kEdge1, kDiff)) * fSign;
    if (fDdE1xQ >= 0.0f)
    {
      if (fDdQxE2 + fDdE1xQ <= fDdN)
      {
#ifdef TESTING
        if (idx == DEBUG_IDX)
          printf("fDdQxE2( = %f) + fDdE1xQ( = %f) <= fDdN + 0.000001f( = %f)\n", fDdQxE2, fDdE1xQ, fDdN + 0.000001f);
#endif
        // line intersects triangle, check if ray does
        Real fQdN = (kDiff * kNormal) * -fSign;
        if (fQdN >= 0.0f)
        {
          // ray intersects triangle
          return true;
        }
        // else: t < 0, no intersection
#ifdef TESTING
        else
        {
          if (idx == DEBUG_IDX)
            printf("t < 0\n");
        }
#endif
      }
      // else: b1+b2 > 1, no intersection
#ifdef TESTING
      else
      {
        if (idx == DEBUG_IDX)
        {
          printf("b1+b2 > 1\n");
          printf("%f + %f <= %f\n", fDdQxE2, fDdE1xQ, fDdN);
        }
      }
#endif
    }
    // else: b2 < 0, no intersection
#ifdef TESTING
    else
    {
      if (idx == DEBUG_IDX)
        printf("b2 < 0\n");
    }
#endif
  }
  // else: b1 < 0, no intersection
#ifdef TESTING
  else
  {
    if (idx == DEBUG_IDX)
    {
      printf("b1 < 0\n");
      printf("b1 = %6.18f\n", fDdQxE2);
    }
  }
#endif
  return false;

}//end Intersection