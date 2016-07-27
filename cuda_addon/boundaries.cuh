#ifndef BOUNDARIES_CUH_RJBOY3WK
#define BOUNDARIES_CUH_RJBOY3WK

__device__ void d_checkBoundaryBox(float3 &pos, float3 &vel, ParticleWorld<float, unified> *pw)
{

  if (pos.x >  1.0f - pw->params_->particleRadius_) { pos.x =  1.0f - pw->params_->particleRadius_; vel.x *= pw->params_->boundaryDamping_; }
  if (pos.x < -1.0f + pw->params_->particleRadius_) { pos.x = -1.0f + pw->params_->particleRadius_; vel.x *= pw->params_->boundaryDamping_; }
  if (pos.y >  1.0f - pw->params_->particleRadius_) { pos.y =  1.0f - pw->params_->particleRadius_; vel.y *= pw->params_->boundaryDamping_; }
  if (pos.z >  1.0f - pw->params_->particleRadius_) { pos.z =  1.0f - pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.z < -1.0f + pw->params_->particleRadius_) { pos.z = -1.0f + pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.y < -1.0f + pw->params_->particleRadius_) { pos.y = -1.0f + pw->params_->particleRadius_; vel.y *= pw->params_->boundaryDamping_; }

}

__device__ void d_checkBoundaryCyl(float3 &pos, float3 &vel, ParticleWorld<float, unified> *pw)
{

  float3 p = make_float3(pos.x,pos.y, 0.0f); 
  float len = sqrtf(dot(p, p));
  float rad_cyl = 1.0f;

  if(len > rad_cyl)
  {
    float penetration = len - rad_cyl;
    float3 n = normalize(p);
    pos = pos - (penetration + pw->params_->particleRadius_)* n;
    vel.x *= pw->params_->boundaryDamping_; 
    vel.y *= pw->params_->boundaryDamping_;

  }

  if (pos.z >  1.0f - pw->params_->particleRadius_) { pos.z =  1.0f - pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.z < -1.0f + pw->params_->particleRadius_) { pos.z = -1.0f + pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }

}

__device__ void checkDistMap(float3 &pos, float3 &vel, 
                             ParticleWorld<float, unified> *pw,
                             DistanceMap<float,gpu> *map)
{

    vector3 query(pos.x, pos.y, pos.z);
    vector3 cp(0,0,0);
    vector3 normal(0,0,0);
    float dist=0.0;
    map->queryMap(query,dist,cp,normal);
    float rad_cyl = pw->params_->particleRadius_;

    if(dist < rad_cyl)
    {
      float3 cp0 = make_float3(cp.x, cp.y, cp.z);
      float3 n0  = pos - cp0;
      float3 n = normalize(n0);
      float penetration = rad_cyl - dist;
      pos = pos + (penetration * n) + (0.1 * rad_cyl * n);
      vel.x *=  0.5f * pw->params_->boundaryDamping_; 
      vel.y *=  0.5f * pw->params_->boundaryDamping_;
      vel.z *=  0.7f * pw->params_->boundaryDamping_;

    }

}

__device__ void d_checkBoundaryDistMap(float3 &pos, float3 &vel, 
                                       ParticleWorld<float, unified> *pw,
                                       DistanceMap<float,gpu> *map_boundary)
{

  float3 p = make_float3(pos.x,pos.y, 0.0f); 
  float len = sqrtf(dot(p, p));
  float rad_cyl = 1.0f;

  checkDistMap(pos, vel, pw, map_boundary);

  if(len > rad_cyl)
  {
    float penetration = len - rad_cyl;
    float3 n = normalize(p);
    pos = pos - (penetration + pw->params_->particleRadius_)* n;
    vel.x *= pw->params_->boundaryDamping_; 
    vel.y *= pw->params_->boundaryDamping_;

  }

  if (pos.z >  10.0f - pw->params_->particleRadius_) { pos.z =  10.0f - pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }
  if (pos.z < -1.0f + pw->params_->particleRadius_) { pos.z = -1.0f + pw->params_->particleRadius_; vel.z *= pw->params_->boundaryDamping_; }

}


#endif /* end of include guard: BOUNDARIES_CUH_RJBOY3WK */
