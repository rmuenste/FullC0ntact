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


#endif /* end of include guard: BOUNDARIES_CUH_RJBOY3WK */
