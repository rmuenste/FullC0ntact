#ifndef DISTANCE_CUH_ERLAQEVZ
#define DISTANCE_CUH_ERLAQEVZ

__device__ float distance_tri(const vector3 &query, triangle *tri, vector3 *vertices)
{
  float da, db, dc, dd, de, ds, dt;
  float ddenom, dnum;

  vector3 e0, e1, vClosestPoint;

  vector3 &v0 = vertices[tri->idx0];
  vector3 &v1 = vertices[tri->idx1];
  vector3 &v2 = vertices[tri->idx2];

  e0 = v1 - v0;
  e1 = v2 - v0;

  da = e0 * e0;
  db = e0 * e1;
  dc = e1 * e1;
  dd = e0 * (v0 - query);
  de = e1 * (v0 - query);

  ddenom = da*dc - db*db;
  ds = db*de - dc*dd;
  dt = db*dd - da*de;

  if (ds + dt <= ddenom)
  {
    if (ds < 0)
    {
      if (dt < 0)
      {
        if (de < 0.0)
        {
          ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : dd / da));
          dt = 0;
        }
        else
        {
          ds = 0;
          dt = (de >= 0 ? 0 : (-de >= dc ? 1 : -de / dc));
        }
      }
      //Region 3
      else
      {
        ds = 0;
        dt = (de >= 0 ? 0 : (-de >= dc ? 1 : -de / dc));
      }
    }
    //Region 5
    else if (dt < 0)
    {
      ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : -dd / da));
      dt = 0;
    }
    //Region 0
    else
    {
      float invDenom = float(1.0) / ddenom;
      ds *= invDenom;
      dt *= invDenom;
    }
  }
  else
  {
    //Region 2
    if (ds < 0)
    {
      float tmp0 = db + dd;
      float tmp1 = dc + de;
      if (tmp1 > tmp0)
      {
        dnum = tmp1 - tmp0;
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1 : dnum / ddenom);
        dt = 1 - ds;
      }
      else
      {
        ds = 0;
        dt = (tmp1 <= 0 ? 1 : (de >= 0 ? 0 : -de / dc));
      }
    }
    //Region 6
    else if (dt < 0)
    {
      float tmp0 = -da - dd;
      float tmp1 = db + de;
      float tmp2 = db + dd;
      float tmp3 = dc + de;
      if (tmp1 > tmp0)
      {
        dnum = tmp3 - tmp2;
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1 : dnum / ddenom);
        dt = 1 - ds;
      }
      else
      {
        ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : dd / da));
        dt = 0;
      }
    }
    // Region 1
    else
    {
      dnum = (dc + de - db - dd);
      if (dnum <= 0)
      {
        ds = 0.0;
      }
      else
      {
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1.0 : dnum / ddenom);
      }
      dt = 1 - ds;
    }
  }


  vector3 closestPoint = v0 + e0*ds + e1*dt;

  return (query - closestPoint).norm2();

}//end distance_tri

template <typename T>
__device__ T distance_triangle(const Vector3<T> &query, const Vector3<T> &v0, const Vector3<T> &v1, const Vector3<T> &v2)
{
  T da, db, dc, dd, de, ds, dt;
  T ddenom, dnum;

  Vector3<T> e0, e1, vClosestPoint;

  e0 = v1 - v0;
  e1 = v2 - v0;

  da = e0 * e0;
  db = e0 * e1;
  dc = e1 * e1;
  dd = e0 * (v0 - query);
  de = e1 * (v0 - query);

  ddenom = da*dc - db*db;
  ds = db*de - dc*dd;
  dt = db*dd - da*de;

  if (ds + dt <= ddenom)
  {
    if (ds < 0)
    {
      if (dt < 0)
      {
        if (de < 0.0)
        {
          ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : dd / da));
          dt = 0;
        }
        else
        {
          ds = 0;
          dt = (de >= 0 ? 0 : (-de >= dc ? 1 : -de / dc));
        }
      }
      //Region 3
      else
      {
        ds = 0;
        dt = (de >= 0 ? 0 : (-de >= dc ? 1 : -de / dc));
      }
    }
    //Region 5
    else if (dt < 0)
    {
      ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : -dd / da));
      dt = 0;
    }
    //Region 0
    else
    {
      T invDenom = T(1.0) / ddenom;
      ds *= invDenom;
      dt *= invDenom;
    }
  }
  else
  {
    //Region 2
    if (ds < 0)
    {
      T tmp0 = db + dd;
      T tmp1 = dc + de;
      if (tmp1 > tmp0)
      {
        dnum = tmp1 - tmp0;
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1 : dnum / ddenom);
        dt = 1 - ds;
      }
      else
      {
        ds = 0;
        dt = (tmp1 <= 0 ? 1 : (de >= 0 ? 0 : -de / dc));
      }
    }
    //Region 6
    else if (dt < 0)
    {
      T tmp0 = -da - dd;
      T tmp1 =  db + de;
      T tmp2 =  db + dd;
      T tmp3 =  dc + de;
      if (tmp1 > tmp0)
      {
        dnum = tmp3 - tmp2;
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1 : dnum / ddenom);
        dt = 1 - ds;
      }
      else
      {
        ds = (dd >= 0 ? 0 : (-dd >= da ? 1 : dd / da));
        dt = 0;
      }
    }
    // Region 1
    else
    {
      dnum = (dc + de - db - dd);
      if (dnum <= 0)
      {
        ds = 0.0;
      }
      else
      {
        ddenom = da - 2 * db + dc;
        ds = (dnum >= ddenom ? 1.0 : dnum / ddenom);
      }
      dt = 1 - ds;
    }
  }


  Vector3<T> closestPoint = v0 + e0*ds + e1*dt;

  return (query - closestPoint).norm2();

}//end distance

#endif /* end of include guard: DISTANCE_CUH_ERLAQEVZ */
