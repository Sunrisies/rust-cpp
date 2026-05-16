#include "../CoverMapObsPlan.h"
#include "../CoverMapObsPlan_initialize.h"
#include "../CoverMapObsPlan_terminate.h"
#include "../coverMapbyYaw.h"
#include "../circleAvoidRTL.h"
#include "../edgeCollision.h"
#include "../mainShrOut.h"
#include "../pygObsCollision.h"
#include "../pygObsAvoid.h"

extern "C" void cover_map_obs_plan_initialize_shim()
{
  CoverMapObsPlan_initialize();
}

extern "C" void cover_map_obs_plan_terminate_shim()
{
  CoverMapObsPlan_terminate();
}

extern "C" void cover_map_obs_plan_run_shim(const struct0_T *map,
  const struct1_T *pyg, struct2_T *obs, const struct3_T *params,
  double *wpCnt, struct4_T *wp, double *wpLen, double *wpArea,
  double *wpTime, double *wpYaw, double *Err)
{
  CoverMapObsPlan(map, pyg, obs, params, wpCnt, wp, wpLen, wpArea, wpTime,
                  wpYaw, Err);
}

extern "C" void cover_map_obs_plan_main_shr_out_shim(const double lat_in[200],
  const double lon_in[200], double len, double distance,
  double shrOutVertex[600], double *out_cnt)
{
  mainShrOut(lat_in, lon_in, len, distance, shrOutVertex, out_cnt);
}

extern "C" void cover_map_obs_plan_main_shr_out_expand_shim(
  const double lat_in[200], const double lon_in[200], int len, double distance,
  double shrOutVertex[600], int *out_cnt)
{
  b_mainShrOut(lat_in, lon_in, len, distance, shrOutVertex, out_cnt);
}

extern "C" void cover_map_obs_plan_cover_map_by_yaw_shim(const double map[400],
  double mapCnt, double width, double yaw, double f2c, double dir,
  double *wpCnt, double wp[15000])
{
  coverMapbyYaw(map, mapCnt, width, yaw, f2c, dir, wpCnt, wp);
}

extern "C" void cover_map_obs_plan_edge_collision_shim(const double mapNew[400],
  double mapCnt, const double cPoints[4], double c3PointsOut[400],
  double *c3PointsCnt)
{
  edgeCollision(mapNew, mapCnt, cPoints, c3PointsOut, c3PointsCnt);
}

extern "C" void cover_map_obs_plan_circle_avoid_rtl_shim(
  const double pCNNew[10000], double pNC, const double obsCircle[150],
  double obsCnt, double myEps, double pNNew[10000], double *pNNewCnt)
{
  circleAvoidRTL(pCNNew, pNC, obsCircle, obsCnt, myEps, pNNew, pNNewCnt);
}

extern "C" void cover_map_obs_plan_pyg_obs_collision_shim(
  const double mapNew[600], int mapRows, int mapCnt, double pygState,
  const double cPoints[6], double c3PointsOut[1000], double *c3PointsCnt)
{
  int mapSize[2] = { mapRows, 3 };
  pygObsCollision(mapNew, mapSize, mapCnt, pygState, cPoints, c3PointsOut,
                  c3PointsCnt);
}

extern "C" void cover_map_obs_plan_pyg_obs_avoid_shim(
  const double pCNNew[15000], double pNC, const double pygNew[600],
  const int pygPnt[200], double pygCnt, const double pygState[200],
  double pNNew[10000], double *pNNewCnt)
{
  pygObsAvoid(pCNNew, pNC, pygNew, pygPnt, pygCnt, pygState, pNNew, pNNewCnt);
}
