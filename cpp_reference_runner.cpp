#include "../CoverMapObsPlan.h"
#include "../CoverMapObsPlan_initialize.h"
#include "../CoverMapObsPlan_terminate.h"

#include <cstdio>
#include <cstring>

int main()
{
  struct0_T map;
  struct1_T pyg;
  struct2_T obs;
  struct3_T params;
  struct4_T wp;
  double wpCnt = 0.0;
  double wpLen = 0.0;
  double wpArea = 0.0;
  double wpTime = 0.0;
  double wpYaw = 0.0;
  double err = 0.0;

  std::memset(&map, 0, sizeof(map));
  std::memset(&pyg, 0, sizeof(pyg));
  std::memset(&obs, 0, sizeof(obs));
  std::memset(&params, 0, sizeof(params));
  std::memset(&wp, 0, sizeof(wp));

  map.Lat[0] = 30.0000;
  map.Lat[1] = 30.0000;
  map.Lat[2] = 30.0010;
  map.Lat[3] = 30.0010;
  map.Lon[0] = 120.0000;
  map.Lon[1] = 120.0010;
  map.Lon[2] = 120.0010;
  map.Lon[3] = 120.0000;
  map.Cnt = 4.0;

  params.width = 20.0;
  params.yaw = 0.0;
  params.dir = 1.0;
  params.speed = 2.0;
  params.safe_dist_obs = 0.0;
  params.safe_dist_map = 0.0;
  params.long_edge_yaw_flag = 0;

  CoverMapObsPlan_initialize();
  CoverMapObsPlan(&map, &pyg, &obs, &params, &wpCnt, &wp, &wpLen, &wpArea,
                  &wpTime, &wpYaw, &err);
  CoverMapObsPlan_terminate();

  if (err == 0.0) {
    std::puts("Planning succeeded");
    std::printf("waypoint_count=%d\n", static_cast<int>(wpCnt));
    std::printf("path_length=%.3f\n", wpLen);
    std::printf("coverage_area=%.3f\n", wpArea);
    std::printf("estimated_time=%.3f\n", wpTime);
    std::printf("yaw=%.3f\n", wpYaw);
  } else {
    std::printf("Planning failed: %.0f\n", err);
  }

  return err == 0.0 ? 0 : 1;
}
