/*
 * Copyright (c) Deron (Delong Zhu)
   The Chinese University of Hong Kong
   Carnegie Mellon University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DYNAMICVDBEDT3D_H_
#define _DYNAMICVDBEDT3D_H_

#include <limits.h>
#include <queue>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <openvdb/openvdb.h>
#include "bucketedqueue.h"
#include "datacell.h"


#define EXPLORATION

using namespace openvdb;
typedef openvdb::math::dataCell7i dataCell;
using EDTree = tree::Tree4<dataCell, 5, 4, 3>::Type;
//using EDTree = tree::Tree3<dataCell, 6, 4>::Type;
using EDTGrid = Grid<EDTree>;
typedef std::vector<openvdb::math::Coord> CoordList;



class DynamicVDBEDT {

public:
  DynamicVDBEDT(int max_coor_dist);
  ~DynamicVDBEDT(){}

  void setAccessor(EDTGrid::Ptr dist_map);
  void initialize(EDTGrid::Ptr& dist_map, float vox_size, int version);

  // return distance and the nearest obstacle
  double query_sq_distance(const Coord& target);
  double query_sq_distance(const Coord& target, Coord& obst);

  // update distance map to reflect the changes
  void update();
  void update(EDTGrid::Ptr dist_map);

  // record changes in the distance map
  void setObstacle(Coord &p);
  void setObstacle(int x, int y, int z);
  void removeObstacle(Coord &p);
  void removeObstacle(int x, int y, int z);


  void commitAndColorize(CoordList &freeToOccu, CoordList &unkwToOccu,
                         CoordList &occuToFree, CoordList &unkwToFree);

  std::shared_ptr<EDTGrid::Accessor> dist_acc_;
  std::vector<Coord> obst_list_;


protected:

  typedef enum {unOccupied=0, Occupied=1} Occupancy;
  typedef enum {notRaise=-1, needRaise=0} RaiseLevel;
  typedef enum {notQueued=0, Queued=1, Processed=2} StateLevel;
  typedef enum {Invalid=0, Success=1, Failed=2} LowerResult;
  typedef enum {outBound=0, No=1, Yes=2} ObstState;
  typedef enum {invalidObstData = INT_MAX, Empty = INT_MAX} ObstDataState;

  /* raise functions */
  inline void raiseCell26(Coord &p, dataCell &c);
  // refined version of raise
  inline void inspectCellRaise(int &nx, int &ny, int &nz);

  /* lower functions */
  inline void lowerCell26(Coord &p, dataCell &c);
  // original version of lower
  inline void inspectCellLower(int &nx, int &ny, int &nz, dataCell &c);
  // imporved version of lower
  inline void inspectCellLowerEx(int &nx, int &ny, int &nz, dataCell &c);
  // handler of the above two functions
  boost::function<void(int &nx, int &ny, int &nz, dataCell &c)> inspectCellLowerF;


private:
  /* fetch map cell and check Occu field */
  inline bool isOccupied(Coord &xyz) const;
  inline bool isOccupied(Vec3i &xyz) const;
  inline bool isOccupied(int &x, int &y, int &z) const;

  int getSqDistance(int i, int j, int k);
  void visualization(int layer,
                     int minX, int minY, int minZ,
                     int maxX, int maxY, int maxZ);

  // parameters
  int maxSqDist_;
  int version_;  // 0-original 1-ex version
  BucketPrioQueue<Coord> open_;
  std::vector<Coord> addList_;
  std::vector<Coord> removeList_;

public:
  // statistics
  Int64 sum_raised_num, sum_lowered_num, sum_queued_num, sum_occ_changed;

};


#endif
