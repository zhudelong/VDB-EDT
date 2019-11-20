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


#include <math.h>
#include <stdlib.h>
#include "vdb_edt/datacell.h"
#include "vdb_edt/dynamicVDBEDT.h"




DynamicVDBEDT::DynamicVDBEDT(int max_coor_dist)
    : maxSqDist_(max_coor_dist*max_coor_dist), version_(1){
    sum_raised_num = 0;
    sum_lowered_num = 0;
    sum_queued_num = 0;
    sum_occ_changed = 0;
}

void DynamicVDBEDT::initialize(EDTGrid::Ptr& dist_map, float vox_size, int version)
{
    dataCell c;
    c.obstX() = Empty;
    c.obstY() = Empty;
    c.obstZ() = Empty;
    c.occ() = unOccupied;
    c.dist() = maxSqDist_;
    c.radius() = notRaise;
    c.status() = notQueued;
    openvdb::initialize();
    dist_map = EDTGrid::create(/*background=*/c);
    const Vec3d offset(vox_size/2., vox_size/2., vox_size/2.);
    auto tf = math::Transform::createLinearTransform(vox_size);
    tf->postTranslate(offset);
    dist_map->setTransform(tf);
    dist_acc_ = std::make_shared<EDTGrid::Accessor>(dist_map->getAccessor());
    if (version == 0){
        inspectCellLowerF = boost::bind(&DynamicVDBEDT::inspectCellLower, this, _1, _2, _3, _4);
    } else if (version == 1 /*ral ex-version*/) {
        inspectCellLowerF = boost::bind(&DynamicVDBEDT::inspectCellLowerEx, this, _1, _2, _3, _4);
    }else {
        std::cout << "Currently, only two versions are available!" << std::endl;
        inspectCellLowerF = boost::bind(&DynamicVDBEDT::inspectCellLowerEx, this, _1, _2, _3, _4);
    }
}

double DynamicVDBEDT::query_sq_distance(const math::Coord &target)
{
    dataCell cell;
    bool known = dist_acc_->probeValue(target, cell);
    if (!known){
        return -1;
    }
    return cell.dist();
}

double DynamicVDBEDT::query_sq_distance(const Coord &target, Coord &obst)
{
    dataCell cell;
    bool known = dist_acc_->probeValue(target, cell);
    if (!known){
        return -1;
    }
    obst.x() = cell.obstX();
    obst.y() = cell.obstY();
    obst.z() = cell.obstZ();
    return cell.dist();
}


void DynamicVDBEDT::setAccessor(EDTGrid::Ptr dist_map)
{
    dist_acc_ = std::make_shared<EDTGrid::Accessor>(dist_map->getAccessor());
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////record dynamic changes in distance map////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void DynamicVDBEDT::setObstacle(int x, int y, int z)
{
    Coord p(x, y, z);
    setObstacle(p);
}

void DynamicVDBEDT::setObstacle(Coord& p) {
  dataCell c = dist_acc_->getValue(p);
    if(c.occ()!=Occupied && c.status()!=Queued){
        c.obstX() = p.x();
        c.obstY() = p.y();
        c.obstZ() = p.z();
        c.occ() = Occupied;
        c.dist() = 0;
        c.radius() = notRaise; // -1
        c.status() = Queued;
        open_.push(0, p);
        dist_acc_->setValueOn(p, c);
        obst_list_.push_back(p);
    }
}

void DynamicVDBEDT::removeObstacle(int x, int y, int z)
{
    Coord p(x, y, z);
    removeObstacle(p);
}

void DynamicVDBEDT::removeObstacle(Coord& p) {
    dataCell c = dist_acc_->getValue(p);
    if(c.occ()==Occupied && c.status()!=Queued){
        c.obstX() = Empty;
        c.obstY() = Empty;
        c.obstZ() = Empty;
        c.occ() = unOccupied;
        c.dist() = maxSqDist_;
        c.radius() = needRaise; // 0
        c.status() = Queued;
        open_.push(0, p);
        dist_acc_->setValueOn(p, c);
    }
}

///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////update distance map////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void DynamicVDBEDT::update()
{
    sum_occ_changed += open_.size();
    while (!open_.empty()) {
        Coord p = open_.pop();
        dataCell c = dist_acc_->getValue(p);

        // processed
        if(c.status() == notQueued){
            continue;
        }

        // propagate
        if (c.radius() >= needRaise) {
            raiseCell26(p, c);
            c.radius() = notRaise;
            c.status() = notQueued;
            dist_acc_->setValue(p, c);
            sum_raised_num ++;
        }else /*if (isOccupied(c.obst))*/ {
            lowerCell26(p, c);
            c.status() = notQueued;
            dist_acc_->setValue(p, c);
            sum_lowered_num ++;
        }

        // visualization(0);

    } // end while

}

void DynamicVDBEDT::update(EDTGrid::Ptr dist_map)
{
    dist_acc_ = std::make_shared<EDTGrid::Accessor>(dist_map->getAccessor());
    update();
}





//////////////////////////////////////////////////////////////////////////////////
////////////////////////////raise function areas//////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void DynamicVDBEDT::raiseCell26(Coord &p, dataCell &c)
{
    int x = p.x();
    int y = p.y();
    int z = p.z();
    int xp1 = x + 1;
    int xm1 = x - 1;
    int yp1 = y + 1;
    int ym1 = y - 1;
    int zp1 = z + 1;
    int zm1 = z - 1;

    inspectCellRaise(x, y, zp1);
    inspectCellRaise(x, y, zm1);
    inspectCellRaise(x, yp1, z);
    inspectCellRaise(x, yp1, zp1);
    inspectCellRaise(x, yp1, zm1);
    inspectCellRaise(x, ym1, z);
    inspectCellRaise(x, ym1, zp1);
    inspectCellRaise(x, ym1, zm1);
    inspectCellRaise(xp1, y, z);
    inspectCellRaise(xp1, y, zp1);
    inspectCellRaise(xp1, y, zm1);
    inspectCellRaise(xp1, yp1, z);
    inspectCellRaise(xp1, yp1, zp1);
    inspectCellRaise(xp1, yp1, zm1);
    inspectCellRaise(xp1, ym1, z);
    inspectCellRaise(xp1, ym1, zp1);
    inspectCellRaise(xp1, ym1, zm1);
    inspectCellRaise(xm1, y, z);
    inspectCellRaise(xm1, y, zp1);
    inspectCellRaise(xm1, y, zm1);
    inspectCellRaise(xm1, yp1, z);
    inspectCellRaise(xm1, yp1, zp1);
    inspectCellRaise(xm1, yp1, zm1);
    inspectCellRaise(xm1, ym1, z);
    inspectCellRaise(xm1, ym1, zp1);
    inspectCellRaise(xm1, ym1, zm1);

}


void DynamicVDBEDT::inspectCellRaise(int &nx, int &ny, int &nz){

    /** obst holds three type of values
            1. empty, need-raise and unoccupied obst -> raise wavefront
            2. empty, not-need-raise and unoccupied obst-> to lower or keep
            3. non-empty, not-need-raise but unoccupied obst-> to raise
            4. non-empty, not-need-raise and occupied obst ->to lower or keep
        and follow two rules:
            5. if a grid needs raise -> it must hold empty obstacle
            6. if a grid holds empty obstacle -> it must be unoccupied
        thus three useful conclusions:
            7. non-empty => not-need-raise
            8. empty => unoccupied
            9. grids in state-1 and part of grids in state-4 will be queued
    **/


    Coord np(nx, ny, nz);
#ifdef EXPLORATION
    dataCell nc;
    bool known = dist_acc_->probeValue(np, nc);
    if(!known) return;
#else
    dataCell nc = dist_acc_->getValue(np);
#endif

    if (nc.obstX() != Empty) { // notEmpty thus notNeedRaise
        if(!isOccupied(nc.obstX(), nc.obstY(), nc.obstZ())) { // but unoccupied obstacle
            open_.push(nc.dist(), np);
            nc.obstX() = Empty;
            nc.obstY() = Empty;
            nc.obstZ() = Empty;
            nc.radius() = nc.dist();
            nc.status() = Queued;
            nc.dist() = maxSqDist_;
            dist_acc_->setValueOn(np, nc);
        }else { // and occupied obstacle
            if(nc.status() != Queued){
                open_.push(nc.dist(), np);
                nc.status() = Queued;
                dist_acc_->setValueOn(np, nc);
            } // maybe already by other raise or lower waves
        } // reach boundaries

    } // finish checking neighbor
}


//////////////////////////////////////////////////////////////////////////////////
////////////////////////////lower function areas//////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


void DynamicVDBEDT::lowerCell26(Coord &p, dataCell &c)
{
    int x = p.x();
    int y = p.y();
    int z = p.z();
    int xp1 = x + 1;
    int xm1 = x - 1;
    int yp1 = y + 1;
    int ym1 = y - 1;
    int zp1 = z + 1;
    int zm1 = z - 1;
    int dpx = x - c.obstX();
    int dpy = y - c.obstY();
    int dpz = z - c.obstZ();

    if(c.dist() == 0){
        inspectCellLowerF(x, y, zp1, c);
        inspectCellLowerF(x, y, zm1, c);
        inspectCellLowerF(x, yp1, z, c);
        inspectCellLowerF(x, yp1, zp1, c);
        inspectCellLowerF(x, yp1, zm1, c);
        inspectCellLowerF(x, ym1, z, c);
        inspectCellLowerF(x, ym1, zp1, c);
        inspectCellLowerF(x, ym1, zm1, c);
        inspectCellLowerF(xp1, y, z, c);
        inspectCellLowerF(xp1, y, zp1, c);
        inspectCellLowerF(xp1, y, zm1, c);
        inspectCellLowerF(xp1, yp1, z, c);
        inspectCellLowerF(xp1, yp1, zp1, c);
        inspectCellLowerF(xp1, yp1, zm1, c);
        inspectCellLowerF(xp1, ym1, z, c);
        inspectCellLowerF(xp1, ym1, zp1, c);
        inspectCellLowerF(xp1, ym1, zm1, c);
        inspectCellLowerF(xm1, y, z, c);
        inspectCellLowerF(xm1, y, zp1, c);
        inspectCellLowerF(xm1, y, zm1, c);
        inspectCellLowerF(xm1, yp1, z, c);
        inspectCellLowerF(xm1, yp1, zp1, c);
        inspectCellLowerF(xm1, yp1, zm1, c);
        inspectCellLowerF(xm1, ym1, z, c);
        inspectCellLowerF(xm1, ym1, zp1, c);
        inspectCellLowerF(xm1, ym1, zm1, c);

    } else {

        int zeroVal = 0;
        if(dpz >= zeroVal){inspectCellLowerF(x, y, zp1, c);}
        if(dpz <= zeroVal){inspectCellLowerF(x, y, zm1, c);}

        if(dpy >= zeroVal){
            inspectCellLowerF(x, yp1, z, c);
            if(dpz >= zeroVal){inspectCellLowerF(x, yp1, zp1, c);}
            if(dpz <= zeroVal){inspectCellLowerF(x, yp1, zm1, c);}
        }
        if(dpy <= zeroVal){
            inspectCellLowerF(x, ym1, z, c);
            if(dpz >= zeroVal){inspectCellLowerF(x, ym1, zp1, c);}
            if(dpz <= zeroVal){inspectCellLowerF(x, ym1, zm1, c);}
        }
        if(dpx >= zeroVal){
            inspectCellLowerF(xp1, y, z, c);
            if(dpz >= zeroVal){inspectCellLowerF(xp1, y, zp1, c);}
            if(dpz <= zeroVal){inspectCellLowerF(xp1, y, zm1, c);}

            if(dpy >= zeroVal){
                inspectCellLowerF(xp1, yp1, z, c);
                if(dpz >= zeroVal){inspectCellLowerF(xp1, yp1, zp1, c);}
                if(dpz <= zeroVal){inspectCellLowerF(xp1, yp1, zm1, c);}
            }
            if(dpy <= zeroVal){
                inspectCellLowerF(xp1, ym1, z, c);
                if(dpz >= zeroVal){inspectCellLowerF(xp1, ym1, zp1, c);}
                if(dpz <= zeroVal){inspectCellLowerF(xp1, ym1, zm1, c);}
            }
        }
        if(dpx <= zeroVal){
            inspectCellLowerF(xm1, y, z, c);
            if(dpz >= zeroVal){inspectCellLowerF(xm1, y, zp1, c);}
            if(dpz <= zeroVal){inspectCellLowerF(xm1, y, zm1, c);}

            if(dpy >= zeroVal){
                inspectCellLowerF(xm1, yp1, z, c);
                if(dpz >= zeroVal){inspectCellLowerF(xm1, yp1, zp1, c);}
                if(dpz <= zeroVal){inspectCellLowerF(xm1, yp1, zm1, c);}
            }
            if(dpy <= zeroVal){
                inspectCellLowerF(xm1, ym1, z, c);
                if(dpz >= zeroVal){inspectCellLowerF(xm1, ym1, zp1, c);}
                if(dpz <= zeroVal){inspectCellLowerF(xm1, ym1, zm1, c);}
            }
        } // end if(dpx <= zeroVal)

    } // end else
}


void DynamicVDBEDT::inspectCellLower(int &nx, int &ny, int &nz, dataCell &c){

    /** obst holds three type of values
            1. empty, need-raise and unoccupied obst -> raise wavefront
            2. empty, not-need-raise and unoccupied obst-> to lower or keep
            3. non-empty, not-need-raise but unoccupied obst-> to raise or pre-lower
            4. non-empty, not-need-raise and occupied obst ->to lower or keep
        and follow two rules:
            5. if a grid needs raise -> it must hold empty obstacle
            6. if a grid holds empty obstacle -> it must be unoccupied
        thus three useful conclusions:
            7. non-empty => not-need-raise
            8. empty => unoccupied
            9. grids in state-1 and part of grids in state-4 will be queued
    **/

    Coord np(nx, ny, nz);

#ifdef EXPLORATION
    dataCell nc;
    bool known = dist_acc_->probeValue(np, nc);
    if(!known) return;
#else
    dataCell nc = dist_acc_->getValue(np);
#endif

    if(nc.radius() == notRaise) { // notNeedRaise

        int distx = nx - c.obstX();
        int disty = ny - c.obstY();
        int distz = nz - c.obstZ();
        int newSqDist = distx*distx + disty*disty + distz*distz;
        newSqDist = std::min(newSqDist, maxSqDist_);

        /*bool equ = (newSqDist == nc.dist());*/
        bool less = (newSqDist < nc.dist()); // state 2-4

        if(less /* || (equ && (!isOccupied(nc.obstX(), nc.obstY(), nc.obstZ())) ) */){
            nc.obstX() = c.obstX();
            nc.obstY() = c.obstY();
            nc.obstZ() = c.obstZ();
            nc.dist() = newSqDist;
            nc.radius() = notRaise;
            if (newSqDist < maxSqDist_){
                nc.status() = Queued;
                open_.push(newSqDist, np);
            } // boundary grids will not be queued
            dist_acc_->setValue(np, nc);

        } // propagate lower-wave

    } // end checking neighbor
}


void DynamicVDBEDT::inspectCellLowerEx(int &nx, int &ny, int &nz, dataCell &c)
{
    Coord np(nx, ny, nz);
#ifdef EXPLORATION
    dataCell nc;
    bool known = dist_acc_->probeValue(np, nc);
    if(!known) return;
#else
    dataCell nc = dist_acc_->getValue(np);
#endif
    int distx = nx - c.obstX();
    int disty = ny - c.obstY();
    int distz = nz - c.obstZ();
    int newSqDist = distx*distx + disty*disty + distz*distz;
    newSqDist = std::min(newSqDist, maxSqDist_);

    if(nc.radius() >= newSqDist){
        nc.obstX() = c.obstX();
        nc.obstY() = c.obstY();
        nc.obstZ() = c.obstZ();
        nc.dist() = newSqDist;
        nc.radius() = notRaise;
        nc.status() = Queued;
        open_.push(newSqDist, np);
        dist_acc_->setValue(np, nc);
        return;
    }else if(nc.radius() < needRaise) { // notNeedRaise
        /*bool equ = (newSqDist == nc.dist());*/
        bool less = (newSqDist < nc.dist()); // state 2-4
        if(less /*|| (equ && (!isOccupied(nc.obstX(), nc.obstY(), nc.obstZ())))*/ ){
            nc.obstX() = c.obstX();
            nc.obstY() = c.obstY();
            nc.obstZ() = c.obstZ();
            nc.dist() = newSqDist;
            nc.radius() = notRaise;
            if (newSqDist < maxSqDist_){
                nc.status() = Queued;
                open_.push(newSqDist, np);
            } // boundary grids will not be queued
            dist_acc_->setValue(np, nc);
        } // propagate lower-wave

    } // end checking neighbor

}


//////////////////////////////////////////////////////////////////////////////////
////////////////////////////tools function areas//////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/* ensure there are no repeated elements in any of the lists*/
void DynamicVDBEDT::commitAndColorize(CoordList& freeToOccu, CoordList& unkwToOccu,
                                      CoordList& occuToFree, CoordList& unkwToFree) {

    // add new obstacles
    for (auto pos = freeToOccu.begin(); pos != freeToOccu.end(); ++pos) {
        open_.push(0, *pos);
    }
    for (auto pos = unkwToOccu.begin(); pos != unkwToOccu.end(); ++pos) {
        open_.push(0, *pos);
    }

    // remove old obstacles
    for (auto pos = occuToFree.begin(); pos != occuToFree.end(); ++pos) {
        open_.push(0, *pos);
    }
    /* no need to process unkwToFree, which is already processed */
}


bool DynamicVDBEDT::isOccupied(int& x, int& y, int& z) const {
    Coord ijk(x, y, z);
    return dist_acc_->getValue(ijk).occ() == Occupied;
}

bool DynamicVDBEDT::isOccupied(Coord& xyz) const
{
    return dist_acc_->getValue(xyz).occ() == Occupied;
}

bool DynamicVDBEDT::isOccupied(Vec3i& xyz) const
{
    Coord ijk(xyz);
    return dist_acc_->getValue(ijk).occ() == Occupied;
}

void DynamicVDBEDT::visualization(int layer, int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
{
    if(layer > maxZ || layer < minZ){
        std::cout << "layer index overflows!" << std::endl;
        return;
    }

    std::cout << "current map = " << std::endl;
    int k = layer;
    std::string line = "";
    for (int i = minX; i <= maxX; ++i) {
        if(line.length() <= 0) line += "|";
        else {
            std::cout << line << std::endl;
            line = "|";
        }
        for (int j = minY; j <= maxY; ++j) {
            Coord ijk(i,j,k);
            dataCell c = dist_acc_->getValue(ijk);
            char buff[6];
            sprintf(buff, "%2d|", c.dist());
            line += buff;
        }
    }
    std::cout << line << std::endl;
}

int DynamicVDBEDT::getSqDistance(int i, int j, int k)
{
   Coord ijk(i,j,k);
   return dist_acc_->getValue(ijk).dist();

}



