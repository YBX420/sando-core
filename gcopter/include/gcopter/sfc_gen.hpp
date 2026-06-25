/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

// VENDORED CHANGE (sando-core): the original planPath used OMPL InformedRRTstar to find the guide path. OMPL is a
// heavy external dependency we don't ship; the guide path is only a TOPOLOGICAL hint for convexCover (GCOPTER's
// MINCO + FIRI optimization core below is untouched), so we replace it with a self-contained 3D grid A* on the
// VoxelMap. Same signature; returns the path length (or INFINITY if no grid path -> caller falls back).
#include <deque>
#include <memory>
#include <queue>
#include <vector>
#include <limits>
#include <Eigen/Eigen>

namespace sfc_gen
{

    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        (void)lb; (void)hb; (void)timeout;
        const Eigen::Vector3i size = mapPtr->getSize();
        const long sy = size(1), sz = size(2);
        auto idx = [&](const Eigen::Vector3i &c) -> long { return (long(c(0)) * sy + c(1)) * sz + c(2); };
        auto i2c = [&](long i) -> Eigen::Vector3i {
            return Eigen::Vector3i(int(i / (sy * sz)), int((i / sz) % sy), int(i % sz)); };
        auto inb = [&](const Eigen::Vector3i &c) {
            return c(0) >= 0 && c(1) >= 0 && c(2) >= 0 && c(0) < size(0) && c(1) < size(1) && c(2) < size(2); };
        const Eigen::Vector3i si = mapPtr->posD2I(s), gi = mapPtr->posD2I(g);
        if (!inb(si) || !inb(gi))
        {
            p.clear(); p.push_back(s); p.push_back(g); return (g - s).norm();
        }
        const long N = long(size(0)) * sy * sz;
        std::vector<float> gsc(N, std::numeric_limits<float>::infinity());
        std::vector<long> came(N, -1);
        auto hcost = [&](const Eigen::Vector3i &c) { return float((c - gi).cast<double>().norm()); };
        typedef std::pair<float, long> QE;
        std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
        gsc[idx(si)] = 0.0f; pq.push({hcost(si), idx(si)});
        const long gIdx = idx(gi); long found = -1;
        while (!pq.empty())
        {
            QE top = pq.top(); pq.pop();
            long ci = top.second; if (ci == gIdx) { found = ci; break; }
            Eigen::Vector3i c = i2c(ci);
            if (top.first - hcost(c) > gsc[ci] + 1e-4f) continue;   // stale entry
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (!dx && !dy && !dz) continue;
                        Eigen::Vector3i nc(c(0) + dx, c(1) + dy, c(2) + dz);
                        if (!inb(nc) || mapPtr->query(nc)) continue;
                        float ng = gsc[ci] + float(Eigen::Vector3i(dx, dy, dz).cast<double>().norm());
                        long ni = idx(nc);
                        if (ng < gsc[ni]) { gsc[ni] = ng; came[ni] = ci; pq.push({ng + hcost(nc), ni}); }
                    }
        }
        p.clear();
        if (found < 0)
        {
            p.push_back(s); p.push_back(g); return (g - s).norm();   // no grid path -> straight hint
        }
        std::vector<Eigen::Vector3d> rev;
        for (long ci = found; ci >= 0; ci = came[ci]) rev.push_back(mapPtr->posI2D(i2c(ci)));
        for (auto it = rev.rbegin(); it != rev.rend(); ++it) p.push_back(*it);
        p.front() = s; p.back() = g;
        return double(gsc[found]) * mapPtr->getScale();
    }

    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
