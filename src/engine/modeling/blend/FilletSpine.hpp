#pragma once

#include "engine/geom/Transform3D.hpp"
#include "engine/geom/Plane.hpp"
#include "engine/modeling/BlendGeometry.hpp"
#include "engine/modeling/ssi/QuadricIntersection.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace Tessera::Modeling::Blend {

struct SpineSample {
    double param{0.0};
    Geom::Point3D edge_pt{0.0, 0.0, 0.0};
    Geom::Point3D center_pt{0.0, 0.0, 0.0};
    Geom::Point3D rail1_pt{0.0, 0.0, 0.0};
    Geom::Point3D rail2_pt{0.0, 0.0, 0.0};
    Geom::Vector3D tangent{0.0, 0.0, 1.0};
    Geom::Vector3D normal1{0.0, 1.0, 0.0};
    Geom::Vector3D normal2{1.0, 0.0, 0.0};
};

class FilletSpine {
public:
    double radius{0.2};
    bool is_convex{true};
    double dihedral_angle{1.5707963267948966};
    std::vector<SpineSample> samples;

    FilletSpine() = default;
    FilletSpine(double r, bool convex) : radius(r), is_convex(convex) {}

    void Evaluate(
        const Geom::Point3D& p0,
        const Geom::Point3D& p1,
        const Geom::Vector3D& n1,
        const Geom::Vector3D& n2,
        double r,
        bool convex,
        int sampleCount = 16
    ) {
        radius = r;
        is_convex = convex;
        samples.clear();

        double edgeLen = p0.distance_to(p1);
        if (edgeLen < 1e-5) return;

        Geom::Vector3D t = (p1 - p0) / edgeLen;
        DihedralInfo info = BlendGeometry::ClassifyDihedralEdge(n1, n2, t);
        dihedral_angle = info.dihedral_angle;

        int n = std::max(2, sampleCount);
        samples.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            double s = static_cast<double>(i) / static_cast<double>(n);
            Geom::Point3D pe = p0 + (p1 - p0) * s;

            Geom::Point3D center = BlendGeometry::ComputeRollingBallCenter(
                pe, n1, n2, radius, is_convex, dihedral_angle
            );

            Geom::Point3D r1, r2;
            BlendGeometry::ComputeContactRails(
                center, n1, n2, radius, is_convex, r1, r2
            );

            SpineSample smp;
            smp.param = s;
            smp.edge_pt = pe;
            smp.center_pt = center;
            smp.rail1_pt = r1;
            smp.rail2_pt = r2;
            smp.tangent = t;
            smp.normal1 = n1.normalized();
            smp.normal2 = n2.normalized();

            samples.push_back(smp);
        }
    }

    void TrimStartAtPlane(const SSI::AnalyticalPlane& plane) {
        if (samples.size() < 2) return;

        double tIntersect = 0.0;
        Geom::Point3D ptIntersect;
        for (size_t i = 0; i + 1 < samples.size(); ++i) {
            if (SSI::QuadricIntersection::IntersectSegmentPlane(samples[i].edge_pt, samples[i + 1].edge_pt, plane, tIntersect, ptIntersect)) {
                // Adjust samples: discard elements before i, replace i with trimmed sample
                samples.erase(samples.begin(), samples.begin() + i);
                samples[0].edge_pt = ptIntersect;
                samples[0].center_pt = BlendGeometry::ComputeRollingBallCenter(
                    ptIntersect, samples[0].normal1, samples[0].normal2, radius, is_convex, dihedral_angle
                );
                BlendGeometry::ComputeContactRails(
                    samples[0].center_pt, samples[0].normal1, samples[0].normal2, radius, is_convex,
                    samples[0].rail1_pt, samples[0].rail2_pt
                );
                break;
            }
        }
    }

    void TrimEndAtPlane(const SSI::AnalyticalPlane& plane) {
        if (samples.size() < 2) return;

        double tIntersect = 0.0;
        Geom::Point3D ptIntersect;
        for (int i = static_cast<int>(samples.size()) - 2; i >= 0; --i) {
            if (SSI::QuadricIntersection::IntersectSegmentPlane(samples[i].edge_pt, samples[i + 1].edge_pt, plane, tIntersect, ptIntersect)) {
                samples.erase(samples.begin() + (i + 1), samples.end());
                SpineSample endSmp = samples.back();
                endSmp.edge_pt = ptIntersect;
                endSmp.center_pt = BlendGeometry::ComputeRollingBallCenter(
                    ptIntersect, endSmp.normal1, endSmp.normal2, radius, is_convex, dihedral_angle
                );
                BlendGeometry::ComputeContactRails(
                    endSmp.center_pt, endSmp.normal1, endSmp.normal2, radius, is_convex,
                    endSmp.rail1_pt, endSmp.rail2_pt
                );
                samples.push_back(endSmp);
                break;
            }
        }
    }
};

} // namespace Tessera::Modeling::Blend
