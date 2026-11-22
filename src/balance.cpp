#include "reflex/balance.h"

#include <algorithm>
#include <cmath>

namespace reflex {

Vec3 WeightedCentre(const Vec3* points, const float* masses, int count) {
    Vec3 sum;
    float m = 0.0f;
    for (int i = 0; i < count; ++i) {
        sum += points[i] * masses[i];
        m += masses[i];
    }
    return m > 0.0f ? sum / m : Vec3{};
}

Vec3 CapturePoint(const Vec3& com, const Vec3& v, float h, float g) {
    float t = std::sqrt(std::max(h, 0.05f) / g);
    return Vec3{com.x + v.x * t, 0.0f, com.z + v.z * t};
}

namespace {

float Cross2(const Vec3& o, const Vec3& a, const Vec3& b) {
    return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
}

float SegmentDistanceSq(const Vec3& p, const Vec3& a, const Vec3& b, Vec3* closest) {
    float abx = b.x - a.x, abz = b.z - a.z;
    float len2 = abx * abx + abz * abz;
    float t = len2 > 1e-12f ? ((p.x - a.x) * abx + (p.z - a.z) * abz) / len2 : 0.0f;
    t = Saturate(t);
    Vec3 c{a.x + abx * t, 0.0f, a.z + abz * t};
    if (closest) *closest = c;
    float dx = p.x - c.x, dz = p.z - c.z;
    return dx * dx + dz * dz;
}

} // namespace

SupportPolygon SupportPolygon::FromPoints(const Vec3* pts, int n) {
    SupportPolygon poly;
    if (n <= 0) return poly;
    Vec3 sorted[64];
    n = std::min(n, 64);
    for (int i = 0; i < n; ++i) sorted[i] = Vec3{pts[i].x, 0.0f, pts[i].z};
    std::sort(sorted, sorted + n, [](const Vec3& a, const Vec3& b) { return a.x < b.x || (a.x == b.x && a.z < b.z); });
    // Andrew's monotone chain.
    Vec3 hull[2 * 64];
    int k = 0;
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && Cross2(hull[k - 2], hull[k - 1], sorted[i]) <= 0) --k;
        hull[k++] = sorted[i];
    }
    for (int i = n - 2, t = k + 1; i >= 0; --i) {
        while (k >= t && Cross2(hull[k - 2], hull[k - 1], sorted[i]) <= 0) --k;
        hull[k++] = sorted[i];
    }
    if (k > 1) --k; // last point repeats the first
    if (n == 1) k = 1;
    // Collapse duplicates for tiny inputs.
    poly.count = std::min(k, kMaxPoints);
    for (int i = 0; i < poly.count; ++i) poly.points[i] = hull[i];
    return poly;
}

float SupportPolygon::SignedDistance(const Vec3& p) const {
    if (count == 0) return 1e9f;
    if (count == 1) return std::sqrt((p.x - points[0].x) * (p.x - points[0].x) + (p.z - points[0].z) * (p.z - points[0].z));
    float minDistSq = 1e18f;
    for (int i = 0; i < count; ++i) {
        const Vec3& a = points[i];
        const Vec3& b = points[(i + 1) % count];
        minDistSq = std::min(minDistSq, SegmentDistanceSq(p, a, b, nullptr));
    }
    float d = std::sqrt(minDistSq);
    if (count < 3) return d;
    // Hull is counter-clockwise: inside means left of every edge.
    bool inside = true;
    for (int i = 0; i < count; ++i) {
        if (Cross2(points[i], points[(i + 1) % count], p) < 0.0f) {
            inside = false;
            break;
        }
    }
    return inside ? -d : d;
}

Vec3 SupportPolygon::ClosestPoint(const Vec3& p) const {
    if (count == 0) return Vec3{p.x, 0, p.z};
    if (count == 1) return points[0];
    if (count >= 3 && Contains(p)) return Vec3{p.x, 0.0f, p.z};
    float best = 1e18f;
    Vec3 bestPt = points[0];
    for (int i = 0; i < count; ++i) {
        Vec3 c;
        float d = SegmentDistanceSq(p, points[i], points[(i + 1) % count], &c);
        if (d < best) {
            best = d;
            bestPt = c;
        }
    }
    return bestPt;
}

Vec3 SupportPolygon::Centroid() const {
    Vec3 s;
    for (int i = 0; i < count; ++i) s += points[i];
    return count ? s / static_cast<float>(count) : Vec3{};
}

LegLoads ComputeLegLoads(float capL, float capR, float bias) {
    capL = Saturate(capL);
    capR = Saturate(capR);
    LegLoads out;
    float sum = capL + capR;
    if (sum < 1e-4f) {
        out.left = out.right = 0.5f;
        out.deficit = 1.0f;
        return out;
    }
    float l = capL / sum;
    // The bias shifts weight, but a leg is never loaded beyond what it can bear.
    l += 0.25f * Clamp(bias, -1.0f, 1.0f);
    const float maxL = 0.15f + 0.85f * capL, maxR = 0.15f + 0.85f * capR;
    const float lo = 1.0f - maxR, hi = maxL;
    if (lo <= hi) l = Clamp(l, lo, hi);
    else l = capL / sum; // combined capacity too low: share proportionally
    out.left = l;
    out.right = 1.0f - l;
    out.deficit = Saturate(1.0f - sum);
    return out;
}

Vec3 ComputeStepTarget(const Vec3& cp, const Vec3& stanceFoot, const Vec3& velDir, float maxReach, float margin) {
    Vec3 dir = Horizontal(velDir);
    float dl = Length(dir);
    Vec3 target = Vec3{cp.x, 0.0f, cp.z};
    if (dl > 1e-4f) target += dir / dl * margin;
    Vec3 offset = Vec3{target.x - stanceFoot.x, 0.0f, target.z - stanceFoot.z};
    float len = Length(offset);
    if (len > maxReach && len > 1e-6f) offset = offset * (maxReach / len);
    return Vec3{stanceFoot.x + offset.x, 0.0f, stanceFoot.z + offset.z};
}

GaitParams ComputeGait(float capL, float capR) {
    GaitParams g;
    float cap[2] = {Saturate(capL), Saturate(capR)};
    float weaker = std::min(cap[0], cap[1]);
    for (int i = 0; i < 2; ++i) {
        float c = cap[i];
        g.stepLengthScale[i] = Lerp(0.35f, 1.0f, c);
        g.swingTimeScale[i] = Lerp(0.7f, 1.0f, c);
        // Weak legs spend less time under load, strong legs carry on longer.
        g.stanceTimeScale[i] = Lerp(0.55f, 1.0f, c);
        g.drag[i] = c < 0.25f;
    }
    // The healthy leg takes longer, safer strides next to a weak one.
    int strong = cap[0] >= cap[1] ? 0 : 1;
    g.stepLengthScale[strong] = Lerp(0.6f, 1.0f, weaker);
    g.speedScale = Lerp(0.35f, 1.0f, std::sqrt(weaker));
    return g;
}

} // namespace reflex
