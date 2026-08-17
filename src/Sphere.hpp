#pragma once

#ifndef SPHERE_HPP
#define SPHERE_HPP

#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

#include <iostream>

#include "MVec.hpp"

struct Triangle3d
{
    Vec3 p0, p1, p2;
};

class Sphere : public sf::Drawable
{
public:
    Sphere(const Vec3& _position, float _radius, int _density = 1, bool _drawWireframe = false) {
        position = _position;
        radius = _radius;
        this->density = _density;
        this->drawWireframe = _drawWireframe;
        buildGeodesicMesh();
    }
    virtual ~Sphere() {
        deleteDisplayLists();
    };

    // World-space triangles of the (possibly terrain-displaced) surface, for collision.
    // Built lazily and cached: displacedPoint() calls the virtual radiusOffsetForVertex(),
    // which can't resolve to a derived override (e.g. Planet's terrain noise) while still
    // inside the Sphere constructor, and the surface never changes after construction
    // anyway, so a one-time cache avoids recomputing per-triangle noise every collision check.
    const std::vector<Triangle3d>& getSurfaceTriangles() const {
        if (surfaceCacheDirty)
            buildSurfaceCache();
        return surfaceTriangles;
    }

    Vec3 getPosition() const { return position; }

    // Surface radius (base radius + terrain offset) along a given direction from
    // center, without building/walking the mesh - the primitive a height-field bake
    // samples over a theta/phi grid instead of scanning triangles.
    float surfaceRadiusFor(const Vec3& direction) const {
        return radius + radiusOffsetForVertex(normalize(direction));
    }

    void draw(sf::RenderTarget& target, sf::RenderStates states) const override{
        (void)target;
        (void)states;

        //std::cout << "Drawing sphere at position: " << position << " with radius: " << radius << std::endl;

        if (!drawFilled && !drawWireframe)
            return;

        if (listsDirty)
            prepareDraw();

        glPushMatrix();
        glTranslatef(position.x, position.y, position.z);

        if (drawFilled && fillList != 0)
            glCallList(fillList);

        if (drawWireframe && wireFrameList != 0)
        {
            glDisable(GL_LIGHTING);
            glCallList(wireFrameList);
            glEnable(GL_LIGHTING);
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glPopMatrix();
    }

    void toggleWireframe() {
        drawWireframe = !drawWireframe;
    }

    void toggleFilled() {
        drawFilled = !drawFilled;
    }

    void prepareDraw() const
    {
        deleteDisplayLists();

        fillList = glGenLists(1);
        if (fillList != 0)
        {
            glNewList(fillList, GL_COMPILE);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glBegin(GL_TRIANGLES);
            std::cout << "Preparing draw: " << vertices.size() << " vertices, " << indices.size() << " faces." << std::endl;
            for (const auto& face : indices)
            {
                const Vec3& n0 = vertices[face[0]];
                const Vec3& n1 = vertices[face[1]];
                const Vec3& n2 = vertices[face[2]];

                const Vec3 p0 = displacedPoint(n0);
                const Vec3 p1 = displacedPoint(n1);
                const Vec3 p2 = displacedPoint(n2);

                glNormal3f(n0.x, n0.y, n0.z);
                const Vec3 c0 = fillColorForVertex(n0);
                glColor3f(c0.x, c0.y, c0.z);
                glVertex3f(p0.x, p0.y, p0.z);

                glNormal3f(n1.x, n1.y, n1.z);
                const Vec3 c1 = fillColorForVertex(n1);
                glColor3f(c1.x, c1.y, c1.z);
                glVertex3f(p1.x, p1.y, p1.z);

                glNormal3f(n2.x, n2.y, n2.z);
                const Vec3 c2 = fillColorForVertex(n2);
                glColor3f(c2.x, c2.y, c2.z);
                glVertex3f(p2.x, p2.y, p2.z);
            }
            glEnd();
            glEndList();
        }

        wireFrameList = glGenLists(1);
        if (wireFrameList != 0)
        {
            glNewList(wireFrameList, GL_COMPILE);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glLineWidth(1.2f);
            glColor3f(wireframeColor.x, wireframeColor.y, wireframeColor.z);
            glBegin(GL_TRIANGLES);
            for (const auto& face : indices)
            {
                const Vec3& n0 = vertices[face[0]];
                const Vec3& n1 = vertices[face[1]];
                const Vec3& n2 = vertices[face[2]];

                const Vec3 p0 = displacedPoint(n0);
                const Vec3 p1 = displacedPoint(n1);
                const Vec3 p2 = displacedPoint(n2);

                glVertex3f(p0.x, p0.y, p0.z);
                glVertex3f(p1.x, p1.y, p1.z);
                glVertex3f(p2.x, p2.y, p2.z);
            }
            glEnd();
            glEndList();
        }

        std::cout << "Display lists prepared: fillList=" << fillList << ", wireFrameList=" << wireFrameList << std::endl;

        listsDirty = false;
    }

protected:
    virtual Vec3 fillColorForVertex(const Vec3& normal) const
    {
        (void)normal;
        return fillColor;
    }

    virtual float radiusOffsetForVertex(const Vec3& normal) const
    {
        (void)normal;
        return 0.0f;
    }

    Vec3 displacedPoint(const Vec3& normal) const
    {
        const float r = radius + radiusOffsetForVertex(normal);
        return normal * r;
    }

    static Vec3 normalize(const Vec3& v)
    {
        const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len <= 0.0f)
            return Vec3::nullV;

        return Vec3(v.x / len, v.y / len, v.z / len);
    }

private:
    void invalidateDrawCache()
    {
        listsDirty = true;
    }

    void deleteDisplayLists() const
    {
        if (fillList != 0)
        {
            glDeleteLists(fillList, 1);
            fillList = 0;
        }

        if (wireFrameList != 0)
        {
            glDeleteLists(wireFrameList, 1);
            wireFrameList = 0;
        }
    }

    void buildSurfaceCache() const
    {
        surfaceTriangles.clear();
        surfaceTriangles.reserve(indices.size());
        for (const auto& face : indices)
        {
            surfaceTriangles.push_back({
                position + displacedPoint(vertices[face[0]]),
                position + displacedPoint(vertices[face[1]]),
                position + displacedPoint(vertices[face[2]])
            });
        }
        surfaceCacheDirty = false;
    }

    unsigned int midpointIndex(unsigned int i0, unsigned int i1, std::unordered_map<unsigned long long, unsigned int>& cache)
    {
        const unsigned int a = std::min(i0, i1);
        const unsigned int b = std::max(i0, i1);
        const unsigned long long key = (static_cast<unsigned long long>(a) << 32) | static_cast<unsigned long long>(b);

        const auto found = cache.find(key);
        if (found != cache.end())
            return found->second;

        const Vec3 mid = normalize((vertices[a] + vertices[b]) * 0.5f);
        vertices.push_back(mid);
        const unsigned int idx = static_cast<unsigned int>(vertices.size() - 1);
        cache.emplace(key, idx);
        return idx;
    }

    void buildGeodesicMesh()
    {
        vertices.clear();
        indices.clear();

        const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;

        vertices = {
            normalize(Vec3(-1.0f,  t,  0.0f)),
            normalize(Vec3( 1.0f,  t,  0.0f)),
            normalize(Vec3(-1.0f, -t,  0.0f)),
            normalize(Vec3( 1.0f, -t,  0.0f)),
            normalize(Vec3( 0.0f, -1.0f,  t)),
            normalize(Vec3( 0.0f,  1.0f,  t)),
            normalize(Vec3( 0.0f, -1.0f, -t)),
            normalize(Vec3( 0.0f,  1.0f, -t)),
            normalize(Vec3( t,  0.0f, -1.0f)),
            normalize(Vec3( t,  0.0f,  1.0f)),
            normalize(Vec3(-t,  0.0f, -1.0f)),
            normalize(Vec3(-t,  0.0f,  1.0f))
        };

        indices = {
            {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
            {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
            {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
            {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
        };

        const int subdivisions = std::max(0, density);
        for (int s = 0; s < subdivisions; ++s)
        {
            std::vector<std::array<unsigned int, 3>> next;
            next.reserve(indices.size() * 4);
            std::unordered_map<unsigned long long, unsigned int> cache;
            cache.reserve(indices.size() * 3);

            for (const auto& tri : indices)
            {
                const unsigned int a = midpointIndex(tri[0], tri[1], cache);
                const unsigned int b = midpointIndex(tri[1], tri[2], cache);
                const unsigned int c = midpointIndex(tri[2], tri[0], cache);

                next.push_back({tri[0], a, c});
                next.push_back({tri[1], b, a});
                next.push_back({tri[2], c, b});
                next.push_back({a, b, c});
            }

            indices.swap(next);
        }

        invalidateDrawCache();
    }

    Vec3 position;
    float radius;
    int density;
    std::vector<Vec3> vertices;
    std::vector<std::array<unsigned int, 3>> indices;
    bool drawWireframe = false;
    bool drawFilled = true;
    Vec3 fillColor = Vec3(0.35f, 0.35f, 0.35f);
    Vec3 wireframeColor = Vec3(0.95f, 0.2f, 0.2f);

    mutable GLuint fillList = 0;
    mutable GLuint wireFrameList = 0;
    mutable bool listsDirty = true;

    mutable std::vector<Triangle3d> surfaceTriangles;
    mutable bool surfaceCacheDirty = true;
};

// Maps a direction from a sphere's center to a (thetaBucket, phiBucket) cell in a
// thetaCount x phiCount grid. Shared by SphereHeightField and SphereTriangleGrid
// so both bucket directions the same way.
struct SphericalBucketing
{
    int thetaCount;
    int phiCount;

    static float wrapPhi(float phi)
    {
        const float twoPi = 2.f * M_PIF;
        phi = std::fmod(phi, twoPi);
        if (phi < 0.f)
            phi += twoPi;
        return phi;
    }

    static void anglesFor(const Vec3& direction, float& theta, float& phi)
    {
        const float len = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
        if (len <= 0.f) {
            theta = 0.f;
            phi = 0.f;
            return;
        }
        theta = std::acos(std::clamp(direction.y / len, -1.f, 1.f));
        phi = wrapPhi(std::atan2(direction.z / len, direction.x / len));
    }

    static Vec3 directionForAngles(float theta, float phi)
    {
        const float sinTheta = std::sin(theta);
        return Vec3(sinTheta * std::cos(phi), std::cos(theta), sinTheta * std::sin(phi));
    }

    void bucketFor(const Vec3& direction, int& ti, int& pi) const
    {
        float theta, phi;
        anglesFor(direction, theta, phi);
        ti = std::clamp(static_cast<int>(theta / M_PIF * thetaCount), 0, thetaCount - 1);
        pi = static_cast<int>(phi / (2.f * M_PIF) * phiCount) % phiCount;
    }
};

// Bakes the surface radius on a theta/phi grid once up front, for a cheap O(1)
// "is this particle anywhere near the surface" broad-phase reject - not precise
// enough to resolve a collision from (see SphereTriangleGrid for that), just fast
// enough to skip the exact check for particles that are clearly nowhere close.
class SphereHeightField
{
public:
    SphereHeightField(const Sphere& sphere, int thetaSamples, int phiSamples)
        : bucketing{ std::max(2, thetaSamples), std::max(3, phiSamples) }
    {
        radii.resize(static_cast<size_t>(bucketing.thetaCount) * bucketing.phiCount);

        #pragma omp parallel for schedule(static)
        for (int ti = 0; ti < bucketing.thetaCount; ++ti)
        {
            for (int pi = 0; pi < bucketing.phiCount; ++pi)
            {
                const float theta = M_PIF * ti / (bucketing.thetaCount - 1);
                const float phi = 2.f * M_PIF * pi / bucketing.phiCount;
                radii[static_cast<size_t>(ti) * bucketing.phiCount + pi] =
                    sphere.surfaceRadiusFor(SphericalBucketing::directionForAngles(theta, phi));
            }
        }
    }

    // Bilinearly interpolated surface radius along `direction` from center.
    // direction need not be normalized or in world space - caller passes
    // (particlePos - sphere.getPosition()).
    float surfaceRadiusAt(const Vec3& direction) const
    {
        float theta, phi;
        SphericalBucketing::anglesFor(direction, theta, phi);

        const float tf = theta / M_PIF * (bucketing.thetaCount - 1);
        const float pf = phi / (2.f * M_PIF) * bucketing.phiCount;

        const int t0 = std::clamp(static_cast<int>(tf), 0, bucketing.thetaCount - 1);
        const int t1 = std::clamp(t0 + 1, 0, bucketing.thetaCount - 1);
        const float tFrac = tf - t0;

        const int p0 = static_cast<int>(pf) % bucketing.phiCount;
        const int p1 = (p0 + 1) % bucketing.phiCount;
        const float pFrac = pf - std::floor(pf);

        const float r00 = radii[static_cast<size_t>(t0) * bucketing.phiCount + p0];
        const float r01 = radii[static_cast<size_t>(t0) * bucketing.phiCount + p1];
        const float r10 = radii[static_cast<size_t>(t1) * bucketing.phiCount + p0];
        const float r11 = radii[static_cast<size_t>(t1) * bucketing.phiCount + p1];

        const float r0 = r00 + (r01 - r00) * pFrac;
        const float r1 = r10 + (r11 - r10) * pFrac;
        return r0 + (r1 - r0) * tFrac;
    }

private:
    SphericalBucketing bucketing;
    std::vector<float> radii; // row-major: theta-major, phi-minor
};

// Buckets a Sphere's cached surface triangles (by centroid direction from
// center) into a coarse theta/phi grid, mirroring ChunkGrid3d's spatial-hash
// pattern - just for triangles-by-direction instead of particles-by-position.
// Lets a collision check find the handful of triangles actually near a given
// direction (a bucket's contents plus its immediate neighbors) instead of
// brute-forcing every triangle on the planet. Resolving against the real,
// exact triangles this way - rather than an interpolated/baked approximation -
// is what makes this robust right at steep terrain: closest_point_on_triangle
// has no equivalent of the height-field-normal instability, since it's exact
// geometry, not a blended estimate.
class SphereTriangleGrid
{
public:
    SphereTriangleGrid(const Sphere& sphere, int thetaBuckets, int phiBuckets)
        : bucketing{ std::max(1, thetaBuckets), std::max(1, phiBuckets) },
          triangles{ &sphere.getSurfaceTriangles() }
    {
        buckets.resize(static_cast<size_t>(bucketing.thetaCount) * bucketing.phiCount);

        const Vec3 center = sphere.getPosition();
        for (size_t i = 0; i < triangles->size(); ++i)
        {
            const Triangle3d& tri = (*triangles)[i];
            const Vec3 centroid = (tri.p0 + tri.p1 + tri.p2) * (1.f / 3.f);
            int ti, pi;
            bucketing.bucketFor(centroid - center, ti, pi);
            buckets[static_cast<size_t>(ti) * bucketing.phiCount + pi].push_back(static_cast<int>(i));
        }
    }

    const std::vector<Triangle3d>& allTriangles() const { return *triangles; }

    // Appends indices (into allTriangles()) of triangles bucketed near
    // `direction` - the home bucket plus its neighbors, since a triangle whose
    // centroid falls in an adjacent bucket can still be the closest one. Does
    // not clear outIndices first, may contain duplicates (harmless - checking
    // the same triangle twice just wastes a comparison, not correctness).
    void queryNearby(const Vec3& direction, std::vector<int>& outIndices) const
    {
        int ti, pi;
        bucketing.bucketFor(direction, ti, pi);

        // A phi bucket's physical width shrinks toward the poles (it scales with
        // sin(theta), the local radius of the sphere's latitude circle - a fixed
        // +/-1 bucket margin covers less and less real distance there), so a
        // triangle can be geometrically close but many phi-buckets away in angle.
        // Widen the phi search proportionally to 1/sin(theta) to keep the
        // physical margin roughly constant near the equator's; clamp so it just
        // wraps the whole ring once that would cover it anyway (which is
        // correct right at a pole, where every phi is nearly the same point).
        const float midTheta = (ti + 0.5f) * M_PIF / bucketing.thetaCount;
        const float sinTheta = std::max(0.05f, std::sin(midTheta));
        const int phiRadius = std::min(bucketing.phiCount / 2, static_cast<int>(std::ceil(1.f / sinTheta)));

        for (int dt = -1; dt <= 1; ++dt)
        {
            const int t = std::clamp(ti + dt, 0, bucketing.thetaCount - 1);
            for (int dp = -phiRadius; dp <= phiRadius; ++dp)
            {
                const int p = ((pi + dp) % bucketing.phiCount + bucketing.phiCount) % bucketing.phiCount;
                const auto& bucket = buckets[static_cast<size_t>(t) * bucketing.phiCount + p];
                outIndices.insert(outIndices.end(), bucket.begin(), bucket.end());
            }
        }
    }

private:
    SphericalBucketing bucketing;
    const std::vector<Triangle3d>* triangles;
    std::vector<std::vector<int>> buckets;
};

#endif // SPHERE_HPP
