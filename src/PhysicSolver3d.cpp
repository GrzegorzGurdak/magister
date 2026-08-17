#include "PhysicSolver3d.hpp"
#include <chrono>
#include <omp.h>
#include <limits>

namespace {
struct CollisionCorrection {
    PhysicBody3d* body;
    Vec3 delta;
};

inline void accumulate_collision_pair(PhysicBody3d* first, PhysicBody3d* second, std::vector<CollisionCorrection>& corrections) {
    if (first == second) {
        return;
    }

    if (!first->isKinematic && !second->isKinematic) {
        return;
    }

    Vec3 diff = first->getPos() - second->getPos();
    float diffLen = diff.length();
    if (diffLen <= std::numeric_limits<float>::epsilon()) {
        diff = Vec3{ 1.f, 0.f, 0.f };
        diffLen = 1.f;
    }

    float overlap = (first->getRadius() + second->getRadius()) - diffLen;
    if (overlap <= 0.f) {
        return;
    }

    Vec3 correction = diff / diffLen * (overlap * 0.25f);
    if (first->isKinematic) {
        corrections.push_back({ first, correction });
    }
    if (second->isKinematic) {
        corrections.push_back({ second, correction * -1.f });
    }
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Closest point on triangle (a,b,c) to point p. Standard region-test approach
// (Ericson, "Real-Time Collision Detection", 5.1.5).
Vec3 closest_point_on_triangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;

    const float d1 = dot(ab, ap);
    const float d2 = dot(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) {
        return a;
    }

    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp);
    const float d4 = dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) {
        return b;
    }

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp);
    const float d6 = dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) {
        return c;
    }

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

inline void accumulate_planet_collision(const Triangle3d& tri, PhysicBody3d* body, std::vector<CollisionCorrection>& corrections) {
    if (!body->isKinematic) {
        return;
    }

    const Vec3 center = body->getPos();
    const Vec3 closest = closest_point_on_triangle(center, tri.p0, tri.p1, tri.p2);

    Vec3 diff = center - closest;
    float dist = diff.length();

    const float radius = body->getRadius();
    if (dist >= radius) {
        return;
    }

    Vec3 normal;
    if (dist <= std::numeric_limits<float>::epsilon()) {
        normal = cross(tri.p1 - tri.p0, tri.p2 - tri.p0).normal();
        dist = 0.f;
    } else {
        normal = diff / dist;
    }

    // Only the particle moves (the terrain is static), but still resolve half the
    // overlap per substep rather than snapping fully onto the surface in one step -
    // matches accumulate_collision_pair's softness and avoids a single large Verlet
    // velocity spike if a fast particle tunnels deep into a triangle in one substep.
    const float overlap = radius - dist;
    corrections.push_back({ body, normal * (overlap * 0.5f) });
}
}

////ChunkGrid:

ChunkGrid3d::ChunkGrid3d(int cS, float minParticleSize, Vec3 beginning, Vec3 end) :
    beginning{ beginning }, end{ end },
    cellSize{ cS },
    grid_width{ int((end.x - beginning.x) / cS) }, grid_height{ int((end.y - beginning.y) / cS) }, grid_depth{ int((end.z - beginning.z)/cS) },
    window_width{int(end.x - beginning.x)}, window_height{ int(end.y - beginning.y) }, window_depth{ int(end.z - beginning.z) }
{
    // int gW = wW / cS;
    // int gH = wH / cS;
    grid = std::vector<Chunk3d>(grid_width * grid_height * grid_depth);
    min_particle_radius = cS / minParticleSize;
    rebuild_pool();
    std::cout << "ChunkGrid3d created: " << grid_width << "x" << grid_height << "x" << grid_depth << ", total: " << grid.size() << std::endl;
    //check if pragma omp is available
    #ifdef _OPENMP
        std::cout << "OpenMP is available, max threads: " << omp_get_max_threads() << std::endl;
    #else
        std::cout << "OpenMP is not available" << std::endl;
    #endif
}

// Sizes `pool` so each chunk can hold as many particle-diameter spheres as could
// plausibly fit in a cellSize^3 volume (with slack for imperfect/random packing),
// then repoints every Chunk3d::data at its slice of the slab. Chunks stay laid out
// in the same order as `grid`, so scanning a cell plus its neighbors (the hot path
// in update_collision/update_collision_mt) stays close together in memory instead
// of chasing a separate heap allocation per chunk.
void ChunkGrid3d::rebuild_pool() {
    const float diameter = std::max(1.f, min_particle_radius * 2.f);
    int per_axis = static_cast<int>(std::ceil(cellSize / diameter)) + 1;
    per_axis = std::clamp(per_axis, 1, 16);
    per_chunk_capacity = std::clamp(per_axis * per_axis * per_axis * 2, 9, 2048);

    pool.assign(static_cast<size_t>(grid.size()) * per_chunk_capacity, nullptr);
    for (size_t i = 0; i < grid.size(); ++i) {
        grid[i].data = &pool[i * per_chunk_capacity];
        grid[i].capacity = per_chunk_capacity * 2;
        grid[i].size = 0;
    }

    std::cout << "ChunkGrid3d pool rebuilt: capacity/chunk=" << per_chunk_capacity
        << ", pool size=" << pool.size() << std::endl;
}

void ChunkGrid3d::assignGrid(std::vector<PhysicBody3d*>& obj) {
    for (auto& i : grid) {
        i.clear();
    }

    for (auto& i : obj) {
        int x = int((i->getPos().x - beginning.x) / cellSize);
        int y = int((i->getPos().y - beginning.y) / cellSize);
        int z = int((i->getPos().z - beginning.z) / cellSize);
        if (0 <= x && x < grid_width && 0 <= y && y < grid_height && 0 <= z && z < grid_depth)
            grid.at(array_index(x,y,z)).push_back(i);
        else {
            std::cout << "Object out of bounds: " << i->getPos().x << ", " << i->getPos().y << ", " << i->getPos().z << std::endl;
        }
    }
}

void ChunkGrid3d::updateChunkSize(PhysicBody3d* obj) {
    bool needs_rebuild = false;

    if (obj->getRadius() * 2 > cellSize) {
        cellSize = (int)ceil(obj->getRadius() * 2);
        grid_width = window_width / cellSize;
        grid_height = window_height / cellSize;
        grid_depth = window_depth / cellSize;
        grid = std::vector<Chunk3d>(grid_width*grid_height*grid_depth);
        needs_rebuild = true;
    }

    if (obj->getRadius() < min_particle_radius) {
        min_particle_radius = obj->getRadius();
        needs_rebuild = true;
    }

    if (needs_rebuild) {
        rebuild_pool();
    }
}

void ChunkGrid3d::update_collision() {
    for (int x{ 1 }; x < grid_width - 1; x++)
        for (int y{ 1 }; y < grid_height - 1; y++) {
            for (int z{ 1 }; z < grid_depth - 1; z++) {
                Chunk3d& cell = grid.at(array_index(x, y, z));
                if (cell.size != 0)
                    for (int i{ -1 }; i < 2; i++)
                        for (int j{ -1 }; j < 2; j++)
                            for (int k{ -1 }; k < 2; k++) {
                                if (k < 0 || (k == 0 && j < 0) || (k == 0 && j == 0 && i <= 0)) {
                                    continue;
                                }

                                const int nx = x + i;
                                const int ny = y + j;
                                const int nz = z + k;
                                if (nx < 1 || nx >= grid_width - 1 || ny < 1 || ny >= grid_height - 1 || nz < 1 || nz >= grid_depth - 1) {
                                    continue;
                                }

                                auto& neigh_cell = grid.at(array_index(nx, ny, nz));
                                if (neigh_cell.size != 0)
                                    solve_collision(cell, neigh_cell);
                            }
            }
        }
}

void ChunkGrid3d::update_collision_mt() {
    if (collision_type != DEFAULT) {
        update_collision();
        return;
    }

    if (grid_width < 3 || grid_height < 3 || grid_depth < 3) {
        return;
    }

    const int interior_width = grid_width - 2;
    const int thread_count = std::min(omp_get_max_threads(), interior_width);

    if (thread_count <= 0) {
        return;
    }

    struct XRange {
        int begin;
        int end;
    };

    std::vector<XRange> ranges;
    ranges.reserve(thread_count);

    const int base_width = interior_width / thread_count;
    const int remainder = interior_width % thread_count;

    int current_x = 1;
    for (int thread_index = 0; thread_index < thread_count; ++thread_index) {
        const int slice_width = base_width + (thread_index < remainder ? 1 : 0);
        ranges.push_back({ current_x, current_x + slice_width });
        current_x += slice_width;
    }

    std::vector<std::vector<CollisionCorrection>> thread_corrections(thread_count);

    #pragma omp parallel num_threads(thread_count)
    {
        const int thread_index = omp_get_thread_num();
        std::vector<CollisionCorrection>& corrections = thread_corrections[thread_index];
        const int x_begin = ranges[thread_index].begin;
        const int x_end = ranges[thread_index].end;

        for (int x = x_begin; x < x_end; ++x) {
            for (int y = 1; y < grid_height - 1; ++y) {
                for (int z = 1; z < grid_depth - 1; ++z) {
                    Chunk3d& cell = grid.at(array_index(x, y, z));
                    if (cell.size == 0) {
                        continue;
                    }

                    for (int first_index = 0; first_index < cell.size; ++first_index) {
                        PhysicBody3d* first = cell[first_index];
                        for (int second_index = first_index + 1; second_index < cell.size; ++second_index) {
                            accumulate_collision_pair(first, cell[second_index], corrections);
                        }
                    }

                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dz = -1; dz <= 1; ++dz) {
                                if (dz < 0 || (dz == 0 && dy < 0) || (dz == 0 && dy == 0 && dx <= 0)) {
                                    continue;
                                }

                                const int nx = x + dx;
                                const int ny = y + dy;
                                const int nz = z + dz;

                                if (nx < 1 || nx >= grid_width - 1 || ny < 1 || ny >= grid_height - 1 || nz < 1 || nz >= grid_depth - 1) {
                                    continue;
                                }

                                Chunk3d& neighbor = grid.at(array_index(nx, ny, nz));
                                if (neighbor.size == 0) {
                                    continue;
                                }

                                for (int first_index = 0; first_index < cell.size; ++first_index) {
                                    PhysicBody3d* first = cell[first_index];
                                    for (int second_index = 0; second_index < neighbor.size; ++second_index) {
                                        accumulate_collision_pair(first, neighbor[second_index], corrections);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for (const auto& corrections : thread_corrections) {
        for (const auto& correction : corrections) {
            correction.body->current_position += correction.delta;
        }
    }
}

void ChunkGrid3d::solve_collision(Chunk3d& central_chunk, Chunk3d& neighboring_chunk) {
    for (auto& i : central_chunk) {
        for (auto& j : neighboring_chunk)
        {
            if (&i != &j) {
                if (collision_type == DEFAULT) {
                    Vec3 diff = i->getPos() - j->getPos();
                    float diffLen = diff.length();
                    float dist = diffLen - (i->getRadius() + j->getRadius());
                    if (dist < 0) {
                        if (i->isKinematic) i->current_position -= diff / diffLen * (dist / 2) * 0.5; //squishiness
                        if (j->isKinematic) j->current_position += diff / diffLen * (dist / 2) * 0.5;
                    }
                }
                else if (collision_type == FUNC) {
                    collision_function(i, j);
                }
                else if (collision_type == LAMBDA) {
                    collision_lambda(i, j);
                }
            }
        }
    }
}

int ChunkGrid3d::count() {
    int sum{};

    for (auto& i : grid) {
        sum += i.size;
    }
    return sum;
}

////PhysicSolver:

PhysicSolver3d& PhysicSolver3d::add(PhysicBody3d* obj) {
    objects.push_back(obj);
    grid.updateChunkSize(obj);
    return *this;
}

void PhysicSolver3d::update(long long (&simResult)[7], const float dtime, const int sub_step) {
    float sub_dt = dtime / sub_step;

    for (int i = 0; i < sub_step; i++) {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        update_acceleration();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        simResult[0] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        begin = std::chrono::steady_clock::now();
        update_constraints();
        end = std::chrono::steady_clock::now();
        simResult[1] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        begin = std::chrono::steady_clock::now();
        update_links();
        end = std::chrono::steady_clock::now();
        simResult[2] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        begin = std::chrono::steady_clock::now();
        grid.assignGrid(objects);
        end = std::chrono::steady_clock::now();
        simResult[3] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        begin = std::chrono::steady_clock::now();
        //update_collision();
        //update_collision();
        //grid.update_collision();
        grid.update_collision_mt();//multi_thread
        end = std::chrono::steady_clock::now();
        simResult[4] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        begin = std::chrono::steady_clock::now();
        //update_planet_collision();//brute-force: every particle vs every surface triangle
        update_planet_collision_heightfield();//baked theta/phi surface-radius lookup
        end = std::chrono::steady_clock::now();
        simResult[6] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        begin = std::chrono::steady_clock::now();
        update_position(sub_dt);
        end = std::chrono::steady_clock::now();
        simResult[5] += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    }
}

void PhysicSolver3d::update_position(const float dtime)
{
    for (auto& i : objects)
        i->update_position(dtime);
}

void PhysicSolver3d::update_acceleration() {
    if(acceleration_type == VALUE)
        for (auto& i : objects)
            i->accelerate(accelerationValue);
    else if(acceleration_type == FUNC)
        for (auto& i : objects)
            i->accelerate(acceleration_function(i, objects));
}

void PhysicSolver3d::update_constraints() {
    if (constraint_type == DEFAULT) {
        Vec3 sphere_centre(0, 0, 0);
        float radius = 150;
        for (auto& i : objects) {
            if (i->isKinematic) {
                Vec3 diff = i->getPos() - sphere_centre;
                float diffLen = diff.length();
                if (diffLen + i->radius > radius) {
                    i->current_position -= diff / diffLen * (diffLen + i->radius - radius);
                }
                else if (diffLen < 1) {
                    i->current_position = diff.normal() * (1 + i->radius);
                }
            }
        }
    }
    else if (constraint_type == FUNC) {
        for (auto& i : objects)
            if(i->isKinematic)
                i->current_position = constraint_fun(i);
    }
}

void PhysicSolver3d::update_collision() {
    for (auto& i : objects) {
        for (auto& j : objects)
        {
            //if (&i == &j) break;
            if (&i != &j) {
                Vec3 diff = i->getPos() - j->getPos();
                float diffLen = diff.length();
                float dist = diffLen - (i->getRadius() + j->getRadius());
                if (dist < 0) {
                    i->current_position -= diff / diffLen * (dist / 2);
                    j->current_position += diff / diffLen * (dist / 2);
                }
            }
        }
    }
}

void PhysicSolver3d::update_planet_collision() {
    if (!planet || objects.empty()) {
        return;
    }

    const std::vector<Triangle3d>& triangles = planet->getSurfaceTriangles();
    if (triangles.empty()) {
        return;
    }

    const int thread_count = std::min(omp_get_max_threads(), static_cast<int>(triangles.size()));
    if (thread_count <= 0) {
        return;
    }

    std::vector<std::vector<CollisionCorrection>> thread_corrections(thread_count);

    // Split by triangle, not by particle: there are far fewer triangles than particles,
    // but each thread still brute-forces every particle against its share of triangles.
    #pragma omp parallel num_threads(thread_count)
    {
        std::vector<CollisionCorrection>& corrections = thread_corrections[omp_get_thread_num()];

        #pragma omp for schedule(static)
        for (int t = 0; t < static_cast<int>(triangles.size()); ++t) {
            const Triangle3d& tri = triangles[t];
            for (PhysicBody3d* body : objects) {
                accumulate_planet_collision(tri, body, corrections);
            }
        }
    }

    for (const auto& corrections : thread_corrections) {
        for (const auto& correction : corrections) {
            correction.body->current_position += correction.delta;
        }
    }
}

void PhysicSolver3d::update_planet_collision_heightfield() {
    if (!planet || !planetHeightField || !planetTriangleGrid || objects.empty()) {
        return;
    }

    const Vec3 center = planet->getPosition();
    const std::vector<Triangle3d>& triangles = planetTriangleGrid->allTriangles();

    // Each particle only ever touches its own current_position here (there's no
    // pairwise interaction like in accumulate_collision_pair), so this can write
    // directly without deferred corrections - splitting by particle is race-free.
    // `candidates` is declared once per thread (outside the omp for) and reused
    // across that thread's particles, so it isn't reallocated every iteration.
    #pragma omp parallel
    {
        std::vector<int> candidates;

        #pragma omp for schedule(static)
        for (int idx = 0; idx < static_cast<int>(objects.size()); ++idx) {
            PhysicBody3d* body = objects[idx];
            if (!body->isKinematic) {
                continue;
            }

            const Vec3 toBody = body->getPos() - center;
            const float dist = toBody.length();
            if (dist <= std::numeric_limits<float>::epsilon()) {
                continue;
            }

            const float radius = body->getRadius();

            // Broad phase: cheap reject via the baked (approximate) height field.
            // Generous margin since it's only used to skip particles that are
            // clearly nowhere near the surface - it never resolves a collision by
            // itself, so it doesn't need to be precise, just fast.
            const float approxSurfaceRadius = planetHeightField->surfaceRadiusAt(toBody);
            if (dist > approxSurfaceRadius + radius * 3.f) {
                continue;
            }

            // Narrow phase: the triangle grid's home bucket for this direction
            // plus its neighbors covers every triangle that could plausibly be
            // the closest one - the same "center cell + neighbors" shape
            // ChunkGrid3d uses for particle-particle collision, just bucketed by
            // direction instead of position. Resolve against the real geometry
            // (exact closest_point_on_triangle), not an interpolated estimate:
            // that's what makes this robust right at a cliff edge, where a
            // height-field-derived normal kept going wrong no matter how the
            // approximation was tuned.
            candidates.clear();
            planetTriangleGrid->queryNearby(toBody, candidates);

            float bestDist = std::numeric_limits<float>::max();
            Vec3 bestClosest{};
            for (int triIdx : candidates) {
                const Triangle3d& tri = triangles[triIdx];
                const Vec3 closest = closest_point_on_triangle(body->getPos(), tri.p0, tri.p1, tri.p2);
                const float d = (body->getPos() - closest).length();
                if (d < bestDist) {
                    bestDist = d;
                    bestClosest = closest;
                }
            }
            if (bestDist >= radius || bestDist == std::numeric_limits<float>::max()) {
                // The exact search found nothing within radius. That's the normal
                // case for a particle genuinely floating above the surface - but
                // it's also what happens when a particle is buried MORE than one
                // radius deep (the nearest candidate triangle is then farther than
                // radius away too) or when the triangle bucket neighborhood just
                // missed the true nearest triangle. Those look identical from
                // bestDist alone, so without this fallback a buried particle gets
                // zero corrective force forever - it's permanently stuck, since
                // nothing else in this function ever revisits it. The (coarser,
                // already-computed) height field can tell the difference: if the
                // particle's radial distance is less than the approximate terrain
                // height here, it's below the surface and needs to be nudged back
                // out, capped and softened the same way as a normal correction so
                // a deeply-buried particle walks back out over several substeps
                // instead of snapping out in one.
                if (dist < approxSurfaceRadius) {
                    const float buriedOverlap = std::min(approxSurfaceRadius + radius - dist, radius);
                    body->current_position += (toBody / dist) * (buriedOverlap * 0.5f);
                }
                continue;
            }

            Vec3 diff = body->getPos() - bestClosest;
            const Vec3 normal = (bestDist <= std::numeric_limits<float>::epsilon())
                ? (toBody / dist)
                : diff / bestDist;

            // Exact closest-point distance is always >= 0, so this overlap is
            // naturally bounded to at most `radius` - no separate magnitude clamp
            // needed, unlike the old height-field-plane approximation.
            const float overlap = radius - bestDist;
            body->current_position += normal * (overlap * 0.5f);
        }
    }
}

void PhysicSolver3d::set_acceleration(const Vec3 accVal) {
    acceleration_type = VALUE;
    accelerationValue = accVal;
}
void PhysicSolver3d::set_acceleration(std::function<Vec3(PhysicBody3d*, std::vector<PhysicBody3d*>&)> accFun) {
    acceleration_type = FUNC;
    acceleration_function = accFun;
}
void PhysicSolver3d::set_acceleration() {
    acceleration_type = NONE;
}

void PhysicSolver3d::set_constraints_def() {
    constraint_type = DEFAULT;
}
void PhysicSolver3d::set_constraints(std::function<Vec3(PhysicBody3d*)> conFun) {
    constraint_type = FUNC;
    constraint_fun = conFun;
}
void PhysicSolver3d::set_constraints() {
    constraint_type = NONE;
}

std::pair<bool, PhysicBody3d*> PhysicSolver3d::pop_from_position(const Vec3& cord) {
    auto found = std::find_if(objects.begin(), objects.end(), [&cord](auto& i) {return i->isHere(cord); });
    if (found != objects.end()) {
        PhysicBody3d *r = * found;
        //objects.erase(found);
        return { true, r };
    }
    return { false, &PhysicBody3d::nullPB };
}

std::pair<bool, PhysicBody3d*> PhysicSolver3d::get_from_position(const Vec3& cord) {
    auto found = std::find_if(objects.begin(), objects.end(), [&cord](auto& i) {return i->isHere(cord); });
    if (found != objects.end()) {
        PhysicBody3d *r = *found;
        objects.erase(found);
        return { true, r };
    }
    return { false, &PhysicBody3d::nullPB };
}

//PhysicDrawer:

void PhysicDrawer3d::draw(sf::RenderTarget& target, sf::RenderStates states) const {
    (void)target;
    (void)states;
    // // for (const auto& i : physicSolver.objects)
    // // {
    // //     target.draw(i->getFigure(), states);
    // // }
    // glPointSize(20.f);
    // glPointParameterf(GL_POINT_DISTANCE_ATTENUATION, 20.f);
    // glBegin(GL_POINTS);
    // for (const auto& i : physicSolver.objects)
    // {
    //     glColor3f(i->getFigureColor().r / 255.f, i->getFigureColor().g / 255.f, i->getFigureColor().b / 255.f);
    //     glVertex3f(i->getPos().x, 690 - i->getPos().y, i->getPos().z);
    // }
    // for (const auto& i : physicSolver.getChunkGrid().getGrid())
    // {
    //     for (const auto& j : i)
    //     {
    //         //target.draw(j->getFigure(), states);
    //         glColor3f(j->getFigureColor().r / 255.f, j->getFigureColor().g / 255.f, j->getFigureColor().b / 255.f);
    //         glVertex3f(j->getPos().x, 690 - j->getPos().y, j->getPos().z);
    //     }
    // }
    // glEnd();
    // for (const auto& i : physicSolver.links)
    // {
    //     target.draw(i->getFigure(), states);
    // }

    for (const auto& i : physicSolver.objects)
    {
        glColor3f(i->getFigureColor().r / 255.f, i->getFigureColor().g / 255.f, i->getFigureColor().b / 255.f);
        glPushMatrix();
        glTranslatef(i->getPos().x, i->getPos().y, i->getPos().z); //y = 690 -  i->getPos().y
        //glutSolidSphere(i->getRadius() + 1, 10, 10);
        glCallList(sphereList);
        glPopMatrix();
    }
}