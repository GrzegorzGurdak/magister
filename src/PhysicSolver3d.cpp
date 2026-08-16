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
}

////ChunkGrid:

ChunkGrid3d::ChunkGrid3d(int cS, float minParticleSize, Vec3 beginning, Vec3 end) :
    cellSize{ cS },
    beginning{ beginning }, end{ end },
    window_width{int(end.x - beginning.x)}, window_height{ int(end.y - beginning.y) }, window_depth{ int(end.z - beginning.z) },
    grid_width{ int((end.x - beginning.x) / cS) }, grid_height{ int((end.y - beginning.y) / cS) }, grid_depth{ int((end.z - beginning.z)/cS) }
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
        grid[i].capacity = per_chunk_capacity;
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