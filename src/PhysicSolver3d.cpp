#include "PhysicSolver3d.hpp"
#include <chrono>

////ChunkGrid:

ChunkGrid3d::ChunkGrid3d(int cS, int wW, int wH, int wD) :
    cellSize{ cS }, grid_width{ wW / cS }, grid_height{ wH / cS }, grid_depth{ wD/cS }, window_width{wW}, window_height{ wH }, window_depth{ wD }
{
    // int gW = wW / cS;
    // int gH = wH / cS;
    grid = std::vector<Chunk3d>(grid_width * grid_height * grid_depth);
}

void ChunkGrid3d::assignGrid(std::vector<PhysicBody3d*>& obj) {
    for (auto& i : grid) {
        i.clear();
    }

    for (auto& i : obj) {
        int x = int(i->getPos().x / cellSize);
        int y = int(i->getPos().y / cellSize);
        int z = int(i->getPos().z / cellSize);
        if (0 <= x && x < grid_width && 0 <= y && y < grid_height && 0 <= z && z < grid_depth)
            grid.at(array_index(x,y,z)).push_back(i);
    }
}

void ChunkGrid3d::updateChunkSize(PhysicBody3d* obj) {
    if (obj->getRadius() * 2 > cellSize) {
        cellSize = (int)ceil(obj->getRadius() * 2);
        grid_width = window_width / cellSize;
        grid_height = window_height / cellSize;
        grid_depth = window_depth / cellSize;
        grid = std::vector<Chunk3d>(grid_width*grid_height*grid_depth);
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
                                auto& neigh_cell = grid.at(array_index(x + i, y + j, z + k));
                                if (neigh_cell.size != 0)
                                    solve_collision(cell, neigh_cell);
                            }
            }
        }
}

void ChunkGrid3d::update_collision_mt() {
    // int split_num = 8;
    // #pragma omp parallel for
    // for (int t = 0; t < split_num; t++) {

    //     int b{ 1 + t * grid_width / split_num };
    //     int e{ ((t * 2 + 1) * grid_width - 1) / split_num / 2 + 1 };

    //     for (int x{ 1 + t*grid_width/split_num}; x < ((t * 2 + 1)* grid_width - 1) / split_num/2 + 1; x++) {
    //         if (x == grid_width - 1) break;
    //         for (int y{ 1 }; y < grid_height - 1; y++) {
    //             Chunk& cell = grid.at(x + y * grid_width);
    //             if (cell.size != 0)
    //                 for (int i{ -1 }; i < 2; i++)
    //                     for (int j{ -1 }; j < 2; j++) {
    //                         auto& neigh_cell = grid.at(x + i + (y+j) * grid_width);
    //                         if (neigh_cell.size != 0)
    //                             solve_collision(cell, neigh_cell);
    //                     }
    //         }
    //     }
    // }
    // #pragma omp parallel for
    // for (int t = 0; t < split_num; t++) {

    //     int b{ ((t * 2 + 1) * grid_width - 1) / split_num / 2 + 1 };
    //     int e{ ((t + 1) * grid_width - 1) / split_num + 1 };

    //     for (int x{ ((t * 2 + 1) * grid_width - 1) / split_num / 2 + 1 }; x < ((t + 1) * grid_width - 1) / split_num + 1; x++) {
    //         if (x == grid_width - 1) break;
    //         for (int y{ 1 }; y < grid_height - 1; y++) {
    //             Chunk& cell = grid.at(x + y * grid_width);
    //             if (cell.size != 0)
    //                 for (int i{ -1 }; i < 2; i++)
    //                     for (int j{ -1 }; j < 2; j++) {
    //                         auto& neigh_cell = grid.at(x + i + (y+j) * grid_width);
    //                         if (neigh_cell.size != 0)
    //                             solve_collision(cell, neigh_cell);
    //                     }
    //         }
    //     }
    // }
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
        update_collision();
        //grid.update_collision();
        //grid.update_collision_mt();//multi_thread
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