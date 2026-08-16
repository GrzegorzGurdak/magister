#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <functional>
#include <memory>
#include <SFML/Graphics.hpp>


#include <SFML/OpenGL.hpp>
#include <gl/glu.h>
#include <GL/glut.h>

#include "PhysicBody3d.h"
#include "PhysicLink3d.h"
#include "planet.hpp"

#include <omp.h>

// Points into ChunkGrid3d::pool, a single contiguous slab shared by every chunk
// (laid out in the same x-fastest order as the grid) instead of each chunk owning
// its own heap allocation. Keeps neighbor-chunk traversal cache-friendly and lets
// per-chunk capacity be sized at runtime from cellSize/particle radius.
struct Chunk3d{
    PhysicBody3d** data{ nullptr };
    int capacity{ 0 };
    int size{ 0 };

    inline void push_back(PhysicBody3d* obj) {
        if (size >= capacity) {
            // std::cout << "Chunk is full" << std::endl;
            return;
        }
        data[size] = obj;
        size++;
    }

    inline void clear() {
        size = 0;
    }

    PhysicBody3d* operator[](int i) {
        return data[i];
    }
    const PhysicBody3d* operator[](int i) const {
        return data[i];
    }

    PhysicBody3d** begin() {
        return data;
    }
    PhysicBody3d* const * begin() const {
        return data;
    }
    PhysicBody3d** end() {
        return data + size;
    }
    const PhysicBody3d* const * end() const {
        return data + size;
    }
};

class ChunkGrid3d {
public:
    ChunkGrid3d(int cS, float minParticleSize, Vec3 beginning, Vec3 end);

    void assignGrid(std::vector<PhysicBody3d*>& obj);
    void updateChunkSize(PhysicBody3d* obj);

    ///make sure to reassign grid "assignGrid()", before using
    void update_collision();
    void update_collision_mt();

    void solve_collision(Chunk3d& central, Chunk3d& neigh);

    int count();

    void set_collision_def() { collision_type = DEFAULT; }
    void set_collision() { collision_type = NONE; }
    void set_collision(void (*fun)(PhysicBody3d*, PhysicBody3d*)) { collision_function = fun; collision_type = FUNC; }
    void set_collision(std::function<void(PhysicBody3d*, PhysicBody3d*)> fun) { collision_lambda = fun; collision_type = LAMBDA; }

    std::vector<Chunk3d>& getGrid() { return grid; }
    const std::vector<Chunk3d>& getGrid() const { return grid; }

protected:
    std::vector<Chunk3d> grid;
    std::vector<PhysicBody3d*> pool; // backing storage for every Chunk3d::data slice
    int per_chunk_capacity{ 0 };
    float min_particle_radius{ 1.f };
    Vec3 beginning;
    Vec3 end;
    int cellSize;
    int grid_width;
    int grid_height;
    int grid_depth;
    int window_width;
    int window_height;
    int window_depth;

    inline int array_index(int x, int y, int z) const { return x + (y + z * grid_height) * grid_width; }

    // Resizes `pool` for the current grid_width*height*depth and cellSize/min_particle_radius
    // ratio, then repoints every Chunk3d::data at its slice. Only needed when the grid's shape
    // or the smallest tracked particle changes, not on every frame.
    void rebuild_pool();

    enum { FUNC, NONE, DEFAULT, LAMBDA } collision_type{ DEFAULT };
    void (*collision_function)(PhysicBody3d*, PhysicBody3d*);
    std::function<void(PhysicBody3d*, PhysicBody3d*)> collision_lambda;

};

class PhysicSolver3d{
public:
    // thetaHeightSamples/phiHeightSamples size the SphereHeightField baked for
    // update_planet_collision_heightfield(); ignored if planet is null.
    PhysicSolver3d(ChunkGrid3d g, const Planet* planet = nullptr, int thetaHeightSamples = 128*4, int phiHeightSamples = 256*4)
        : grid{ std::move(g) }, planet{ planet },
          planetHeightField{ planet ? std::make_unique<SphereHeightField>(*planet, thetaHeightSamples, phiHeightSamples) : nullptr }
    {}
    ~PhysicSolver3d() {
        for (auto& i : objects) { delete(i); }
        for (auto& i : links) { delete(i); }
    }


    PhysicSolver3d& add(PhysicBody3d* obj);
    PhysicSolver3d& addLink(PhysicLink3d* obj) { links.push_back(obj); return *this; }

    void update(long long (&simResult)[7], const float dtime, const int sub_step = 1);

    void update_position(const float dtime);
    void update_acceleration();
    void update_constraints();
    void update_collision();
    void update_planet_collision();
    void update_planet_collision_heightfield();
    void update_links() {
        for (auto& i : links)
        {
            i->update_link();
        }
    }

    void set_acceleration(const Vec3 accVal);
    void set_acceleration(std::function<Vec3(PhysicBody3d*, std::vector<PhysicBody3d*>&)> accFun);
    void set_acceleration();

    void set_constraints_def();
    void set_constraints(std::function<Vec3(PhysicBody3d*)> conFun);
    void set_constraints();

    std::pair<bool, PhysicBody3d*> pop_from_position(const Vec3& cord);
    std::pair<bool, PhysicBody3d*> get_from_position(const Vec3& cord);

    size_t getObjectAmount() const { return objects.size(); }
    ChunkGrid3d& getChunkGrid() { return grid; }
    const ChunkGrid3d& getChunkGrid() const { return grid; }
    const std::vector<PhysicBody3d*>& getObjects() const { return objects; }

    std::vector<PhysicBody3d*> objects{};

protected:
    std::vector<PhysicLink3d*> links{};
    ChunkGrid3d grid;
    const Planet* planet;
    std::unique_ptr<SphereHeightField> planetHeightField;
    friend class PhysicDrawer;

    enum { FUNC, NONE, VALUE, DEFAULT } acceleration_type{ NONE }, constraint_type{ DEFAULT };
    Vec3 accelerationValue;
    std::function<Vec3(PhysicBody3d*, std::vector<PhysicBody3d*>&)> acceleration_function;
    std::function<Vec3(PhysicBody3d*)> constraint_fun;
};

class PhysicDrawer3d : public sf::Drawable {
public:
    PhysicDrawer3d(const PhysicSolver3d& ps) : physicSolver{ ps }, sphereList{ glGenLists(1) } {
        glNewList(sphereList, GL_COMPILE);
        glutSolidSphere(2.f, 10, 10);
        glEndList();
    }
    void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    // void drawSphere(const Vec2 pos, const float radius, const sf::Color color) const{
    // }
protected:
    const PhysicSolver3d& physicSolver;
    GLuint sphereList;
};