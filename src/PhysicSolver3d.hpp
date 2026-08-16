#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <functional>
#include <SFML/Graphics.hpp>


#include <SFML/OpenGL.hpp>
#include <gl/glu.h>
#include <GL/glut.h>

#include "PhysicBody3d.h"
#include "PhysicLink3d.h"

#include <omp.h>

constexpr int chunkCapacity = 9;

struct Chunk3d{
    PhysicBody3d* objects[chunkCapacity];
    uint8_t size{ 0 };
    inline void push_back(PhysicBody3d* obj) {
        if(size == chunkCapacity){
            // std::cout << "Chunk is full" << std::endl;
            return;
        }
        objects[size] = obj;
        size++;
    }

    inline void clear() {
        size = 0;
    }

    PhysicBody3d* operator[](int i) {
        return objects[i];
    }
    const PhysicBody3d* operator[](int i) const {
        return objects[i];
    }

    PhysicBody3d** begin() {
        return objects;
    }
    PhysicBody3d* const * begin() const {
        return objects;
    }
    PhysicBody3d** end() {
        return objects + size;
    }
    const PhysicBody3d* const * end() const {
        return objects + size;
    }
};

class ChunkGrid3d {
public:
    ChunkGrid3d(int cS, Vec3 beginning, Vec3 end);

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

    enum { FUNC, NONE, DEFAULT, LAMBDA } collision_type{ DEFAULT };
    void (*collision_function)(PhysicBody3d*, PhysicBody3d*);
    std::function<void(PhysicBody3d*, PhysicBody3d*)> collision_lambda;

};

class PhysicSolver3d{
public:
    PhysicSolver3d(ChunkGrid3d g) : grid { g } {}
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
    void drawSphere(const Vec2 pos, const float radius, const sf::Color color) const{
    }
protected:
    const PhysicSolver3d& physicSolver;
    GLuint sphereList;
};