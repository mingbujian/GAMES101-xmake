#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"
#include <cassert>

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)//pinned_nodes表示哪些结点是固定的
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        masses.resize(num_nodes);

        double length = (end - start).norm();
        double node_length = length / (num_nodes - 1);

        for (int i = 0; i < num_nodes; ++i)
        {
            Vector2D direction = (end - start) / length;
            Vector2D node_position = i * node_length * direction + start;

            Mass* mass_node = new Mass(node_position, node_mass, false);// 每个结点都有质量，称为质点；
            masses[i] = mass_node;
        }

        // 质点之间的线段是一个弹簧
        //vector<Spring *> springs;
        for (int i = 0,j=1; i < num_nodes && j < num_nodes; ++i, ++j)
        {
            Mass* node1 = masses[i];
            Mass* node2 = masses[j];
            Spring* spring = new Spring(node1, node2, k);

            springs.push_back(spring);
        }

        // 通过创建一系列的质点和弹簧，你就可以创建一个像弹簧一样运动的物体。

        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    bool isNaN(double x) {
        if (std::isnan(x))
        {
            return true;
        }

        return false;
    }

    bool isNaNWithVector2D(Vector2D v)
    {
        bool x = isNaN(v.x);
        bool y = isNaN(v.y);
        return  x && y;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        //算法：
        //    1）动态生成一些新的粒子
        //    2）计算每个粒子的作用力（内部的、外部的）
        //    3）根据作用力更新粒子的位置和速度
        //    4）如果粒子有存活时间，移除消亡的粒子
        //    5）渲染粒子

        for (int i = 0; i < springs.size(); ++i)
        {
            auto& s = springs[i];

            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            // 遍历所有的弹簧，对弹簧两端的质点施加正确的弹簧力。保证力的方向是正确的！对每个质点，累加所有的弹簧力。

            Mass* a = s->m1;
            Mass* b = s->m2;

            double old_length = (b->start_position - a->start_position).norm();
            double now_length = (b->position - a->position).norm();

            assert(!std::isinf(old_length));
            assert(!std::isinf(now_length));

            Vector2D direction = (b->position - a->position)/ now_length;
            Vector2D force_b_a = s->k * direction * (now_length - old_length);//b对a的力

            isNaNWithVector2D(force_b_a);

            a->forces += force_b_a;
            b->forces += -1 * force_b_a;

        }

        for (int i = 0; i < masses.size(); ++i)
        {
            auto& m = masses[i];

            float k_d = 0.001;
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                //外部的力，抵抗运动
                m->forces += -k_d * m->velocity;

                //加入内部摩擦力


                Vector2D a_t = (gravity + m->forces) / m->mass;
                Vector2D v_t = m->velocity;
                Vector2D v_t_1 = v_t + delta_t * a_t;

                m->velocity = v_t_1;

                Vector2D p_t_1 = m->position + v_t * delta_t;// for explicit method
                Vector2D p_t_1_s = m->position + v_t_1 * delta_t;// for semi-implicit method

                //isNaNWithVector2D(p_t_1_s);

                m->last_position = m->position;
                m->position = p_t_1_s;

                // TODO (Part 2): Add global damping 阻尼如何添加，在速度的反方向做一个很小的差值来抵抗
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        // Position-Based / Verlet Integration（非物理的方法）
        // 只通过调整它的不同位置，使得它最后能够满足某种限制
        // 可以认为弹簧确实是弹簧，但只要它被拉开了某个位置后，可以通过非物理的方法直接改变它的位置，使其立刻回到原状。可以认为它是一个劲度系数无限大的弹簧

        // x(t+1) = x(t) + [x_t - x(t-1)] + a(t)*dt*dt
        // 加入阻尼，x(t+1) = x(t) + (1-damping_factor)*[x_t - x(t-1)] + a(t)*dt*dt

        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)

            //auto& s = springs[i];

            Mass* a = s->m1;
            Mass* b = s->m2;

            double old_length = (b->start_position - a->start_position).norm();
            double now_length = (b->position - a->position).norm();

            assert(!std::isinf(old_length));
            assert(!std::isinf(now_length));

            Vector2D direction = (b->position - a->position) / now_length;
            Vector2D force_b_a = s->k * direction * (now_length - old_length);//b对a的力

            isNaNWithVector2D(force_b_a);

            a->forces += force_b_a;
            b->forces += -1 * force_b_a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass

                Vector2D a_t = (m->forces + gravity) / m->mass;

                Vector2D x_t0 = m->last_position;
                Vector2D x_t1 = m->position;

                // TODO (Part 4): Add global Verlet damping
                double  damping_factor = 0.00005;
                Vector2D x_t2 = x_t1 + (1- damping_factor)*(x_t1 - x_t0) + a_t * delta_t * delta_t;

                m->last_position = m->position;
                m->position = x_t2;

            }

            m->forces = Vector2D(0, 0);
        }
    }
}
