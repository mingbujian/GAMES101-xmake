//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f UniformHemisphereSample()
{
    float u1 = get_random_float();
    float u2 = get_random_float();

    float z = u1;
    float r = std::sqrt(1.0 - z * z);
    float phi = 2 * M_PI * u2;
    float x = r * std::cos(phi);
    float y = r * std::sin(phi);
    return Vector3f(x, y, z);
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ori_ray, int depth) const
{
    const int Value = 1;
    if (1 == Value)
    {
        Vector3f L_dir, L_indir;
        Intersection p_inter = Scene::intersect(ori_ray);

        // 与场景没有交，直接返回
        if (!p_inter.happened)
        {
            return {};
        }

        // 交点是否为光源
        if (p_inter.m->hasEmission())
        {
            return p_inter.m->getEmission();
        }

        // 物体的直接光照
        Intersection light_pos;
        float light_pdf;
        sampleLight(light_pos, light_pdf);

        //物体的位置
        Vector3f P = p_inter.coords;

        // 光源的位置
        Vector3f Light_x = light_pos.coords;

        // 入射方向
        Vector3f wo = ori_ray.direction;

        //光源方向
        Vector3f ws = (Light_x - P).normalized();

        // p点的法向量
        Vector3f N = p_inter.normal;

        // 光源点的法向量
        Vector3f NN = light_pos.normal;

        // 两点间的距离的平方
        float distance_2 = dotProduct((Light_x - P), (Light_x - P));

        float distance = (Light_x - P).norm();

        // 光和p点间 是否有障碍物
        Ray p_to_light(P, ws);
        Intersection p_to_light_inter = Scene::intersect(p_to_light);

        if (p_to_light_inter.happened && std::abs(p_to_light_inter.distance - distance) < EPSILON)
        {
            Vector3f L_i = light_pos.emit;
            Vector3f f_r = p_inter.m->eval(wo, ws, N);
            float cos_theta = dotProduct(ws, N);
            float cos_theta_light = dotProduct(-ws, NN);

            L_dir = L_i * f_r * cos_theta * cos_theta_light / (distance* distance) / light_pdf;
        }

        // 物体的间接光照
        if (get_random_float() > RussianRoulette)
        {
            return L_dir;
        }

        Vector3f wi = p_inter.m->sample(wo, N);
        wi = wi.normalized();

        Ray p_to_object_q(P, wi);
        Intersection p_to_object_q_inter = Scene::intersect(p_to_object_q);

        // 打到的是非光源
        if (p_to_object_q_inter.happened && !p_to_object_q_inter.m->hasEmission())
        {
            Vector3f f_r = p_inter.m->eval(wo, wi, N);
            float cos_theta = dotProduct(wi, N);
            float hemisphere_pdf = p_inter.m->pdf(wo, wi, N);

            L_indir = castRay(p_to_object_q, 0) * f_r * cos_theta / hemisphere_pdf / RussianRoulette;
        }

        return L_dir + L_indir;
    }
    else if(Value == 2)
    {
        Intersection intersection = intersect(ori_ray);
        if (!intersection.happened)
            return Vector3f();
        if (intersection.m->hasEmission())
            return intersection.m->getEmission();
        //直接光
        Vector3f L_dir = Vector3f(0, 0, 0);
        Intersection L_dir_Inter;
        float pdf_light = 0;
        //在场景的所有 光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度
        sampleLight(L_dir_Inter, pdf_light);
        //观察目标点
        Vector3f p = intersection.coords;
        //光源位置点
        Vector3f x = L_dir_Inter.coords;
        //入射方向
        Vector3f wo = ori_ray.direction;
        //光源方向
        Vector3f ws = (x - p).normalized();
        Ray p_2_light_ray(p, ws);
        Intersection p_2_light_inter = intersect(p_2_light_ray);
        if (p_2_light_inter.distance - (x - p).norm() > -0.005)
        {
            //给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值
            Vector3f f_r = intersection.m->eval(wo, ws, intersection.normal);
            float distance2 = dotProduct(x - p, x - p);
            // L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
            L_dir = L_dir_Inter.emit * f_r * dotProduct(ws, intersection.normal) * dotProduct(-ws, L_dir_Inter.normal) / distance2 / pdf_light;
        }
        //间接光
        Vector3f L_indir = Vector3f(0, 0, 0);

        if (get_random_float() > RussianRoulette)
            return L_dir;
        //按照该 材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
        Vector3f wi = (intersection.m->sample(wo, intersection.normal)).normalized();
        Ray L_indir_Ray(p, wi);
        Intersection L_indir_Inter = intersect(L_indir_Ray);
        if (L_indir_Inter.happened && !L_indir_Inter.m->hasEmission())
        {
            //给定一对入射、出射方向与法向量，计算 sample 方法得到该出射 方向的概率密度
            float pdf = intersection.m->pdf(wo, wi, intersection.normal);
            L_indir = castRay(L_indir_Ray, depth + 1) * L_indir_Inter.m->eval(wo, wi, intersection.normal) * dotProduct(wi, intersection.normal) / pdf / RussianRoulette;
        }
        return L_dir + L_indir;
    }
    else if (Value == 3)
    {
        Intersection p_intersection = Scene::intersect(ori_ray);

        // 1、没有打到物体
        if (!p_intersection.happened)
        {
            return {};//TODO ？
        }

        // 2、是否打到的光源
        if (p_intersection.m->hasEmission())
        {
            if (depth == 0)
            {
                return p_intersection.m->getEmission();
            }
            return {};
        }

        // 3、打到物体了，判断物体和光源是否遮挡，直接光照，弹射一次的情况

        Intersection light_pos;
        float pdf_light;
        sampleLight(light_pos, pdf_light);

        Vector3f P = p_intersection.coords;    // 光线与物体的交点，p点
        Vector3f X_Light = light_pos.coords;               // 光源的点x1

        Vector3f wo = ori_ray.direction;
        wo = wo.normalized();
        Vector3f ws = (X_Light -P).normalized();
        Vector3f N = p_intersection.normal;
        Vector3f NN = light_pos.normal;

        float Distance = (P - X_Light).norm();

        Vector3f L_dir;
        // 判断光源 x1点 到 p点之间是否被遮挡
        {
            Vector3f eye_pos = X_Light;
            Vector3f Direction = (P - X_Light).normalized();
            Ray ray(eye_pos, Direction, 0.0);
            Intersection Inter = Scene::intersect(ray);
            //if (!Inter.happened)//这个是没有打到光源
            if (Inter.happened && std::abs(Inter.distance - Distance) < EPSILON)
            {
                Vector3f L_i = light_pos.emit;
                Vector3f f_r = p_intersection.m->eval(wo, ws, N);
                float cos_theta = dotProduct(ws, N);
                float cos_theta_light = dotProduct(-ws, NN);

                L_dir = L_i * f_r * cos_theta * cos_theta_light / (Distance * Distance) / pdf_light;
            }
        }

        // # contribution from other reflectors.
        Vector3f L_indir;

        if (get_random_float() > RussianRoulette)
        {
            return L_dir;
        }

        // 4、间接光照
        // Uniformly sample the hemisphere toward wi (pdf_hemi = 1/ 2pi)
        {
            Vector3f wi = p_intersection.m->sample(wo, N);
            Vector3f p = p_intersection.coords;

            Ray p_ray(p, wi);

            Intersection q_object_intersection = Scene::intersect(p_ray);

            if (q_object_intersection.happened && !q_object_intersection.m->hasEmission())//打到的是不发光的物体
            {
                //  L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette

                Vector3f f_r = p_intersection.m->eval(wo, wi, N);
                float cos_theta = dotProduct(wi, N);
                float pdf_hemi = p_intersection.m->pdf(wo, wi, N);

                L_indir = castRay(p_ray, 0) * f_r * cos_theta / pdf_hemi / RussianRoulette;
            }
        }

        return L_dir + L_indir;
    }
}