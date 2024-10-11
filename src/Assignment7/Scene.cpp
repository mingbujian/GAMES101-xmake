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
    // TODO Implement Path Tracing Algorithm here
    if (depth > this->maxDepth) {
        return Vector3f(0.0, 0.0, 0.0);
    }

    Intersection p_intersection = Scene::intersect(ori_ray);
    Material* m = p_intersection.m;
    Object* hitObject = p_intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    //    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;
    if (!p_intersection.happened)
    {
        return hitColor;//TODO ？
    }

    /*
    sampleLight(inter , pdf_light)
    Get x, ws, NN, emit from inter // x是当前求radiance的点(p点)，x`是光源的点（在光源上采样），ws是立体角，NN
    Shoot a ray from p to x
    If the ray is not blocked in the middle
    L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light //ws是光源x`到p点的单位向量，N是p点的法向量，NN是光源的法向量
    L_indir = 0.0
    Test Russian Roulette with probability RussianRoulette
    wi = sample(wo, N)
    Trace a ray r(p, wi)
    If ray r hit a non -emitting object at q
    L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
    Return L_dir + L_indir
    */

    Intersection light_pos;
    float pdf_light;
    sampleLight(light_pos, pdf_light);

    light_pos.emit;// 这个就是L_i，进入的Radiance

    Vector3f hitPoint = p_intersection.coords;    // 光线与物体的交点，p点
    Vector3f x1 = light_pos.coords;                     // 光源的x1点

    Vector3f wo = -1 * ori_ray.direction;
    wo = wo.normalized();
    Vector3f ws = (x1 - hitPoint).normalized();
    Vector3f N = p_intersection.normal;
    Vector3f NN = light_pos.normal;

    float Distance = (x1 - hitPoint).norm();

    Vector3f L_dir;
    // 判断光源 x1点 到 p点之间是否被遮挡
    {
        Vector3f eye_pos = x1;
        Vector3f Direction = (x1 - hitPoint).normalized();
        Ray ray(eye_pos, Direction, 0.0);
        Intersection Inter = Scene::intersect(ray);
        if (!Inter.happened)
        {
            L_dir = light_pos.emit * light_pos.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-1*ws, NN) / (Distance * Distance) / pdf_light;
        }
    }  

    // # contribution from other reflectors.
    Vector3f L_indir;

    if (get_random_float() > RussianRoulette)
    {
        return L_dir;
    }

    // Uniformly sample the hemisphere toward wi (pdf_hemi = 1/ 2pi)
    {
        // wi = sample(wo, N)
        // Trace a ray r(p, wi)
        Vector3f wi = UniformHemisphereSample();
        Vector3f p = p_intersection.coords;

        Ray p_ray(p, wi);

        Intersection q_object_intersection = Scene::intersect(p_ray);

        if (q_object_intersection.happened && !q_object_intersection.obj->hasEmit())//打到的是不发光的物体
        {
            //  L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette

            Intersection pos;
            float pdf_light;
            Object* q_hitObject = q_object_intersection.obj;
            q_hitObject->Sample(pos, pdf_light);

            Vector3f hitPoint = q_object_intersection.coords;    // 光线与物体的交点，p点

            Vector3f wo = -1 * p_ray.direction;
            Vector3f ws = p - hitPoint;
            Vector3f N = pos.normal;

            L_indir = castRay(p_ray, 0) * light_pos.m->eval(wo, ws, N) * dotProduct(wi, N) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}