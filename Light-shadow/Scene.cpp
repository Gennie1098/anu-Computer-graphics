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
            pos.happened=true;  // area light that has emission exists
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


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // Limit depth to avoid infinite recursion 
    const int maxDepth = 30;
    if (depth > maxDepth)
    {
        return Vector3f(0, 0, 0);
    }
    
    Vector3f hitColor = Vector3f(0);
    auto inter = intersect(ray);
    if (!inter.happened)return backgroundColor;

    Vector3f hitPoint = inter.coords;
    Vector3f N = inter.normal; // normal
    Vector2f st = inter.tcoords; // texture coordinates
    Vector3f dir = ray.direction;

    if (inter.material->m_type == EMIT) 
    {
        return inter.material->m_emission;
    } 
    else if (inter.material->m_type == DIFFUSE || TASK_N<3) 
    {
        Vector3f lightAmt = 0;
        Vector3f specularColor = 0;

        // sample area light
        int light_sample=4;
        for (int i = 0; i < light_sample && TASK_N >= 5; ++i) {
            Intersection lightInter;
            float pdf_light = 0.0f;
            sampleLight(lightInter, pdf_light);  // sample a point on the area light
            // TODO: task 5 soft shadow

            //Calculate the direction from the intersection point to the light source
            Vector3f lightDir = (lightInter.coords - hitPoint).normalized();
            float distance = (lightInter.coords - hitPoint).norm();
            Ray shadowRay(hitPoint, lightDir);

            // Check to see if the shadow ray is covered by any object
            Intersection shadowInter = intersect(shadowRay);
            if (!shadowInter.happened || shadowInter.obj->hasEmit()) {
                float cosTheta = dotProduct(lightDir, N);
                lightAmt += inter.material->Kd * inter.obj->evalDiffuseColor(st) * lightInter.material->getEmission() * lightInter.material->eval(lightDir, N) * cosTheta / (distance * distance * pdf_light);
            }
          
        }
    
        // Average the light amount based on light samples
        lightAmt = lightAmt / light_sample;

        // TODO: task 1.3 Basic shading
        for (auto &light : lights) {
            Vector3f lightDir = (light->position - hitPoint).normalized();
            Vector3f reflectDir = reflect(-lightDir, N).normalized();
            bool inShadow = false;

            // Send a shadow ray to check if the point is covered
            Ray shadowRay(hitPoint + N * EPSILON, lightDir);
            inShadow = intersect(shadowRay).happened;

            // Calculate diffuse light and shadows
            lightAmt += !inShadow ? inter.material->Kd * std::max(0.0f, dotProduct(lightDir, N)) * light->intensity : Vector3f(0);
            specularColor += inter.material->Ks * pow(std::max(0.0f, -dotProduct(reflectDir, ray.direction)), inter.material->specularExponent) * light->intensity;
        }

        hitColor = lightAmt * inter.obj->evalDiffuseColor(st) + specularColor;
    }

    else if (inter.material->m_type == GLASS && TASK_N>=3) {
        // TODO: task 3 glass material
        
        float kr = fresnel(ray.direction, N, inter.material->ior);

        // Create reflected rays
        Vector3f reflectVec = reflect(ray.direction, N).normalized();
        Ray reflecRay(hitPoint + N * EPSILON, reflectVec);

        // Create refracted rays
        Vector3f refractVec = refract(ray.direction, N, inter.material->ior).normalized();
        Vector3f refractOrigin = dotProduct(refractVec, N) < 0 ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
        Ray refractRay(refractOrigin, refractVec);

        // Call recursively to get the reflection and refraction colors
        Vector3f reflectColor = castRay(reflecRay, depth + 1);
        Vector3f refractColor = castRay(refractRay, depth + 1);

        // Color matching based on Fresnel coeff
        hitColor = kr * reflectColor + (1 - kr) * refractColor;
    }

    return hitColor;
}