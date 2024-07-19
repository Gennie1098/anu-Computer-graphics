//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <sstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "Material.hpp"
#ifdef _OPENMP
    #include <omp.h>
#endif


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 1e-2;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    std::cout << "SPP: " << scene.spp << "\n";

    float progress = 0.0f;

#pragma omp parallel for num_threads(8) // use multi-threading for speedup if openmp is available
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {

            int m = i + j * scene.width;
            if(scene.spp==1){
                // TODO: task 1.2 pixel projection

                // Calculate x, y 
                float x = (2 * (i + 0.5) / scene.width - 1) * scale * imageAspectRatio;
                float y = (1 - 2 * (j + 0.5) / scene.height) * scale;

                // Create ray direction vector from eye position to point (x, y, 1)
                Vector3f dir = normalize(Vector3f(-x, y, 1));

                // Calculate and save the pixel color to the framebuffer
                framebuffer[m] = scene.castRay(Ray(eye_pos, dir), 0);
                
            }else {
                // TODO: task 4 multi-sampling

                for(int index = 0; index < scene.spp; index++){

                    // Random offset within the pixel for anti-aliasing
                    float randX = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                    float randY = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

                    float x = (2 * (i + randX) / scene.width - 1) * scale * imageAspectRatio;
                    float y = (1 - 2 * (j + randY) / scene.height) * scale;

                    // Create ray direction vector from eye position to point (x, y, 1)
                    Vector3f direction = normalize(Vector3f(-x, y, 1));

                    // Accumulate the color from each sample
                    framebuffer[m] += scene.castRay(Ray(eye_pos, direction), 0);
                }
                
                framebuffer[m] = framebuffer[m] / scene.spp;

            }
        }
        progress += 1.0f / (float)scene.height;
        UpdateProgress(progress);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    std::stringstream ss;
    ss << "binary_task" << TASK_N<<".ppm";
    std::string str = ss.str();
    const char* file_name = str.c_str();
    FILE* fp = fopen(file_name, "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
