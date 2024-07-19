#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "Eigen/Dense"
// #include "Eigen/src/Core/Matrix.h"
#include "OBJ_Loader.h"
#include "Shader.hpp"
#include "Texture.hpp"
#include "Triangle.hpp"
#include "global.hpp"
#include "rasterizer.hpp"

Eigen::Matrix4f get_rotation(float rotation_angle, const Eigen::Vector3f &axis) {
    // Calculate a rotation matrix from rotation axis and angle.
    // Note: rotation_angle is in degree.
    Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();

    float rotation_angle_rad = rotation_angle * MY_PI / 180.0;
    float cos_theta = cos(rotation_angle_rad);
    float sin_theta = sin(rotation_angle_rad);

    Eigen::Vector3f axis_ = axis.normalized();
    Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f ux;
    ux << 0, -axis_.z(), axis_.y(), axis_.z(), 0, -axis_.x(), -axis_.y(), axis_.x(), 0;

    Eigen::Matrix3f rotation_matrix_3x3 =
        cos_theta * identity + (1 - cos_theta) * (axis_ * axis_.transpose()) + sin_theta * ux;
    rotation_matrix.block<3, 3>(0, 0) = rotation_matrix_3x3;

    return rotation_matrix;
}

Eigen::Matrix4f get_translation(const Eigen::Vector3f &translation) {
    // Calculate a transformation matrix of given translation vector.
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans(0, 3) = translation.x();
    trans(1, 3) = translation.y();
    trans(2, 3) = translation.z();
    return trans;
}

Eigen::Matrix4f look_at(Eigen::Vector3f eye_pos, Eigen::Vector3f target,
                        Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0)) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Vector3f z = (eye_pos - target).normalized();
    Eigen::Vector3f x = up.cross(z).normalized();
    Eigen::Vector3f y = z.cross(x).normalized();

    Eigen::Matrix4f rotate;
    rotate << x.x(), x.y(), x.z(), 0, y.x(), y.y(), y.z(), 0, z.x(), z.y(), z.z(), 0, 0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

    view = rotate * translate * view;
    return view;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    view = look_at(eye_pos, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0));

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, const Eigen::Vector3f &axis,
                                 const Eigen::Vector3f &translation) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotation = get_rotation(rotation_angle, axis);

    Eigen::Matrix4f trans = get_translation(translation);

    model = trans * rotation * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fovy, float aspect_ratio, float zNear, float zFar) {
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float eye_fovy_rad = eye_fovy * MY_PI / 180.0;
    float top = zNear * tan(eye_fovy_rad / 2.0);
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;

    projection << zNear / right, 0, 0, 0, 0, zNear / top, 0, 0, 0, 0, (zNear + zFar) / (zNear - zFar),
        2 * zNear * zFar / (zNear - zFar), 0, 0, -1, 0;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload &payload) {
    return payload.position;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f &vec, const Eigen::Vector3f &axis) {
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}


Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload) {
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture) {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        
        Eigen::Vector2f texture_coords = payload.tex_coords;
        // Clamp the texture coordinates
        texture_coords[0] = std::clamp(texture_coords[0], 0.0f, 1.0f);
        texture_coords[1] = std::clamp(texture_coords[1], 0.0f, 1.0f);

        return_color = payload.texture->getColor(texture_coords.x(), texture_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{10, 10, 10};

    float p = 150;

    std::vector<light> lights = payload.view_lights;
    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);  // cwiseProduct--dot product

    for (auto &light : lights) {
        // TODO: For each light source in the code, calculate what the *ambient*,
        // *diffuse*, and *specular* components are. Then, accumulate that result on the
        // *result_color* object.
        
        //Caculate light & view vector
        Eigen::Vector3f light_vector = (light.position - point).normalized();
        Eigen::Vector3f view_vector = (-point).normalized();
        Eigen::Vector3f bi_vector = (view_vector + light_vector).normalized();

        // Caculate distance from light to ground
        float distance = (light.position - point).norm();

        // Calculate diffusive
        float diffusiveComponent = std::max(0.0f, normal.dot(light_vector));
        Eigen::Vector3f Ld = diffusiveComponent * kd.cwiseProduct(light.intensity) / (distance * distance);

        // Calculate specular
        float specularComponent = std::max(0.0f, normal.dot(bi_vector));
        specularComponent = pow(specularComponent, p);
        Eigen::Vector3f Ls = specularComponent * ks.cwiseProduct(light.intensity) / (distance * distance);

        // Accumlate result color
        result_color += (Ld + Ls);
    }
    result_color += La;

    return result_color * 255.f;
}

// int main(int argc, const char **argv) {
//     std::vector<Triangle *> TriangleList;

//     std::string filename = "output_scene.png";
//     rst::Shading shading = rst::Shading::Phong;
//     objl::Loader Loader;
//     std::string obj_path = "../models/";

//     // Load .obj File
//     bool loadout = Loader.LoadFile(obj_path + "scene.obj");
//     for (auto mesh : Loader.LoadedMeshes) {
//         for (int i = 0; i < mesh.Vertices.size(); i += 3) {
//             Triangle *t = new Triangle();
//             for (int j = 0; j < 3; j++) {
//                 t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y,
//                                          mesh.Vertices[i + j].Position.Z, 1.0));
//                 t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y,
//                                          mesh.Vertices[i + j].Normal.Z));
//                 t->setTexCoord(
//                     j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
//             }
//             TriangleList.push_back(t);
//         }
//     }

//     rst::rasterizer r(700, 700);

//     auto texture_path = "scene.png"; 
//     r.set_texture(Texture(obj_path + texture_path));

//     std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;
//     r.set_vertex_shader(vertex_shader);
//     r.set_fragment_shader(active_shader);

//     Eigen::Vector3f eye_pos = {-20, 5, 20};
//     Eigen::Vector3f target = {0, 0, 0}; 
//     eye_pos.z() /= 6; // Zoom in
//     eye_pos.x() -= 5; // Move to the left
//     eye_pos.y() += 5; // Raise camera up
//     target.z() = -5; // Z value of target must be lower than Z value of eye_pos to get the camera looking down

//     auto l1 = light{{-5, 5, 5}, {50, 50, 50}};
//     auto l2 = light{{-20, 20, 0}, {100, 100, 100}};
//     std::vector<light> lights = {l1, l2};

//     r.set_model(get_model_matrix(0, {0, 1, 0}, {0, 0, 0}));
//     r.set_view(get_view_matrix(eye_pos));
//     r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
//     r.set_lights(lights);

//     r.clear(rst::Buffers::Color | rst::Buffers::Depth);
//     r.draw(TriangleList, true, shading, true);
//     cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
//     image.convertTo(image, CV_8UC3, 1.0f);
//     cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
//     cv::imwrite(filename, image);

//     return 0;
// }

int main(int argc, const char **argv) {
    std::vector<Triangle *> TriangleList;
    std::string filename = "output_scene.png";
    rst::Shading shading = rst::Shading::Phong;
    objl::Loader Loader;
    std::string obj_path = "../models/";

    bool loadout = Loader.LoadFile(obj_path + "scene.obj");
    for (auto mesh : Loader.LoadedMeshes) {
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            Triangle *t = new Triangle();
            for (int j = 0; j < 3; j++) {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y,
                                         mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y,
                                         mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(
                    j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    Eigen::Vector3f eye_pos = {-20, 5, 20};
    Eigen::Vector3f light_pos = {-5, 20, 20}; 
    auto l1 = light{light_pos, {500, 500, 500}}; 

    rst::rasterizer scene_rasterizer(700, 700);
    scene_rasterizer.set_view(get_view_matrix(eye_pos));
    scene_rasterizer.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
    scene_rasterizer.set_lights(std::vector<light>{l1});
    scene_rasterizer.set_texture(Texture(obj_path + "scene.png"));
    scene_rasterizer.set_vertex_shader(vertex_shader);
    scene_rasterizer.set_fragment_shader(texture_fragment_shader);

    rst::rasterizer shadow_rasterizer(700, 700);
    shadow_rasterizer.set_view(get_view_matrix(light_pos));
    shadow_rasterizer.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

    // Render shadow map
    shadow_rasterizer.clear(rst::Buffers::Depth);
    for (auto &tri : TriangleList) {
        shadow_rasterizer.draw(TriangleList);
    }

    // Lấy shadow map
    std::vector<float> shadow_map = shadow_rasterizer.depth_buffer();
    for (int i = 0; i < shadow_map.size(); ++i) {
        std::cout << "Depth at pixel " << i << ": " << shadow_map[i] << std::endl;
    }

    // Render scene với shadow map
    scene_rasterizer.clear(rst::Buffers::Color | rst::Buffers::Depth);
    scene_rasterizer.set_shadow_buffer(shadow_map);
    for (auto &tri : TriangleList) {
        scene_rasterizer.draw(TriangleList);
    }

    cv::Mat image(700, 700, CV_32FC3, scene_rasterizer.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imwrite(filename, image);

    return 0;
}
