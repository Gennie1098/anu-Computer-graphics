#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/core/base.hpp>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "mesh.hpp"
#include "rasterizer.hpp"

constexpr double MY_PI = 3.1415926;

// TODO: Implement this function.
Eigen::Matrix4f get_translation(const Eigen::Vector3f &translation) {
  // Calculate a transformation matrix of given translation vector.
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  trans(0, 3) = translation.x();
  trans(1, 3) = translation.y();
  trans(2, 3) = translation.z();
  return trans;
}

// TODO: Implement this function.
/**
 * @brief Get the rotation transformation matrix given rotation angle and axis.
 *
 * @param rotation_angle: rotation angle in degree.
 * @param axis: rotation axis.
 */
Eigen::Matrix4f get_rotation(float rotation_angle, const Eigen::Vector3f &axis) {
  Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
  
  //transform angle to radians
  float rotation_radians = rotation_angle * M_PI / 180.0;

  //calculate matrix elements
  float cos_angle = std::cos(rotation_radians);
  float sin_angle = std::sin(rotation_radians);
  float one_minus_cos = 1.0 - cos_angle;
  float x = axis.x();
  float y = axis.y();
  float z = axis.z();

  rotation_matrix << cos_angle + x*x*one_minus_cos,    x*y*one_minus_cos - z*sin_angle,   x*z*one_minus_cos + y*sin_angle,  0,
                     y*x*one_minus_cos + z*sin_angle,  cos_angle + y*y*one_minus_cos,     y*z*one_minus_cos - x*sin_angle,  0,
                     z*x*one_minus_cos - y*sin_angle,  z*y*one_minus_cos + x*sin_angle,   cos_angle + z*z*one_minus_cos,    0,
                     0,                                0,                                 0,                                1;

  return rotation_matrix;
}

// TODO: Implement this function
/**
 * @brief Get the view matrix by given eye position, target view position and up vector.
 *
 * @param eye_pos: location of the camera
 * @param target: the point the camera is looking at
 */
Eigen::Matrix4f look_at(Eigen::Vector3f eye_pos, Eigen::Vector3f target,
                        Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0)) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

  //forward direction
  Eigen::Vector3f forward = (target - eye_pos).normalized();

  //right vector
  Eigen::Vector3f right = forward.cross(up).normalized();

  //up vector to ensure it is orthogonal to the forward and right vectors
  Eigen::Vector3f new_up = right.cross(forward);

  //rotation matrix
  rotate << right.x(),    right.y(),    right.z(),    0,
            new_up.x(),   new_up.y(),   new_up.z(),   0,
            -forward.x(), -forward.y(), -forward.z(), 0,
            0,            0,            0,            1;

  //translation matrix
  translate << 1, 0, 0, -eye_pos.x(),
                0, 1, 0, -eye_pos.y(),
                0, 0, 1, -eye_pos.z(),
                0, 0, 0, 1;

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

  // Perspective to orthographic projection
  Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();
  persp_to_ortho << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, -(zNear + zFar), -zNear * zFar, 0, 0, -1,
      0;

  // translate to origin
  float eye_fovy_rad = eye_fovy * MY_PI / 180.0;
  float top = zNear * tan(eye_fovy_rad / 2.0);
  float bottom = -top;
  float right = top * aspect_ratio;
  float left = -right;

  Eigen::Matrix4f ortho_translate = Eigen::Matrix4f::Identity();
  ortho_translate << 1, 0, 0, -(left + right) / 2., 0, 1, 0, -(bottom + top) / 2., 0, 0, 0,
      -(zNear + zFar) / 2., 0, 0, 0, 1;

  // scale to NDC
  Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
  ortho_scale << 2. / (right - left), 0, 0, 0, 0, 2. / (top - bottom), 0, 0, 0, 0,
      2. / (zFar - zNear), 0, 0, 0, 0, 1;

  projection = ortho_scale * ortho_translate * persp_to_ortho * projection;

  return projection;
}

int main(int argc, const char **argv) {
  float angle = 0;
  Eigen::Vector3f axis = Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f translation = Eigen::Vector3f(0, 0, 0);
  bool command_line = false;
  std::string filename = "output.png";

  if (argc >= 3) {
    command_line = true;
    angle = std::stof(argv[2]);  // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    } else
      return 0;
  }

  // load house mesh
  Mesh house_mesh;
  if (!house_mesh.load_obj("../model/house.obj")) {
    std::cerr << "Failed to load house mesh." << std::endl;
    return -1;
  }

  rst::rasterizer r(700, 700);

  {
    // draw a circle first
    Eigen::Vector2i origin = Eigen::Vector2i(350, 350);
    int radius = 200;
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);
    r.draw_circle(origin, radius);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite("circle.png", image);
  }

  Eigen::Vector3f eye_pos = {0, 0, 15};

  auto pos_id = r.load_positions(house_mesh.vertices);
  auto ind_id = r.load_indices(house_mesh.faces);

  int key = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle, axis, translation));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite(filename, image);

    return 0;
  }

  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle, axis, translation));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);

    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << '\n';

    if (key == 'a') {
      eye_pos.x() -= 1;
    } else if (key == 'd') {
      eye_pos.x() += 1;
    } else if (key == 'w') {
      eye_pos.y() += 1;
    } else if (key == 's') {
      eye_pos.y() -= 1;
    } else if (key == 'q') {
      eye_pos.z() -= 1;
    } else if (key == 'e') {
      eye_pos.z() += 1;
    } else if (key == 'j') {
      angle += 10;
    } else if (key == 'k') {
      angle -= 10;
    }
    std::cout << "eye_pos: " << eye_pos.transpose() << std::endl;
  }
  return 0;
}
