#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/core/base.hpp>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "mesh.hpp"
#include "rasterizer.hpp"

constexpr double MY_PI = 3.1415926;

// TODO: Copy from task3
Eigen::Matrix4f get_translation(const Eigen::Vector3f &translation) {
  // Calculate a transformation matrix of given translation vector.
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  trans(0, 3) = translation.x();
  trans(1, 3) = translation.y();
  trans(2, 3) = translation.z();
  return trans;
}

// TODO: Copy from task3
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
Eigen::Matrix4f get_scaling(const Eigen::Vector3f &scaling) {
  Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
  scale(0, 0) = scaling.x();
  scale(1, 1) = scaling.y();
  scale(2, 2) = scaling.z();
  return scale;
}

// TODO: Copy from task3
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

  // up vector to ensure it is orthogonal to the forward and right vectors
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

Mesh create_leg_parts(const Mesh& cube,
                       Eigen::Vector3f scale_vector,
                       float angle, Eigen::Vector3f axis,
                       Eigen::Vector3f translate_vector) {
    return cube.transform(get_scaling(scale_vector) * get_rotation(angle, axis) * get_translation(translate_vector));    
}

Mesh create_legs(const Mesh& cube, bool is_front, bool is_right) {
  Mesh leg;

  if (is_front)
  {
    //front legs
    float side_factor = is_right ? 1.0 : -1.0;
    leg = create_leg_parts(cube, {0.7, 3.8, 0.5}, 0, {0, 0, 0}, {1.3, -0.35, 2.2 * side_factor}) + //leg
          create_leg_parts(cube, {0.7, 0.4, 0.5}, 0, {0, 0, 0}, {1.7, -8.0, 2.2 * side_factor}); //foot
  } else {
    //back legs
    float side_factor = is_right ? 1.0 : -1.0;
    leg = create_leg_parts(cube, {0.8, 1.5, 0.7}, -10.0, {0, 0, 1.0}, {-3.0, -0.6, 1.5 * side_factor}) + //leg thigh
          create_leg_parts(cube, {0.7, 1.0, 0.5}, -25.0, {0, 0, 1.0}, {-3.0, -2.5, 2.0 * side_factor}) + //leg shank 1
          create_leg_parts(cube, {0.5, 2.0, 0.5}, 0, {0, 0, 0}, {-5.7, -1.1, 2.0 * side_factor}) + //leg shank 2
          create_leg_parts(cube, {0.7, 0.4, 0.5}, 0, {0, 0, 0}, {-3.7, -8.0, 2.02 * side_factor}); //foot
  }

  return leg;
}

// TODO: Implement this function.
Mesh create_dog_mesh(const Mesh &cube) {
  Mesh dog;
  
  // Transform the cube mesh to build the dog.

  // You can use Mesh::transform to transform the cube mesh.
  // Mesh new_mesh = cube.transform(transform_matrix);
  // You can use "+" operator to add meshes together.
  // dog_mesh = cube + new_mesh;
  // Transform the cube mesh to build the dog.
  // Body: Scale and place the body
  
  // Body: upper body + lower body + tail in the middle
  Mesh body;

  Eigen::Matrix4f upper_body_scale = get_scaling(Eigen::Vector3f(3.0, 2.0, 2.0));
  Eigen::Matrix4f upper_body_rotate = get_rotation(-10.0, Eigen::Vector3f(0, 0, 1.0));
  Mesh upper_body = cube.transform(upper_body_scale * upper_body_rotate);

  Eigen::Matrix4f lower_body_scale = get_scaling(Eigen::Vector3f(2.0, 1.5, 1.5));
  Eigen::Matrix4f lower_body_rotate = get_rotation(5.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f lower_body_translate = get_translation(Eigen::Vector3f(-1.0, 0.2, 0));
  Mesh lower_body = cube.transform(lower_body_scale * lower_body_rotate * lower_body_translate);

  Eigen::Matrix4f tail1_scale = get_scaling(Eigen::Vector3f(0.7, 0.2, 0.2));
  Eigen::Matrix4f tail1_rotate = get_rotation(30.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f tail1_trans = get_translation(Eigen::Vector3f(-2.5, 5.0, 0));

  Mesh tail = cube.transform(tail1_scale * tail1_rotate * tail1_trans);
  
  body = upper_body + lower_body + tail;
  
  dog = dog + body;

  // Head: skull + neck + mouth + 2 ears + nose
  Mesh head;

  Eigen::Matrix4f neck_scale = get_scaling(Eigen::Vector3f(1.5, 1.0, 1.0));
  Eigen::Matrix4f neck_rotate = get_rotation(-10.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f neck_translate = get_translation(Eigen::Vector3f(0.5, 1.5, 0));
  Mesh neck = cube.transform(neck_scale * neck_rotate * neck_translate);

  Eigen::Matrix4f skull_scale = get_scaling(Eigen::Vector3f(1.5, 1.0, 1.5));
  Eigen::Matrix4f skull_trans = get_translation(Eigen::Vector3f(0.8, 2.2, 0));
  Mesh skull = cube.transform(skull_scale * skull_trans);

  Eigen::Matrix4f mouth_rotate = get_rotation(-10.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f mouth_translate = get_translation(Eigen::Vector3f(2.0, 2.0, 0));
  Mesh mouth = cube.transform(mouth_rotate * mouth_translate);

  Eigen::Matrix4f right_ear_scale = get_scaling(Eigen::Vector3f(0.5, 1.0, 0.2));
  Eigen::Matrix4f right_ear_rotate = get_rotation(-10.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f right_ear_translate = get_translation(Eigen::Vector3f(1.0, 2.5, 4.2));
  Mesh right_ear = cube.transform(right_ear_scale * right_ear_rotate * right_ear_translate);

  Eigen::Matrix4f left_ear_scale = get_scaling(Eigen::Vector3f(0.5, 1.0, 0.2));
  Eigen::Matrix4f left_ear_rotate = get_rotation(-10.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f left_ear_translate = get_translation(Eigen::Vector3f(1.0, 2.5, -4.2));
  Mesh left_ear = cube.transform(left_ear_scale * left_ear_rotate * left_ear_translate);

  Eigen::Matrix4f nose_scale = get_scaling(Eigen::Vector3f(0.3, 0.3, 0.3));
  Eigen::Matrix4f nose_rotate = get_rotation(45.0, Eigen::Vector3f(0, 0, 1.0));
  Eigen::Matrix4f nose_translate = get_translation(Eigen::Vector3f(11.5, -2.3, 0));
  Mesh nose = cube.transform(nose_scale * nose_rotate * nose_translate);
  
  head = neck + skull + mouth + nose + right_ear + left_ear;
  dog = dog + head;

  // 4 Legs
  Mesh legs;

  // Eigen::Matrix4f front_legs_scale = get_scaling(Eigen::Vector3f(0.7, 3.8, 0.5));
  // Eigen::Matrix4f front_legs_translate = get_translation(Eigen::Vector3f(1.3, -0.35, 2.2));

  // Eigen::Matrix4f front_foot_scale = get_scaling(Eigen::Vector3f(0.7, 0.4, 0.5));
  // Eigen::Matrix4f front_foot_translate = get_translation(Eigen::Vector3f(1.7, -8.0, 2.2));
  // Mesh right_front_leg = cube.transform(front_legs_scale * front_legs_translate) + 
  //                        cube.transform(front_foot_scale * front_foot_translate);

  // legs = legs + right_front_leg;

  // Eigen::Matrix4f back_legs_thigh_scale = get_scaling(Eigen::Vector3f(0.8, 1.5, 0.7));
  // Eigen::Matrix4f back_legs_thigh_rotate = get_rotation(-10.0, Eigen::Vector3f(0, 0, 1.0));
  // Eigen::Matrix4f back_legs_thigh_translate = get_translation(Eigen::Vector3f(-3.0, -0.6, 1.5));

  // Eigen::Matrix4f back_legs_shank1_scale = get_scaling(Eigen::Vector3f(0.7, 1.0, 0.5));
  // Eigen::Matrix4f back_legs_shank1_rotate = get_rotation(-25.0, Eigen::Vector3f(0, 0, 1.0));
  // Eigen::Matrix4f back_legs_shank1_translate = get_translation(Eigen::Vector3f(-3.0, -2.5, 2.0));

  // Eigen::Matrix4f back_legs_shank2_scale = get_scaling(Eigen::Vector3f(0.5, 2.0, 0.5));
  // Eigen::Matrix4f back_legs_shank2_translate = get_translation(Eigen::Vector3f(-5.7, -1.1, 2.0));
  
  // Eigen::Matrix4f back_foot_scale = get_scaling(Eigen::Vector3f(0.7, 0.4, 0.5));
  // Eigen::Matrix4f back_foot_translate = get_translation(Eigen::Vector3f(-3.7, -8.0, 2.0));

  // Mesh right_back_leg = cube.transform(back_legs_thigh_scale * back_legs_thigh_rotate * back_legs_thigh_translate) + 
  //                       cube.transform(back_legs_shank1_scale * back_legs_shank1_rotate * back_legs_shank1_translate) +
  //                       cube.transform(back_legs_shank2_scale * back_legs_shank2_translate) +
  //                       cube.transform(back_foot_scale * back_foot_translate);

  // legs = legs + right_back_leg;

  legs = legs + create_legs(cube, 1, 1); //right front leg
  legs = legs + create_legs(cube, 1, 0); //left front leg
  legs = legs + create_legs(cube, 0, 1); //right back leg
  legs = legs + create_legs(cube, 0, 0); //left back leg
  
  dog = dog + legs;


  return dog;
}

int main(int argc, const char **argv) {
  float angle = 30;
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

  // load mesh primitives
  Mesh cube_mesh;
  cube_mesh.load_obj("../model/cube.obj");
  auto dog_mesh = create_dog_mesh(cube_mesh);
  dog_mesh.save_obj("dog.obj");

  rst::rasterizer r(700, 700);

  Eigen::Vector3f eye_pos = {0, 2, 15};

  auto pos_id = r.load_positions(dog_mesh.vertices);
  auto ind_id = r.load_indices(dog_mesh.faces);

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
