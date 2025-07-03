#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotate;
    float angle = rotation_angle / 180.0f * MY_PI;
    rotate << cos(angle), -sin(angle), 0, 0,
    sin(angle), cos(angle), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

    model = rotate * model;

    return model;
}

Eigen::Matrix4f get_model_matrix_rodrigues_rotation(Eigen::Vector3f axis, float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around any given axis.
    // Then return it.

    axis.normalize();
    float angle = rotation_angle / 180.0f * MY_PI;
    float cos_angle = cos(angle);
    float sin_angle = sin(angle);

    Eigen::Matrix3f I, outer, cross, rodrigues;
    I = Eigen::Matrix3f::Identity();
    // Outer product
    outer = axis * axis.transpose();
    // Cross matrix
    cross << 0, -axis.z(), axis.y(),
             axis.z(), 0, -axis.x(),
             -axis.y(), axis.x(), 0;
    
    rodrigues = cos_angle * I + (1 - cos_angle) * outer + sin_angle * cross;

    model.block<3,3>(0,0) = rodrigues;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // Eigen::Matrix4f orthoTranslate;
    Eigen::Matrix4f ortho_scale;
    Eigen::Matrix4f persp_2_ortho;

    // Assume FOV is vertical FOV

    float fov_rad = eye_fov / 180.0f * MY_PI;
    float top_minus_bottom = 2 * tanf(fov_rad / 2) * zNear;
    float right_minus_left = top_minus_bottom * aspect_ratio;

    ortho_scale << 2/right_minus_left, 0, 0, 0,
    0, 2/top_minus_bottom, 0, 0,
    0, 0, 2/(zNear-zFar), 0,
    0, 0, 0, 1;

    persp_2_ortho << zNear, 0, 0, 0,
    0, zNear, 0, 0,
    0, 0, zNear + zFar, - zNear * zFar,
    0, 0, 1, 0;

    projection = ortho_scale * persp_2_ortho * projection;
    
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    bool use_rodrigues = true;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        if (argc ==5 && std::string(argv[4]) == "--z") {
            use_rodrigues = false;
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    // Rotating around any axis
    Eigen::Vector3f rotation_axis = {0, 1, 0};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if (use_rodrigues) {
            r.set_model(get_model_matrix_rodrigues_rotation(rotation_axis, angle));
        } else {
            r.set_model(get_model_matrix(angle));
        }

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        // Flip the image because OpenCV coordinates start from the top-left corner
        cv::flip(image, image, 0);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if (use_rodrigues) {
            r.set_model(get_model_matrix_rodrigues_rotation(rotation_axis, angle));
        } else {
            r.set_model(get_model_matrix(angle));
        }

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        // Flip the image because OpenCV coordinates start from the top-left corner
        cv::flip(image, image, 0);

        std::string text = use_rodrigues ? "Rodrigues Roatation" : "Z-Axis Rotation";
        cv::putText(image, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,255,255), 2);

        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'm') {
            use_rodrigues = !use_rodrigues;
            std::cout << "[Mode Switch] Now using " << (use_rodrigues ? "Rodrigues rotation." : "Z-axis rotation.") << std::endl;
            
        }
    }

    return 0;
}
