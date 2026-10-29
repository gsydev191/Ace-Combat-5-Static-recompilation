#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace ac5 {

struct Vec3 {
    float x, y, z;
};

struct Vec2 {
    float x, y;
};

struct NormalMatrix {
    // Three padded column vectors, matching GLSL's std430/push-constant layout.
    std::array<float, 12> value{};
};

struct Mat4 {
    // Column-major, right-handed transforms. Perspective maps depth to Vulkan's
    // [0, 1] clip range and flips Y for a positive-height Vulkan viewport.
    std::array<float, 16> value{};

    static Mat4 identity() {
        Mat4 out{};
        out.value[0] = out.value[5] = out.value[10] = out.value[15] = 1.0f;
        return out;
    }

    static Mat4 translation(Vec3 p) {
        Mat4 out = identity();
        out.value[12] = p.x;
        out.value[13] = p.y;
        out.value[14] = p.z;
        return out;
    }

    static Mat4 rotation_x(float angle) {
        Mat4 out = identity();
        const float c = std::cos(angle), s = std::sin(angle);
        out.value[5] = c;
        out.value[6] = s;
        out.value[9] = -s;
        out.value[10] = c;
        return out;
    }

    static Mat4 rotation_y(float angle) {
        Mat4 out = identity();
        const float c = std::cos(angle), s = std::sin(angle);
        out.value[0] = c;
        out.value[2] = -s;
        out.value[8] = s;
        out.value[10] = c;
        return out;
    }

    static Mat4 rotation_z(float angle) {
        Mat4 out = identity();
        const float c = std::cos(angle), s = std::sin(angle);
        out.value[0] = c;
        out.value[1] = s;
        out.value[4] = -s;
        out.value[5] = c;
        return out;
    }

    static Mat4 perspective(float vertical_fov_radians, float aspect,
                            float near_plane, float far_plane) {
        if (!std::isfinite(vertical_fov_radians) || !std::isfinite(aspect) ||
            !std::isfinite(near_plane) || !std::isfinite(far_plane) ||
            vertical_fov_radians <= 0.0f || vertical_fov_radians >= 3.14159265f ||
            aspect <= 0.0f || near_plane <= 0.0f || far_plane <= near_plane)
            return identity();
        Mat4 out{};
        const float f = 1.0f / std::tan(vertical_fov_radians * 0.5f);
        out.value[0] = f / aspect;
        out.value[5] = -f;
        out.value[10] = far_plane / (near_plane - far_plane);
        out.value[11] = -1.0f;
        out.value[14] = (far_plane * near_plane) / (near_plane - far_plane);
        return out;
    }

    static Mat4 look_at(Vec3 eye, Vec3 target, Vec3 up) {
        const Vec3 forward = normalize({target.x - eye.x, target.y - eye.y, target.z - eye.z});
        if (dot(forward, forward) == 0.0f) return identity();
        const Vec3 right = normalize(cross(forward, up));
        if (dot(right, right) == 0.0f) return identity();
        const Vec3 camera_up = cross(right, forward);
        Mat4 out = identity();
        out.value[0] = right.x;
        out.value[1] = camera_up.x;
        out.value[2] = -forward.x;
        out.value[4] = right.y;
        out.value[5] = camera_up.y;
        out.value[6] = -forward.y;
        out.value[8] = right.z;
        out.value[9] = camera_up.z;
        out.value[10] = -forward.z;
        out.value[12] = -dot(right, eye);
        out.value[13] = -dot(camera_up, eye);
        out.value[14] = dot(forward, eye);
        return out;
    }

    static Mat4 multiply(const Mat4& a, const Mat4& b) {
        Mat4 out{};
        for (int column = 0; column < 4; ++column) {
            for (int row = 0; row < 4; ++row) {
                for (int k = 0; k < 4; ++k) {
                    out.value[column * 4 + row] +=
                        a.value[k * 4 + row] * b.value[column * 4 + k];
                }
            }
        }
        return out;
    }

    static NormalMatrix normal_matrix(const Mat4& model) {
        double scale = 0.0;
        for (const std::size_t index : {0u, 1u, 2u, 4u, 5u, 6u, 8u, 9u, 10u}) {
            const double component = model.value[index];
            if (!std::isfinite(component)) return identity_normal_matrix();
            scale = std::max(scale, std::abs(component));
        }
        if (scale == 0.0) return identity_normal_matrix();

        const double a00 = model.value[0] / scale;
        const double a01 = model.value[4] / scale;
        const double a02 = model.value[8] / scale;
        const double a10 = model.value[1] / scale;
        const double a11 = model.value[5] / scale;
        const double a12 = model.value[9] / scale;
        const double a20 = model.value[2] / scale;
        const double a21 = model.value[6] / scale;
        const double a22 = model.value[10] / scale;

        const double c00 = a11 * a22 - a12 * a21;
        const double c01 = a12 * a20 - a10 * a22;
        const double c02 = a10 * a21 - a11 * a20;
        const double c10 = a02 * a21 - a01 * a22;
        const double c11 = a00 * a22 - a02 * a20;
        const double c12 = a01 * a20 - a00 * a21;
        const double c20 = a01 * a12 - a02 * a11;
        const double c21 = a02 * a10 - a00 * a12;
        const double c22 = a00 * a11 - a01 * a10;
        const double determinant = a00 * c00 + a01 * c01 + a02 * c02;
        if (!std::isfinite(determinant) || std::abs(determinant) <= 1.0e-12)
            return identity_normal_matrix();

        const double divisor = determinant * scale;
        const std::array<double, 9> inverse_transpose{
            c00 / divisor, c10 / divisor, c20 / divisor,
            c01 / divisor, c11 / divisor, c21 / divisor,
            c02 / divisor, c12 / divisor, c22 / divisor};
        NormalMatrix out{};
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t row = 0; row < 3; ++row) {
                const double component = inverse_transpose[column * 3 + row];
                if (!std::isfinite(component) ||
                    std::abs(component) > std::numeric_limits<float>::max())
                    return identity_normal_matrix();
                out.value[column * 4 + row] = static_cast<float>(component);
            }
        }
        return out;
    }

private:
    static NormalMatrix identity_normal_matrix() {
        NormalMatrix out{};
        out.value[0] = out.value[5] = out.value[10] = 1.0f;
        return out;
    }

    static float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    static Vec3 cross(Vec3 a, Vec3 b) {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }
    static Vec3 normalize(Vec3 v) {
        const float length = std::sqrt(dot(v, v));
        if (!std::isfinite(length) || length <= 1.0e-12f) return {0.0f, 0.0f, 0.0f};
        return {v.x / length, v.y / length, v.z / length};
    }
};

struct Camera {
    Vec3 position{0.0f, 0.0f, 3.0f};
    Vec3 target{0.0f, 0.0f, 0.0f};
    Vec3 up{0.0f, 1.0f, 0.0f};
    float vertical_fov_radians = 1.04719755f;
    float near_plane = 0.1f;
    float far_plane = 100.0f;

    Mat4 view() const { return Mat4::look_at(position, target, up); }
    Mat4 projection(float aspect) const {
        return Mat4::perspective(vertical_fov_radians, aspect, near_plane, far_plane);
    }
    Mat4 view_projection(float aspect) const {
        return Mat4::multiply(projection(aspect), view());
    }
};

} // namespace ac5
