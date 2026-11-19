// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "mujoco_depth_publisher.h"
#include "depth_stream.h"
#include "runtime_logger.h"
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mujoco/mujoco.h>
#include <sstream>
#include <stdexcept>
#include <zmq.h>

namespace perception {
class MujocoDepthPublisher::Impl {
    public:
    std::string endpoint;
    std::mutex mutex;
    std::condition_variable condition;
    bool stop = false, pending = false, subscribed = false;
    mjModel *model = nullptr;
    mjData *snapshot = nullptr;
    double last = -1;
    uint64_t epoch = 0;
    std::thread worker;
    explicit Impl(std::string address) : endpoint(std::move(address)), worker([this] { Loop(); }) {}
    ~Impl() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
        }
        condition.notify_one();
        worker.join();
        if (snapshot)
            mj_deleteData(snapshot);
        if (model)
            mj_deleteModel(model);
    }
    void Observe(double time, const void *m, const void *d) {
        std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
        if (!lock || !subscribed || pending)
            return;
        if (time < last) {
            ++epoch;
            last = -1;
        }
        if (last >= 0 && time - last < .02 - 1e-9)
            return;
        if (!model) {
            model = mj_copyModel(nullptr, static_cast<const mjModel *>(m));
            if (model) {
                // Adapt only our private rendering copy. The simulator continues
                // to use the unmodified SDK robot, including its native contacts.
                const int camera = mj_name2id(model, mjOBJ_CAMERA, "rgbd_camera");
                const int pelvis = mj_name2id(model, mjOBJ_BODY, "pelvis");
                if (camera < 0 || pelvis < 0)
                    throw std::runtime_error("scene lacks SDK rgbd_camera or pelvis");
                const double xml_fovy = model->cam_fovy[camera];
                model->cam_fovy[camera] = 58.29;
                for (int i = 0; i < model->ngeom; ++i) {
                    if (model->body_rootid[model->geom_bodyid[i]] != pelvis)
                        continue;
                    const bool visual = model->geom_contype[i] == 0 &&
                                        model->geom_conaffinity[i] == 0;
                    model->geom_group[i] = visual ? 2 : 3;
                }
                runtime_logging::Log(runtime_logging::Level::kInfo,
                    "[depth publisher] SDK camera XML FOV=" + std::to_string(xml_fovy) +
                    " deg; offscreen FOV=58.29 deg; SDK physics unchanged", false);
            }
        }
        if (!model)
            throw std::runtime_error("depth model allocation failed");
        if (!snapshot)
            snapshot = mj_makeData(model);
        if (!snapshot)
            throw std::runtime_error("depth snapshot allocation failed");
        mj_copyData(snapshot, model, static_cast<const mjData *>(d));
        last = time;
        pending = true;
        condition.notify_one();
    }
    void Loop() {
        EGLDisplay display = EGL_NO_DISPLAY;
        EGLContext gl = EGL_NO_CONTEXT;
        EGLSurface surface = EGL_NO_SURFACE;
        void *ctx = nullptr, *socket = nullptr;
        mjvScene scene;
        mjv_defaultScene(&scene);
        mjrContext render;
        mjr_defaultContext(&render);
        try {
            ctx = zmq_ctx_new();
            if (!ctx)
                throw std::runtime_error("depth ZMQ context failed");
            socket = zmq_socket(ctx, ZMQ_XPUB);
            if (!socket)
                throw std::runtime_error("depth ZMQ socket failed");
            int linger = 0, hwm = 2;
            zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
            zmq_setsockopt(socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));
            if (zmq_bind(socket, endpoint.c_str()) != 0)
                throw std::runtime_error("depth endpoint bind failed");
            std::ostringstream id;
            id << std::chrono::steady_clock::now().time_since_epoch().count();
            uint64_t sequence = 0;
            bool ready = false;
            while (true) {
                std::unique_lock<std::mutex> lock(mutex);
                if (stop)
                    break;
                // XPUB reports the first empty-topic subscription and the last unsubscribe.
                unsigned char event;
                int size;
                while ((size = zmq_recv(socket, &event, 1, ZMQ_DONTWAIT)) >= 0)
                    if (size == 1)
                        subscribed = event != 0;
                if (!subscribed)
                    pending = false;
                if (!pending) {
                    condition.wait_for(lock, std::chrono::milliseconds(20), [this] { return stop || pending; });
                    continue;
                }
                const int camera = mj_name2id(model, mjOBJ_CAMERA, "rgbd_camera");
                if (camera < 0)
                    throw std::runtime_error("scene lacks SDK rgbd_camera");
                const double pos[3] = {.0576235, .01753, .42987};
                for (int i = 0; i < 3; ++i)
                    if (std::abs(model->cam_pos[camera * 3 + i] - pos[i]) > 1e-8)
                        throw std::runtime_error("scene camera is not SDK pose");
                if (std::abs(model->cam_fovy[camera] - 58.29) > 1e-6)
                    throw std::runtime_error("scene camera FOV mismatch");
                mjtNum matrix[9];
                mju_quat2Mat(matrix, model->cam_quat + camera * 4);
                const double axes[6] = {0, -1, 0, .738455340625884, 0, .674302387583723};
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 2; ++j)
                        if (std::abs(matrix[i * 3 + j] - axes[j * 3 + i]) > 1e-8)
                            throw std::runtime_error("scene camera rotation mismatch");
                const int torso = mj_name2id(model, mjOBJ_BODY, "torso_link");
                if (torso < 0 || model->cam_bodyid[camera] != torso)
                    throw std::runtime_error("scene camera must attach to torso_link");
                if (!ready) {
                    auto platform = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
                        eglGetProcAddress("eglGetPlatformDisplayEXT"));
                    display = platform ? platform(0x31DD, EGL_DEFAULT_DISPLAY, nullptr)
                        : eglGetDisplay(EGL_DEFAULT_DISPLAY);
                    if (!eglInitialize(display, nullptr, nullptr) || !eglBindAPI(EGL_OPENGL_API))
                        throw std::runtime_error("EGL OpenGL initialization failed");
                    const EGLint attribs[] = {EGL_SURFACE_TYPE,
                        EGL_PBUFFER_BIT,
                        EGL_RENDERABLE_TYPE,
                        EGL_OPENGL_BIT,
                        EGL_RED_SIZE,
                        8,
                        EGL_GREEN_SIZE,
                        8,
                        EGL_BLUE_SIZE,
                        8,
                        EGL_DEPTH_SIZE,
                        24,
                        EGL_NONE};
                    EGLConfig config;
                    EGLint n = 0;
                    if (!eglChooseConfig(display, attribs, &config, 1, &n) || !n)
                        throw std::runtime_error("EGL config missing");
                    const EGLint pbuffer[] = {EGL_WIDTH, 64, EGL_HEIGHT, 36, EGL_NONE};
                    surface = eglCreatePbufferSurface(display, config, pbuffer);
                    gl = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
                    if (!eglMakeCurrent(display, surface, surface, gl))
                        throw std::runtime_error("EGL context failed");
                    mjv_makeScene(model, &scene, 2000);
                    mjr_makeContext(model, &render, mjFONTSCALE_100);
                    mjr_setBuffer(mjFB_OFFSCREEN, &render);
                    render.readDepthMap = mjDEPTH_ZEROFAR;
                    ready = true;
                }
                mjvOption options;
                mjv_defaultOption(&options);
                // Match demo depth visibility: terrain 0 + robot visual 2; collisions are group 3.
                for (int i = 0; i < 6; ++i)
                    options.geomgroup[i] = 0;
                options.geomgroup[0] = options.geomgroup[2] = 1;
                mjvCamera view;
                mjv_defaultCamera(&view);
                view.type = mjCAMERA_FIXED;
                view.fixedcamid = camera;
                mjv_updateScene(model, snapshot, &options, nullptr, &view, mjCAT_ALL, &scene);
                scene.flags[mjRND_SHADOW] = scene.flags[mjRND_REFLECTION] = 0;
                mjrRect viewport{0, 0, 64, 36};
                mjr_render(viewport, &scene, &render);
                float zbuffer[kPixels];
                mjr_readPixels(nullptr, zbuffer, viewport, &render);
                const double near = model->vis.map.znear * model->stat.extent;
                const double far = model->vis.map.zfar * model->stat.extent;
                Frame frame;
                frame.time = snapshot->time;
                frame.clock = "sim";
                frame.sequence = ++sequence;
                frame.session = id.str() + "-" + std::to_string(epoch);
                frame.pixels.resize(kPixels);
                for (int y = 0; y < 36; ++y)
                    for (int x = 0; x < 64; ++x) {
                        const float z = zbuffer[(35 - y) * 64 + x];
                        // MuJoCo >=3.2 offscreen contexts use reverse-Z depth mapping.
                        const double distance = near * far / (near + double(z) * (far - near));
                        frame.pixels[y * 64 + x] =
                            (!std::isfinite(distance) || distance >= .99 * far) ? 0 : distance;
                    }
                const auto packet = Encode(frame);
                zmq_send(socket, packet.data(), packet.size(), ZMQ_DONTWAIT);
                pending = false;
            }
        } catch (const std::exception &e) {
            runtime_logging::Log(runtime_logging::Level::kError,
                std::string("[depth publisher] ") + e.what(), false);
        }
        mjr_freeContext(&render);
        mjv_freeScene(&scene);
        if (socket)
            zmq_close(socket);
        if (ctx)
            zmq_ctx_term(ctx);
        if (display != EGL_NO_DISPLAY) {
            eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (gl != EGL_NO_CONTEXT)
                eglDestroyContext(display, gl);
            if (surface != EGL_NO_SURFACE)
                eglDestroySurface(display, surface);
            eglTerminate(display);
        }
    }
};
MujocoDepthPublisher::MujocoDepthPublisher(std::string e)
    : impl_(std::make_unique<Impl>(std::move(e))) {}
MujocoDepthPublisher::~MujocoDepthPublisher() = default;
void MujocoDepthPublisher::Observe(double t, const void *m, const void *d) {
    impl_->Observe(t, m, d);
}
} // namespace perception
