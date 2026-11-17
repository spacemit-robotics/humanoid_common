// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0

#ifndef MUJOCO_DEPTH_PUBLISHER_H
#define MUJOCO_DEPTH_PUBLISHER_H

#include <memory>
#include <string>
namespace perception {
class MujocoDepthPublisher {
    public:
    explicit MujocoDepthPublisher(std::string endpoint);
    ~MujocoDepthPublisher();
    void Observe(double time, const void *model, const void *data);

    private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace perception

#endif // MUJOCO_DEPTH_PUBLISHER_H
