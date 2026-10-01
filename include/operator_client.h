/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file operator_client.h
 * @brief Typed operator service client
 */
#ifndef OPERATOR_CLIENT_H
#define OPERATOR_CLIENT_H
#include "operator_types.h"
#include <memory>
#include <string>
#include <vector>
namespace operator_service {
class Client {
public:
    Client();
    ~Client();
    Client(const Client &) = delete;
    Client &operator=(const Client &) = delete;
    // Connect/Disconnect must not run concurrently with other calls.
    bool Connect(const Connection &connection, std::string *error);
    void Disconnect();
    bool Connected() const;
    std::string LastError() const;
    Status LatestStatus() const;
    std::vector<Policy> Policies() const;
    std::string RobotName() const;
    Velocity VelocityStep() const;
    // Requests are serialized, bounded and never automatically retried.
    // The caller renews its lease explicitly while actively operating.
    Reply AcquireControl();
    Reply RenewControl();
    Reply ReleaseControl();
    Reply RequestState(const std::string &state);
    Reply SelectPolicy(const std::string &policy);
    Reply SetVelocity(const Velocity &velocity, int valid_for_ms = 300);
    Reply Stop();
    Reply StartInteraction(const std::string &action);
    Reply CancelInteraction();
    Reply StartReference();
    Reply AcknowledgeFault();
    std::string PairingUrl();
    static Connection ReadConnectionFile(const std::string &path);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace operator_service
#endif // OPERATOR_CLIENT_H
