/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file hmi_runtime.cpp
 * @brief Headless operator service entry point
 */
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "runtime_logger.h"
#include "server.h"
int main(int argc, char *argv[]) {
    if (argc < 2 || std::string(argv[1]) == "--help") {
        std::printf(
            "Usage: %s config.yaml [--sim|--real] [--listen ADDRESS] [--public-url "
            "http://HOST:PORT]\n"
            "       [--terminal-only] [--export-pairing | --show-credential]\n",
            argv[0]);
        return argc < 2 ? 1 : 0;
    }
    try {
        auto config = operator_service::LoadConfig(argv[1]);
        bool export_pairing = false, show_credential = false;
        bool selected_mode = false, network_options = false;
        bool terminal_only = false;
        for (int i = 2; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--terminal-only") {
                terminal_only = true;
                continue;
            }
            if (option == "--sim" || option == "--real") {
                if (selected_mode) throw std::runtime_error("specify only one of --sim or --real");
                config.terminal_only = option == "--sim";
                selected_mode = true;
                continue;
            }
            if (option == "--export-pairing") {
                export_pairing = true;
                continue;
            }
            if (option == "--show-credential") {
                show_credential = true;
                continue;
            }
            if (i + 1 >= argc) throw std::runtime_error("missing option value");
            if (option == "--listen") {
                config.bind_address = argv[++i];
                network_options = true;
            } else if (option == "--public-url") {
                config.public_url = argv[++i];
                network_options = true;
            } else
                throw std::runtime_error("unknown option: " + option);
        }
        config.terminal_only = config.terminal_only || terminal_only;
        if (config.terminal_only && (network_options || export_pairing || show_credential))
            throw std::runtime_error("terminal-only mode has no network or QR options");
        if (export_pairing || show_credential) {
            operator_service::PreparePairing(&config);
            if (show_credential)
                std::puts(config.token.c_str());
            else
                std::printf("Access URL: %s/\nFixed QR: %s\nCredential file: %s\n", config.public_url.c_str(),
                    config.qr_file.c_str(), config.credential_file.c_str());
            return 0;
        }
        runtime_logging::Session logging(robot_base::YamlFile::Load(argv[1]), argv[1], "hmi");
        return operator_service::Run(config, argv[1]);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "[hmi] %s\n", e.what());
        return 1;
    }
}
