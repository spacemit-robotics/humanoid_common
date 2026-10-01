#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SDK="${ANDROID_HOME:-$HOME/Android/Sdk}"
if [[ ! -f "$SDK/platforms/android-35/android.jar" || ! -x "$SDK/build-tools/35.0.0/apksigner" ]]; then
    echo "Android SDK 35 / build-tools 35.0.0 not found in $SDK" >&2
    exit 1
fi
cd "$HERE"
ANDROID_HOME="$SDK" ./gradlew --no-daemon assembleDebug lintDebug
"$SDK/build-tools/35.0.0/apksigner" verify app/build/outputs/apk/debug/app-debug.apk
echo "APK: $HERE/app/build/outputs/apk/debug/app-debug.apk"
