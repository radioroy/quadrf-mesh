# SPDX-FileCopyrightText: 2026 Roy C. Gross
# SPDX-License-Identifier: Apache-2.0
FROM debian:trixie-slim

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --yes --no-install-recommends \
    bluez build-essential ca-certificates cmake curl debhelper devscripts dpkg-dev \
    equivs fakeroot file git libbluetooth-dev libfftw3-dev libgpiod-dev \
    libi2c-dev liborcania-dev libsdl2-dev libsoapysdr-dev libssl-dev libulfius-dev \
    libusb-1.0-0-dev libuv1-dev libyaml-cpp-dev libyder-dev lintian \
    ninja-build openssl patch pkg-config python3 python3-dbus python3-gi \
    python3-pip python3-venv shellcheck tar xz-utils \
    && printf '#!/bin/sh\nexit 101\n' >/usr/sbin/policy-rc.d \
    && chmod 0755 /usr/sbin/policy-rc.d \
    && rm -rf /var/lib/apt/lists/*
