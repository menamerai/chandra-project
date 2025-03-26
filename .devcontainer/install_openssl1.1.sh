#!/bin/bash

# Install required dependencies
sudo apt-get update
sudo apt-get install -y build-essential checkinstall zlib1g-dev wget

# Download OpenSSL 1.1.1
cd /tmp
wget https://www.openssl.org/source/openssl-1.1.1q.tar.gz
tar -xf openssl-1.1.1q.tar.gz
cd openssl-1.1.1q

# Configure and install to a specific location to avoid conflicts
./config --prefix=/usr/local/openssl-1.1.1 --openssldir=/usr/local/openssl-1.1.1 shared zlib
make -j$(nproc)
sudo make install

# Create symbolic links for the libraries
sudo ln -sf /usr/local/openssl-1.1.1/lib/libcrypto.so.1.1 /usr/lib/libcrypto.so.1.1
sudo ln -sf /usr/local/openssl-1.1.1/lib/libssl.so.1.1 /usr/lib/libssl.so.1.1

# Update library cache
sudo ldconfig