#include <phy/bridge/crypto.hpp>

#include <cstring>
#include <openssl/evp.h>

namespace phy::bridge {

bool expandPsk(const uint8_t* psk, size_t len, std::vector<uint8_t>& key_out) {
    key_out.clear();
    if (psk == nullptr || len == 0) {
        return true;  // no encryption
    }
    if (len == 1) {
        if (psk[0] == 0) {
            return true;
        }
        key_out.assign(kDefaultPsk, kDefaultPsk + 16);
        if (psk[0] > 1) {
            key_out[15] = static_cast<uint8_t>(kDefaultPsk[15] + (psk[0] - 1));
        }
        return true;
    }
    if (len == 16 || len == 32) {
        key_out.assign(psk, psk + len);
        return true;
    }
    return false;
}

uint8_t channelHash(const std::string& name, const std::vector<uint8_t>& key) {
    return static_cast<uint8_t>(xorHash(reinterpret_cast<const uint8_t*>(name.data()), name.size()) ^
                                xorHash(key.data(), key.size()));
}

bool aesCtr(const std::vector<uint8_t>& key, uint32_t packet_id, uint32_t from_node, uint8_t* data,
            size_t len) {
    if (key.empty()) {
        return true;  // cleartext
    }
    if (data == nullptr && len != 0) {
        return false;
    }
    if (key.size() != 16 && key.size() != 32) {
        return false;
    }

    uint8_t nonce[16] = {};
    // packet_id as uint64 LE in bytes 0..7 (firmware initNonce)
    nonce[0] = static_cast<uint8_t>(packet_id);
    nonce[1] = static_cast<uint8_t>(packet_id >> 8);
    nonce[2] = static_cast<uint8_t>(packet_id >> 16);
    nonce[3] = static_cast<uint8_t>(packet_id >> 24);
    // bytes 4..7 stay 0
    nonce[8] = static_cast<uint8_t>(from_node);
    nonce[9] = static_cast<uint8_t>(from_node >> 8);
    nonce[10] = static_cast<uint8_t>(from_node >> 16);
    nonce[11] = static_cast<uint8_t>(from_node >> 24);

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (ctx == nullptr) {
        return false;
    }
    const EVP_CIPHER* cipher = (key.size() == 32) ? EVP_aes_256_ctr() : EVP_aes_128_ctr();
    bool ok = EVP_EncryptInit_ex(ctx, cipher, nullptr, key.data(), nonce) == 1;
    if (ok && len != 0) {
        int out_len = 0;
        ok = EVP_EncryptUpdate(ctx, data, &out_len, data, static_cast<int>(len)) == 1 &&
             out_len == static_cast<int>(len);
        if (ok) {
            int final_len = 0;
            ok = EVP_EncryptFinal_ex(ctx, data + out_len, &final_len) == 1;
        }
    }
    EVP_CIPHER_CTX_free(ctx);
    return ok;
}

bool aesCtr(const std::vector<uint8_t>& key, uint32_t packet_id, uint32_t from_node,
            std::vector<uint8_t>& data) {
    return aesCtr(key, packet_id, from_node, data.data(), data.size());
}

}  // namespace phy::bridge
