// secret-store.cpp (macOS) — the receiver password in the login Keychain. See secret-store.hpp.

#include "../../secret-store.hpp"

#include <CoreFoundation/CoreFoundation.h>
#include <Security/Security.h>

namespace airlive {

namespace {
const char *secretService() { return "studio.airlive.obs.auth"; }
}

namespace detail {
inline CFStringRef cf(const std::string &s) {
    return CFStringCreateWithCString(nullptr, s.c_str(), kCFStringEncodingUTF8);
}
} // namespace detail

// Delete any stored password for this source (no-op if none).
void secretStoreDelete(const std::string &account) {
    CFStringRef svc = detail::cf(secretService());
    CFStringRef acc = detail::cf(account);
    const void *keys[] = {kSecClass, kSecAttrService, kSecAttrAccount};
    const void *vals[] = {kSecClassGenericPassword, svc, acc};
    CFDictionaryRef q = CFDictionaryCreate(nullptr, keys, vals, 3,
                                           &kCFTypeDictionaryKeyCallBacks,
                                           &kCFTypeDictionaryValueCallBacks);
    SecItemDelete(q);
    CFRelease(q);
    CFRelease(svc);
    CFRelease(acc);
}

// Store (replace) the password. An empty string just deletes — "no password" is
// the absence of a secret, never a stored blank. Returns false on a Keychain error.
bool secretStoreSet(const std::string &account, const std::string &secret) {
    secretStoreDelete(account);
    if (secret.empty())
        return true;
    CFStringRef svc = detail::cf(secretService());
    CFStringRef acc = detail::cf(account);
    CFDataRef data = CFDataCreate(nullptr, reinterpret_cast<const UInt8 *>(secret.data()),
                                  CFIndex(secret.size()));
    const void *keys[] = {kSecClass, kSecAttrService, kSecAttrAccount, kSecValueData,
                          kSecAttrAccessible};
    const void *vals[] = {kSecClassGenericPassword, svc, acc, data,
                          kSecAttrAccessibleWhenUnlockedThisDeviceOnly};
    CFDictionaryRef item = CFDictionaryCreate(nullptr, keys, vals, 5,
                                              &kCFTypeDictionaryKeyCallBacks,
                                              &kCFTypeDictionaryValueCallBacks);
    OSStatus st = SecItemAdd(item, nullptr);
    CFRelease(item);
    CFRelease(data);
    CFRelease(svc);
    CFRelease(acc);
    return st == errSecSuccess;
}

// Read the stored password, or "" if none.
std::string secretStoreGet(const std::string &account) {
    CFStringRef svc = detail::cf(secretService());
    CFStringRef acc = detail::cf(account);
    const void *keys[] = {kSecClass, kSecAttrService, kSecAttrAccount, kSecReturnData,
                          kSecMatchLimit};
    const void *vals[] = {kSecClassGenericPassword, svc, acc, kCFBooleanTrue,
                          kSecMatchLimitOne};
    CFDictionaryRef q = CFDictionaryCreate(nullptr, keys, vals, 5,
                                           &kCFTypeDictionaryKeyCallBacks,
                                           &kCFTypeDictionaryValueCallBacks);
    CFTypeRef out = nullptr;
    OSStatus st = SecItemCopyMatching(q, &out);
    CFRelease(q);
    CFRelease(svc);
    CFRelease(acc);
    std::string result;
    if (st == errSecSuccess && out) {
        CFDataRef d = reinterpret_cast<CFDataRef>(out);
        result.assign(reinterpret_cast<const char *>(CFDataGetBytePtr(d)),
                      size_t(CFDataGetLength(d)));
        CFRelease(out);
    }
    return result;
}

} // namespace airlive
