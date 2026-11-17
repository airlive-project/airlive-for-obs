// secret-store.cpp (Windows) — the receiver password in the Credential Manager (a generic
// credential per source, local machine). See secret-store.hpp.

#include "../../secret-store.hpp"

#include <windows.h>
#include <wincred.h>

#include <obs-module.h>

namespace airlive {

namespace {
// "studio.airlive.obs.auth/<sid>" - the same service name the Mac Keychain item uses.
std::wstring targetName(const std::string &account) {
    const std::string utf8 = "studio.airlive.obs.auth/" + account;
    const int n = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), int(utf8.size()), nullptr, 0);
    std::wstring w(size_t(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), int(utf8.size()), w.data(), n);
    return w;
}
} // namespace

void secretStoreDelete(const std::string &account) {
    CredDeleteW(targetName(account).c_str(), CRED_TYPE_GENERIC, 0);
}

bool secretStoreSet(const std::string &account, const std::string &secret) {
    secretStoreDelete(account);
    if (secret.empty())
        return true;
    std::wstring target = targetName(account);
    CREDENTIALW cred{};
    cred.Type = CRED_TYPE_GENERIC;
    cred.TargetName = target.data();
    cred.CredentialBlobSize = DWORD(secret.size());
    cred.CredentialBlob = reinterpret_cast<LPBYTE>(const_cast<char *>(secret.data()));
    cred.Persist = CRED_PERSIST_LOCAL_MACHINE;
    if (!CredWriteW(&cred, 0)) {
        blog(LOG_ERROR, "[airlive] could not store the receiver password (error %lu)", GetLastError());
        return false;
    }
    return true;
}

std::string secretStoreGet(const std::string &account) {
    PCREDENTIALW cred = nullptr;
    if (!CredReadW(targetName(account).c_str(), CRED_TYPE_GENERIC, 0, &cred))
        return {};
    std::string result(reinterpret_cast<const char *>(cred->CredentialBlob), cred->CredentialBlobSize);
    CredFree(cred);
    return result;
}

} // namespace airlive
