// bonjour-service.cpp (Windows) — the advert through the DnsService* API built into Windows 10
// 1809 and later, so nothing else (no Bonjour service) has to be installed. See bonjour-service.hpp.
//
// The API has no in-place TXT update: a new busy flag is published by registering the instance
// again (deregister, then register). It calls back on its own thread, so there is no socket for
// the owner to poll (fd() is -1).

#include "../../bonjour-service.hpp"

#include <windows.h>
#include <windns.h>

#include <string>
#include <vector>

#include <obs-module.h>

#include "../../wire.hpp" // kProtoGeneration / kProtoMinGeneration for the pv/mpv TXT keys

namespace airlive {

namespace {

// How long a re-registration waits for Windows to confirm the old one is gone. It runs on the
// connection worker (the busy flag flips as a phone connects), so it must stay short.
constexpr DWORD kDeregisterWaitMs = 250;

std::wstring widen(const std::string &s) {
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), nullptr, 0);
    std::wstring w(size_t(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), w.data(), n);
    return w;
}

// One live registration: the request Windows holds a pointer to, the instance it describes, and an
// event the completion callback signals.
struct Registration {
    DNS_SERVICE_REGISTER_REQUEST request{};
    PDNS_SERVICE_INSTANCE instance = nullptr;
    HANDLE done = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    DWORD status = 0;
    std::wstring name;
};

VOID WINAPI onRegister(DWORD status, PVOID context, PDNS_SERVICE_INSTANCE instance) {
    auto *reg = static_cast<Registration *>(context);
    reg->status = status;
    if (instance)
        DnsServiceFreeInstance(instance); // the callback's copy, not ours
    if (status == ERROR_SUCCESS)
        blog(LOG_INFO, "[airlive] advertised '%ls'", reg->name.c_str());
    else
        blog(LOG_WARNING, "[airlive] mDNS registration error %lu", status);
    SetEvent(reg->done);
}

// "<this computer> <first 8 of sid>": unique per source, readable in a browser. The phone lists
// sources by their TXT record, not by this name.
std::wstring instanceName(const ServiceIdentity &id) {
    wchar_t host[MAX_COMPUTERNAME_LENGTH + 1] = L"OBS";
    DWORD len = MAX_COMPUTERNAME_LENGTH + 1;
    GetComputerNameW(host, &len);
    return std::wstring(host) + L" " + widen(id.sid.substr(0, 8)) + L"._airlive._tcp.local";
}

std::wstring hostName() {
    wchar_t host[MAX_COMPUTERNAME_LENGTH + 1] = L"obs";
    DWORD len = MAX_COMPUTERNAME_LENGTH + 1;
    GetComputerNameW(host, &len);
    return std::wstring(host) + L".local";
}

Registration *registerInstance(uint16_t port, const ServiceIdentity &id, bool busy) {
    const std::vector<std::pair<std::wstring, std::wstring>> txt = {
        {L"v", L"1"},
        {L"role", widen(id.role.empty() ? "obs" : id.role)},
        {L"did", widen(id.did)},
        {L"dev", widen(id.dev)},
        {L"sid", widen(id.sid)},
        {L"src", widen(id.src)},
        {L"busy", busy ? L"1" : L"0"},
        {L"ord", widen(id.ord)},                                // channel order (asymmetry #6)
        {L"pv", std::to_wstring(kProtoGeneration)},             // protocol generation
        {L"mpv", std::to_wstring(kProtoMinGeneration)},
    };
    std::vector<PCWSTR> keys, values;
    for (const auto &kv : txt) {
        keys.push_back(kv.first.c_str());
        values.push_back(kv.second.c_str());
    }

    auto *reg = new Registration();
    reg->name = instanceName(id);
    const std::wstring host = hostName();
    reg->instance = DnsServiceConstructInstance(reg->name.c_str(), host.c_str(), nullptr, nullptr,
                                                port, 0, 0, DWORD(keys.size()), keys.data(),
                                                values.data());
    if (!reg->instance) {
        blog(LOG_ERROR, "[airlive] could not build the mDNS advert");
        CloseHandle(reg->done);
        delete reg;
        return nullptr;
    }
    reg->request.Version = DNS_QUERY_REQUEST_VERSION1;
    reg->request.InterfaceIndex = 0; // every interface
    reg->request.pServiceInstance = reg->instance;
    reg->request.pRegisterCompletionCallback = onRegister;
    reg->request.pQueryContext = reg;
    const DWORD st = DnsServiceRegister(&reg->request, nullptr);
    if (st != DNS_REQUEST_PENDING) {
        blog(LOG_ERROR, "[airlive] DnsServiceRegister failed (%lu) on port %u", st, port);
        DnsServiceFreeInstance(reg->instance);
        CloseHandle(reg->done);
        delete reg;
        return nullptr;
    }
    return reg;
}

void deregister(Registration *reg) {
    if (!reg)
        return;
    if (DnsServiceDeRegister(&reg->request, nullptr) == DNS_REQUEST_PENDING &&
        WaitForSingleObject(reg->done, kDeregisterWaitMs) != WAIT_OBJECT_0) {
        // Windows still holds the request and will call back into it: freeing it now would be a
        // use-after-free, so it is deliberately left (a few hundred bytes, once in a long while).
        blog(LOG_WARNING, "[airlive] mDNS deregistration not confirmed in %lu ms - left to Windows",
             kDeregisterWaitMs);
        return;
    }
    DnsServiceFreeInstance(reg->instance);
    CloseHandle(reg->done);
    delete reg;
}

} // namespace

BonjourService::~BonjourService() { stop(); }

bool BonjourService::start(uint16_t port, const ServiceIdentity &id) {
    id_ = id;
    busy_ = false;
    port_ = port;
    ref_ = registerInstance(port, id_, false);
    if (!ref_)
        return false;
    blog(LOG_INFO, "[airlive] mDNS registering on port %u (did=%s sid=%s)", port, id_.did.c_str(),
         id_.sid.c_str());
    return true;
}

bool BonjourService::publishTxt(bool busy) {
    if (!ref_)
        return false;
    deregister(static_cast<Registration *>(ref_));
    ref_ = registerInstance(port_, id_, busy);
    return ref_ != nullptr;
}

void BonjourService::setBusy(bool busy) {
    if (busy == busy_)
        return;
    busy_ = busy;
    publishTxt(busy);
}

int BonjourService::fd() const { return -1; }

void BonjourService::process() {}

void BonjourService::stop() {
    deregister(static_cast<Registration *>(ref_));
    ref_ = nullptr;
}

} // namespace airlive
