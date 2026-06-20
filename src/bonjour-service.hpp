// bonjour-service.hpp — advertise one "_airlive._tcp" instance per OBS source.
//
// WHY: the iPhone is the TCP client; it browses Bonjour for receivers and
// connects out. We are the server, so we must advertise the ephemeral port we
// bound to, plus a TXT record the phone uses to GROUP and LABEL us:
//   v=1, role=obs, did=<per-install UUID, group key>, dev=<group display name>,
//   sid=<stable source id>, src=<source display name>, busy=0|1
// The phone groups instances by `did` (header = `dev`), lists each `src`, and
// flips our row to "In use" when busy=1.
//
// Implemented against dns_sd (Bonjour API): built in on macOS, provided by the
// Bonjour SDK on Windows and by avahi-compat-libdns_sd on Linux — so one code
// path covers all three.

#pragma once

#include <cstdint>
#include <string>

namespace airlive {

struct ServiceIdentity {
    std::string did; // stable per-install UUID — group key
    std::string dev; // group display name, default "Airlive OBS"
    std::string sid; // stable per-source id
    std::string src; // source display name, default "OBS Source N"
};

class BonjourService {
public:
    BonjourService() = default;
    ~BonjourService();

    BonjourService(const BonjourService &) = delete;
    BonjourService &operator=(const BonjourService &) = delete;

    // Register the service on `port` (host byte order) with busy=0. Returns
    // false and logs if registration fails; the source still works locally, the
    // phone just can't auto-discover it.
    bool start(uint16_t port, const ServiceIdentity &id);

    // Update only the busy flag and re-publish the TXT record.
    void setBusy(bool busy);

    void stop();

    // The dns_sd socket fd to poll, and the handler to call when it's readable —
    // lets the owner service registration/conflict callbacks. -1 when inactive.
    int fd() const;
    void process();

private:
    bool publishTxt(bool busy);

    void *ref_ = nullptr; // DNSServiceRef (opaque here; cast in the .cpp)
    ServiceIdentity id_;
    bool busy_ = false;
};

} // namespace airlive
