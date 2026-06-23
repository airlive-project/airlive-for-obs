#include "bonjour-service.hpp"

#include <cstring>

#include <obs-module.h>
#include <dns_sd.h>

namespace airlive {

namespace {
// dns_sd's registration callback. We only log the outcome — once registered,
// the mDNS daemon maintains the advertisement until we deallocate the ref.
void DNSSD_API onRegister(DNSServiceRef, DNSServiceFlags, DNSServiceErrorType err,
                          const char *name, const char *regtype, const char *domain, void *) {
    if (err == kDNSServiceErr_NoError)
        blog(LOG_INFO, "[airlive] advertised '%s' %s%s", name ? name : "?",
             regtype ? regtype : "", domain ? domain : "");
    else
        blog(LOG_WARNING, "[airlive] Bonjour registration error %d", int(err));
}

void setTxt(TXTRecordRef &txt, const char *key, const std::string &val) {
    // TXT values are length-prefixed; clamp to the 255-byte per-value limit.
    const uint8_t n = uint8_t(val.size() > 255 ? 255 : val.size());
    TXTRecordSetValue(&txt, key, n, val.data());
}
} // namespace

BonjourService::~BonjourService() { stop(); }

bool BonjourService::start(uint16_t port, const ServiceIdentity &id) {
    id_ = id;
    busy_ = false;

    TXTRecordRef txt;
    TXTRecordCreate(&txt, 0, nullptr);
    setTxt(txt, "v", "1");
    setTxt(txt, "role", id_.role.empty() ? "obs" : id_.role);
    setTxt(txt, "did", id_.did);
    setTxt(txt, "dev", id_.dev);
    setTxt(txt, "sid", id_.sid);
    setTxt(txt, "src", id_.src);
    setTxt(txt, "busy", "0");

    // Port must be network byte order. name=NULL → use the host name; the phone
    // identifies us by the TXT record, not the service instance name.
    DNSServiceRef sdref = nullptr;
    const DNSServiceErrorType err = DNSServiceRegister(
        &sdref, 0, 0, /*name*/ nullptr, "_airlive._tcp", /*domain*/ nullptr,
        /*host*/ nullptr, htons(port), TXTRecordGetLength(&txt),
        TXTRecordGetBytesPtr(&txt), onRegister, this);

    TXTRecordDeallocate(&txt);

    if (err != kDNSServiceErr_NoError) {
        blog(LOG_ERROR, "[airlive] DNSServiceRegister failed (%d) on port %u", int(err), port);
        ref_ = nullptr;
        return false;
    }
    ref_ = sdref;
    blog(LOG_INFO, "[airlive] Bonjour registering on port %u (did=%s sid=%s)", port,
         id_.did.c_str(), id_.sid.c_str());
    return true;
}

bool BonjourService::publishTxt(bool busy) {
    if (!ref_)
        return false;

    TXTRecordRef txt;
    TXTRecordCreate(&txt, 0, nullptr);
    setTxt(txt, "v", "1");
    setTxt(txt, "role", id_.role.empty() ? "obs" : id_.role);
    setTxt(txt, "did", id_.did);
    setTxt(txt, "dev", id_.dev);
    setTxt(txt, "sid", id_.sid);
    setTxt(txt, "src", id_.src);
    setTxt(txt, "busy", busy ? "1" : "0");

    // Passing NULL recordRef updates the service's primary TXT record in place —
    // no teardown, so the phone never sees the instance disappear/reappear.
    const DNSServiceErrorType err = DNSServiceUpdateRecord(
        static_cast<DNSServiceRef>(ref_), nullptr, 0, TXTRecordGetLength(&txt),
        TXTRecordGetBytesPtr(&txt), 0);
    TXTRecordDeallocate(&txt);

    if (err != kDNSServiceErr_NoError) {
        blog(LOG_WARNING, "[airlive] TXT update failed (%d)", int(err));
        return false;
    }
    return true;
}

void BonjourService::setBusy(bool busy) {
    if (busy == busy_)
        return;
    busy_ = busy;
    publishTxt(busy);
}

int BonjourService::fd() const {
    return ref_ ? DNSServiceRefSockFD(static_cast<DNSServiceRef>(ref_)) : -1;
}

void BonjourService::process() {
    if (ref_)
        DNSServiceProcessResult(static_cast<DNSServiceRef>(ref_));
}

void BonjourService::stop() {
    if (ref_) {
        DNSServiceRefDeallocate(static_cast<DNSServiceRef>(ref_));
        ref_ = nullptr;
    }
}

} // namespace airlive
