// secret-store.hpp — OS secret store for the receiver-password (STREAM-AUTH §4).
//
// The auth password must NOT live in the scene-collection JSON (plaintext on
// disk). It goes in the OS secret store instead: the Keychain on macOS
// (platform/macos/secret-store.cpp), the Credential Manager on Windows
// (platform/windows/secret-store.cpp).
//
// One generic-password item per source, keyed by the source's stable `sid` (the
// UUID persisted in settings), so it survives renames and is scoped to one source.

#pragma once

#include <string>

namespace airlive {

// Delete any stored password for this source (no-op if none).
void secretStoreDelete(const std::string &account);

// Store (replace) the password. An empty string just deletes — "no password" is
// the absence of a secret, never a stored blank. Returns false on a store error.
bool secretStoreSet(const std::string &account, const std::string &secret);

// Read the stored password, or "" if none.
std::string secretStoreGet(const std::string &account);

} // namespace airlive
