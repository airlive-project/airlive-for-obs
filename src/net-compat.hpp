// net-compat.hpp — thin POSIX/Winsock shim so the socket code reads the same on
// every platform. OBS plugins ship on Win/macOS/Linux from one source tree.

#pragma once

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
using poll_fd_t = WSAPOLLFD;
constexpr socket_t kInvalidSocket = INVALID_SOCKET;
inline int close_socket(socket_t s) { return ::closesocket(s); }
inline int poll_sockets(poll_fd_t *f, unsigned long n, int t) { return ::WSAPoll(f, n, t); }
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
using socket_t = int;
using poll_fd_t = struct pollfd;
constexpr socket_t kInvalidSocket = -1;
inline int close_socket(socket_t s) { return ::close(s); }
inline int poll_sockets(poll_fd_t *f, unsigned long n, int t) { return ::poll(f, n, t); }
#endif
