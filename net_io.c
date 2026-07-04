// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// net_io.c: network handling.
//
// Copyright (c) 2019 Michael Wolf <michael@mictronics.de>
//
// This code is based on a detached fork of dump1090-fa.
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This file incorporates work covered by the following copyright and
// license:
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "ais_charset.h"
#include "readsb.h"

#include <assert.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>

#include "uat2esnt/uat2esnt.h"

#include "net_beast.h"
#include "net_sbs.h"
#include "net_asterix.h"
#include "net_uat.h"
#include "net_planefinder.h"
#include "net_gpsd.h"

// ============================= Networking =============================
//

// read_fn typedef read_handler functions
static int handleCommandSocket(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb);
static int processHexMessage(struct client *c, char *hex, int remote, int64_t now, struct messageBuffer *mb);
// end read handlers

static void send_heartbeat(struct net_service *service);

static void *pthreadGetaddrinfo(void *param);

static void modesReadFromClient(struct client *c, struct messageBuffer *mb);

static void drainMessageBuffer(struct messageBuffer *buf);

// ModeAC all zero messag
static const char beast_heartbeat_msg[] = {0x1a, '1', 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const char raw_heartbeat_msg[] = "*0000;\n";
static const char sbs_heartbeat_msg[] = "\r\n"; // is there a better one?
//static const char newline_heartbeat_msg[] = "\n";
// CAUTION: sizeof includes the trailing \0 byte

static const heartbeat_t beast_heartbeat = {
    .msg = beast_heartbeat_msg,
    .len = sizeof(beast_heartbeat_msg)
};
static const heartbeat_t raw_heartbeat = {
    .msg = raw_heartbeat_msg,
    .len = sizeof(raw_heartbeat_msg) - 1
};
static const heartbeat_t sbs_heartbeat = {
    .msg = sbs_heartbeat_msg,
    .len = sizeof(sbs_heartbeat_msg) - 1
};
static const heartbeat_t no_heartbeat = {
    .msg = NULL,
    .len = 0
};
/*
static const heartbeat_t newline_heartbeat = {
    .msg = newline_heartbeat_msg,
    .len = sizeof(newline_heartbeat_msg) - 1
};
*/

//
//=========================================================================
//
// Networking "stack" initialization
//

// Init a service with the given read/write characteristics, return the new service.
// Doesn't arrange for the service to listen or connect
static struct net_service *serviceInit(struct net_service_group *group, const char *descr, struct net_writer *writer, heartbeat_t heartbeat_out, heartbeat_t heartbeat_in, read_mode_t mode, const char *sep, read_fn handler) {
    if (!descr) {
        fprintf(stderr, "Fatal: no service description\n");
        exit(1);
    }

    if (!group->services) {
        group->alloc = NET_SERVICE_GROUP_MAX;
        group->services = cmalloc(group->alloc * sizeof(struct net_service));
    }

    if (!group->services) {
    }

    group->len++;
    if (group->len + 1 > group->alloc) {
        fprintf(stderr, "FATAL: Increase NET_SERVICE_GROUP_MAX\n");
        exit(1);
    }

    struct net_service *service = &group->services[group->len - 1];
    memset(service, 0, 2 * sizeof(struct net_service));
    // also set the extra service to zero, zero terminatd array

    service->group = group;
    service->descr = descr;
    service->listener_count = 0;
    service->pusher_count = 0;
    service->connections = 0;
    service->writer = writer;
    service->read_sep = sep;
    service->read_sep_len = sep ? strlen(sep) : 0;
    service->read_mode = mode;
    service->read_handler = handler;
    service->clients = NULL;

    service->heartbeat_out = heartbeat_out;
    service->heartbeat_in = heartbeat_in;

    if (service->writer) {
        if (service->writer->data) {
            fprintf(stderr, "FATAL: serviceInit() called twice on the same service: %s\n", descr);
            exit(1);
        }

        // set writer to zero
        memset(service->writer, 0, sizeof(struct net_writer));

        service->writer->data = cmalloc(Modes.writerBufSize);

        service->writer->service = service;
        service->writer->dataUsed = 0;
        service->writer->lastWrite = mstime();
        service->writer->lastReceiverId = 0;
        service->writer->connections = 0;

        if (service->writer == &Modes.beast_reduce_out) {
            service->writer->flushInterval = Modes.net_output_flush_interval_beast_reduce;
        } else {
            service->writer->flushInterval = Modes.net_output_flush_interval;
        }
    }

    return service;
}

static int sendFiveHeartbeats(struct client *c, int64_t now) {
    // only send 5 heartbeats for beast type output
    if (c->service->heartbeat_out.msg != beast_heartbeat.msg) {
        return 0;
    }
    //fprintf(stderr, "sending 5 hbs\n");
    // send 5 heartbeats to signal that we are a client that can accomodate feedback .... some counterparts crash if they get stuff they don't understand
    // this is really a crutch, but there is no other good way to signal this without causing issues
    int repeats = 5;
    const char *heartbeat_msg = c->service->heartbeat_out.msg;
    int heartbeat_len = c->service->heartbeat_out.len;

    if (heartbeat_msg && c->sendq && c->sendq_len + repeats * heartbeat_len < c->sendq_max) {
        for (int k = 0; k < repeats; k++) {
            memcpy(c->sendq + c->sendq_len, heartbeat_msg, heartbeat_len);
            c->sendq_len += heartbeat_len;
        }
    }
    return flushClient(c, now);
}



static void setProxyString(struct client *c) {
    snprintf(c->proxy_string, sizeof(c->proxy_string), "%s port %s", c->host, c->port);
    if (!c->receiverIdLocked) {
        c->receiverId = fasthash64(c->proxy_string, strlen(c->proxy_string), 0x2127599bf4325c37ULL);
    }
}

static int getSNDBUF(struct net_service *service) {
    if (service->sendqOverrideSize) {
        return service->sendqOverrideSize;
    } else {
        return Modes.netBufSize;
    }
}
static int getRCVBUF(struct net_service *service) {
    if (service->recvqOverrideSize) {
        return service->recvqOverrideSize;
    } else {
        return Modes.netBufSize;
    }
}
static void setSockopts(struct client *c) {

    if (anetTcpNoDelay(Modes.aneterr, c->fd) != ANET_OK) {
        if (c->con) {
            fprintf(stderr, "%s: Unable to set TCP_NODELAY: connection to %s port %s ...\n", c->con->service->descr, c->con->address, c->con->port);
        } else {
            fprintf(stderr, "%s: Unable to set TCP_NODELAY on connection from %s local port %s\n", c->service->descr, c->host, c->port);
        }
    }
    if (anetTcpKeepAlive(Modes.aneterr, c->fd) != ANET_OK) {
        if (c->con) {
            fprintf(stderr, "%s: Unable to set keepalive: connection to %s port %s ...\n", c->con->service->descr, c->con->address, c->con->port);
        } else {
            fprintf(stderr, "%s: Unable to set keepalive on connection from %s local port %s\n", c->service->descr, c->host, c->port);
        }
    }

    if (Modes.tcpBuffersAuto) {
        return;
    }
    // explicitely setting tcp buffers causes failure of linux tcp window auto tuning
    // let's not dealy with this, set Modes.tcpBuffersAuto to 1 for the time being

    int sndsize = getSNDBUF(c->service);
    if (sndsize > 0 && setsockopt(c->fd, SOL_SOCKET, SO_SNDBUF, (void*)&sndsize, sizeof(sndsize)) == -1) {
        fprintf(stderr, "setsockopt SO_SNDBUF: %s", strerror(errno));
    }

    if (0) {
        int rcvsize = getRCVBUF(c->service);
        // much better to just let the OS handle the receive buffer
        if (rcvsize > 0 && setsockopt(c->fd, SOL_SOCKET, SO_RCVBUF, (void*)&rcvsize, sizeof(rcvsize)) == -1) {
            fprintf(stderr, "setsockopt SO_RCVBUF: %s", strerror(errno));
        }
    }
}

// Create a client attached to the given service using the provided socket FD ... not a socket in some exceptions
static struct client *createSocketClient(struct net_service *service, int fd, char *uuid) {
    struct client *c;
    int64_t now = mstime();

    if (!service || fd == -1) {
        fprintf(stderr, "<3> FATAL: createSocketClient called with invalid parameters!\n");
        exit(1);
    }
    if (!(c = (struct client *) cmalloc(sizeof (struct client)))) {
        fprintf(stderr, "<3> FATAL: Out of memory allocating a new %s network client\n", service->descr);
        exit(1);
    }
    memset(c, 0, sizeof (struct client));

    c->service = service;
    c->fd = fd;
    c->last_send = now;
    c->last_read = now;
    c->connectedSince = now;
    c->last_read_flush = now;


    c->proxy_string[0] = '\0';
    c->host[0] = '\0';
    c->port[0] = '\0';

    c->receiverIdLocked = 0;
    c->receiverId2 = 0;

    if (uuid != NULL) {
        read_uuid(c, uuid, uuid + strlen(uuid));
        if (c->receiverIdLocked) {
            //fprintf(stderr, "Using supplied uuid %s, c->receiverId: %016"PRIx64"\n", uuid, c->receiverId);
        }
    } else {
        c->receiverId = random();
        c->receiverId <<= 22;
        c->receiverId ^= random();
        c->receiverId <<= 22;
        c->receiverId ^= random();
        //fprintf(stderr, "preliminary random uuid might be overwritten, c->receiverId: %016"PRIx64"\n", c->receiverId);
    }

    c->recent_rtt = -1;

    c->remote = 1; // Messages will be marked remote by default
    if ((c->fd == Modes.beast_fd) && (SdrConfig.sdr_type == SDR_MODESBEAST || SdrConfig.sdr_type == SDR_GNS)) {
        /* Message from a local connected Modes-S beast or GNS5894 are passed off the internet */
        c->remote = 0;
        c->serial = 1;
    }

    //fprintf(stderr, "c->receiverId: %016"PRIx64"\n", c->receiverId);

    c->bufmax = Modes.netBufSize;
    if (service->recvqOverrideSize) {
        c->bufmax = service->recvqOverrideSize;
    }

    c->buf = cmalloc(c->bufmax);

    if (service->writer) {
        c->sendq_max = Modes.netBufSize;
        if (service->sendqOverrideSize) {
            c->sendq_max = service->sendqOverrideSize;
        }
        if (!(c->sendq = cmalloc(c->sendq_max))) {
            fprintf(stderr, "Out of memory allocating client SendQ\n");
            exit(1);
        }

        service->writer->connections++;
    }
    service->connections++;
    Modes.modesClientCount++;

    c->next = service->clients;
    service->clients = c;


    if (Modes.debug_net && service->connections % 50 == 0) {
        fprintf(stderr, "%s connection count: %d\n", service->descr, service->connections);
    }

    if (service->writer && service->connections == 1) {
        service->writer->lastWrite = now; // suppress heartbeat initially
    }

    epoll_data_t data;
    data.ptr = c;
    c->epollEvent.events = EPOLLIN | EPOLLRDHUP | EPOLLERR | EPOLLHUP;
    c->epollEvent.data = data;
    if (epoll_ctl(Modes.net_epfd, EPOLL_CTL_ADD, c->fd, &c->epollEvent))
        perror("epoll_ctl fail:");

    return c;
}

static int sendUUID(struct client *c, int64_t now) {
    struct net_connector *con = c->con;
    // sending UUID for beast_reduce_plus output
    char uuid[150];
    uuid[0] = '\0';
    if ((c->sendq && c->sendq_len + 256 < c->sendq_max) && con
            && (con->enable_uuid_ping || Modes.debug_ping || Modes.debug_send_uuid)) {

        int res = -1;

        if (con->uuid) {
            strncpy(uuid, con->uuid, 135);
            res = strlen(uuid);
        } else if (Modes.uuidFile) {
            int fd = open(Modes.uuidFile, O_RDONLY);
            if (fd != -1) {
                res = read(fd, uuid, 128);
                close(fd);
            }
        }

        if (res >= 28) {
            if (uuid[res - 1] == '\n') {
                // remove trailing newline
                res--;
            }
            uuid[res] = '\0';

            c->sendq[c->sendq_len++] = 0x1A;
            c->sendq[c->sendq_len++] = 0xE4;
            // uuid is padded with 'f', always send 36 chars
            memset(c->sendq + c->sendq_len, 'f', 36);
            strncpy(c->sendq + c->sendq_len, uuid, res);
            c->sendq_len += 36;
        } else {
            fprintf(stderr, "ERROR: Not a valid UUID: '%s' (to generate a valid uuid use this command: cat /proc/sys/kernel/random/uuid)\n", uuid);
            uuid[0] = '\0';
        }

        // enable ping stuff
        // O for high resolution timer, both P and p already used for previous iterations
        c->sendq[c->sendq_len++] = 0x1a;
        c->sendq[c->sendq_len++] = 'W';
        c->sendq[c->sendq_len++] = 'O';
        return flushClient(c, now);
    }
    return -1;
}

static int suppressConnectError(struct net_connector *con) {
    uint32_t failLimit = 10;
    con->fail_counter += 1; // increment fail counter
    if (con->silent_fail) {
        return 1;
    }
    if (con->fail_counter < failLimit || Modes.debug_net) {
        return 0;
    }
    if (con->fail_counter == failLimit || con->fail_counter % 200 == 0) {
        fprintf(stderr, "%s: Connection to %s port %s failed %u times, suppressing most error messages until connection succeeds\n",
                con->service->descr, con->address, con->port, con->fail_counter);
    }
    return 1;
}

static void checkServiceConnected(struct net_connector *con, int64_t now) {

    if (!con->connecting) {
        return;
    }

    //fprintf(stderr, "checkServiceConnected fd: %d\n", con->fd);
    // delete dummyClient epollEvent for connection that is being established
    epoll_ctl(Modes.net_epfd, EPOLL_CTL_DEL, con->fd, &con->dummyClient.epollEvent);
    // we'll register new epollEvents in createSocketClient

    // At this point, we need to check getsockopt() to see if we succeeded or failed...
    int optval = -1;
    socklen_t optlen = sizeof(optval);
    if (getsockopt(con->fd, SOL_SOCKET, SO_ERROR, &optval, &optlen) == -1) {
        fprintf(stderr, "getsockopt failed: %d (%s)\n", errno, strerror(errno));
        // Bad stuff going on, but clear this anyway
        con->connecting = 0;
        anetCloseSocket(con->fd);
        return;
    }

    if (optval != 0) {
        // only 0 means "connection ok"

        if (!suppressConnectError(con)) {
            fprintf(stderr, "%s: Connection to %s%s port %s failed (%u): %d (%s)\n",
                    con->service->descr, con->address, con->resolved_addr, con->port, con->fail_counter, optval, strerror(optval));
        }
        con->connecting = 0;
        anetCloseSocket(con->fd);
        return;
    }

    // If we're able to create this "client", save the sockaddr info and print a msg
    struct client *c;

    c = createSocketClient(con->service, con->fd, con->uuid);
    if (!c) {
        con->connecting = 0;
        fprintf(stderr, "createSocketClient failed on fd %d to %s%s port %s\n",
                con->fd, con->address, con->resolved_addr, con->port);
        anetCloseSocket(con->fd);
        return;
    }

    strncpy(c->host, con->address, sizeof(c->host) - 1);
    strncpy(c->port, con->port, sizeof(c->port) - 1);
    setProxyString(c);

    con->connecting = 0;
    con->connected = 1;
    con->lastConnect = now;
    // link connection and client so we have access from one to the other
    c->con = con;
    con->c = c;

    int uuid_sent = (sendUUID(c, now) == 0);

    sendFiveHeartbeats(c, now);

    if ((c->sendq && c->sendq_len + 256 < c->sendq_max)
                && strcmp(con->protocol, "gpsd_in") == 0) {
        if (Modes.debug_gps) {
            fprintTime(stderr, now);
            fprintf(stderr, " gpsdebug: sending \'?WATCH={\"enable\":true,\"json\":true};\\n\'\n");
        }
        c->sendq_len += snprintf(c->sendq, 256, "?WATCH={\"enable\":true,\"json\":true};\n");
        if (flushClient(c, now) < 0) {
            return;
        }
    }
    if (!Modes.interactive) {
        if (uuid_sent) {
            fprintf(stderr, "%s: Connection established: %s%s port %s (sent UUID)\n",
                    con->service->descr, con->address, con->resolved_addr, con->port);
        } else {
            fprintf(stderr, "%s: Connection established: %s%s port %s\n",
                    con->service->descr, con->address, con->resolved_addr, con->port);
        }
    }

    con->fail_counter = 0; // reset fail counter on successful connection
}

// Initiate an outgoing connection.
static void serviceConnect(struct net_connector *con, int64_t now) {

    int fd;

    // make sure backoff is never too small
    con->backoff = imax(Modes.net_connector_delay_min, con->backoff);

    if (con->try_addr) {
        // iterate the address info linked list if we have one
        con->try_addr = con->try_addr->ai_next;
    }

    if (!con->try_addr)  {
        // if ((!con->addr_info || now - con->lastResolve > 2 * Modes.net_connector_delay) && !con->gai_request_in_progress)  {
        // keeping a DNS reply for any length of time without knowing lifetime is a bad idea
        // cause issues with recreating docker containers and connecting to the wrong container due
        // to the caching of the address info
        if (!con->gai_request_in_progress)  {
            // launch a pthread for async getaddrinfo
            if (con->addr_info) {
                freeaddrinfo(con->addr_info);
                con->addr_info = NULL;
            }

            pthread_mutex_lock(&con->mutex);
            con->gai_request_done = 0;
            pthread_mutex_unlock(&con->mutex);

            if (0 && Modes.debug_net) {
                fprintf(stderr, "%s: calling getaddrinfo for %s port %s\n", con->service->descr, con->address, con->port);
            }

            if (pthread_create(&con->thread, NULL, pthreadGetaddrinfo, con)) {
                con->next_reconnect = now + Modes.net_connector_delay;
                fprintf(stderr, "%s: pthread_create ERROR for %s port %s: %s\n", con->service->descr, con->address, con->port, strerror(errno));
                return;
            }

            con->gai_request_in_progress = 1;
            con->next_reconnect = now + 20;
            return;
        }

        if (con->gai_request_in_progress) {
            // gai request is in progress, let's check if it's done

            pthread_mutex_lock(&con->mutex);
            if (!con->gai_request_done) {
                con->next_reconnect = now + 20;
                pthread_mutex_unlock(&con->mutex);
                return;
            }
            pthread_mutex_unlock(&con->mutex);

            con->gai_request_in_progress = 0;
            // gai request is done, join the thread that performed it
            if (pthread_join(con->thread, NULL)) {
                fprintf(stderr, "%s: pthread_join ERROR for %s port %s: %s\n", con->service->descr, con->address, con->port, strerror(errno));
                con->next_reconnect = now + Modes.net_connector_delay;
                return;
            }

            if (con->gai_error) {
                if (!con->silent_fail) {
                    fprintf(stderr, "%s: Name resolution for %s failed: %s\n", con->service->descr, con->address, gai_strerror(con->gai_error));
                }
                // limit name resolution attempts via backoff
                con->next_reconnect = now + con->backoff;
                con->backoff = imin(Modes.net_connector_delay, 2 * con->backoff);
                return;
            }
            con->lastResolve = now;
            // SUCCESS, we got the address info
        }

        // start with the first element of the linked list
        con->try_addr = con->addr_info;
    }

    // limit tcp connection attemtps via backoff
    con->next_reconnect = now + con->backoff;
    if (!con->try_addr->ai_next) {
        // only increase backoff once all DNS results have been tried
        con->backoff = imin(Modes.net_connector_delay, 2 * con->backoff);
    }

    struct timespec watch;
    startWatch(&watch);

    getnameinfo(con->try_addr->ai_addr, con->try_addr->ai_addrlen,
            con->resolved_addr, sizeof(con->resolved_addr) - 3,
            NULL, 0,
            NI_NUMERICHOST | NI_NUMERICSERV);

    int64_t getnameinfoElapsed = lapWatch(&watch);
    if (getnameinfoElapsed > 1) {
        fprintf(stderr, "WARNING: getnameinfo() took %"PRId64" ms\n", getnameinfoElapsed);
    }

    if (
            strcmp(con->resolved_addr, con->address) != 0
            && (
                strcmp(con->resolved_addr, "::") == 0
                || strcmp(con->resolved_addr, "0.0.0.0") == 0
               )
       ) {
        fprintf(stderr, "%s: %s port %s: Ignoring this DNS reply: %s\n", con->service->descr, con->address, con->port, con->resolved_addr);
        return;
    }

    if (strcmp(con->resolved_addr, con->address) == 0) {
        con->resolved_addr[0] = '\0';
    } else {
        char tmp[sizeof(con->resolved_addr)+3]; // shut up gcc
        snprintf(tmp, sizeof(tmp), " (%s)", con->resolved_addr);
        memcpy(con->resolved_addr, tmp, sizeof(con->resolved_addr));
    }

    if (Modes.debug_net) {
        //fprintf(stderr, "%s: Attempting connection to %s port %s ... (gonna set SNDBUF %d RCVBUF %d)\n", con->service->descr, con->address, con->port, getSNDBUF(con->service), getRCVBUF(con->service));
        fprintf(stderr, "%s: Attempting connection to %s%s port %s ...\n", con->service->descr, con->address, con->resolved_addr, con->port);
    }

    struct addrinfo *ai = con->try_addr;
    fd = anetCreateSocket(Modes.aneterr, ai->ai_family, SOCK_NONBLOCK);

    if (fd == ANET_ERR) {
        if (!suppressConnectError(con)) {
            fprintf(stderr, "%s: Connection to %s%s port %s failed: %s\n",
                    con->service->descr, con->address, con->resolved_addr, con->port, Modes.aneterr);
        }
        return;
    }

    con->fd = fd;
    con->connecting = 1;
    con->connect_timeout = now + imin(Modes.net_connector_delay, 5000); // really if your connection won't establish after 5 seconds ... tough luck.
    //fprintf(stderr, "connect_timeout: %ld\n", (long) (con->connect_timeout - now));

    // struct client for epoll purposes
    struct client *c = &con->dummyClient;

    c->service = con->service;
    c->fd = con->fd;
    c->net_connector_dummyClient = 1;
    c->epollEvent.events = EPOLLIN | EPOLLOUT | EPOLLRDHUP | EPOLLERR | EPOLLHUP;
    c->epollEvent.data.ptr = c;
    c->con = con;

    if (epoll_ctl(Modes.net_epfd, EPOLL_CTL_ADD, c->fd, &c->epollEvent)) {
        perror("epoll_ctl fail:");
    }

    setSockopts(c);

    if (connect(fd, ai->ai_addr, ai->ai_addrlen) < 0 && errno != EINPROGRESS) {
        epoll_ctl(Modes.net_epfd, EPOLL_CTL_DEL, con->fd, &con->dummyClient.epollEvent);
        con->connecting = 0;
        anetCloseSocket(con->fd);
        if (!suppressConnectError(con)) {
            fprintf(stderr, "%s: Connection to %s%s port %s failed: %s\n",
                    con->service->descr, con->address, con->resolved_addr, con->port, strerror(errno));
        }
    }
}

// Timer callback checking periodically whether the push service lost its server
// connection and requires a re-connect.
static void serviceReconnectCallback(int64_t now) {
    // Loop through the connectors, and
    //  - If it's not connected:
    //    - If it's "connecting", check to see if it timed out
    //    - Otherwise, if enough time has passed, try reconnecting

    for (int i = 0; i < Modes.net_connectors_count; i++) {
        struct net_connector *con = &Modes.net_connectors[i];
        if (!con->connected) {
            // If we've exceeded our connect timeout, close connection.
            if (con->connecting && now >= con->connect_timeout) {
                if (!suppressConnectError(con)) {
                    fprintf(stderr, "%s: Connection to %s%s port %s timed out.\n",
                            con->service->descr, con->address, con->resolved_addr, con->port);
                }
                con->connecting = 0;
                // delete dummyClient epollEvent for connection that is being established
                epoll_ctl(Modes.net_epfd, EPOLL_CTL_DEL, con->fd, &con->dummyClient.epollEvent);
                anetCloseSocket(con->fd);
            }

            //fprintf(stderr, "next_reconnect in: %lld\n", (long long) (con->next_reconnect - now));
            if (!con->connecting && (now >= con->next_reconnect || Modes.synthetic_now)) {
                serviceConnect(con, now);
            }
        } else {
            // check for idle connection, this server version requires data
            // or a heartbeat, otherwise it will force a reconnect
            struct client *c = con->c;
            if (Modes.net_heartbeat_interval && c
                    && now - c->last_read > 2 * Modes.net_heartbeat_interval
                    && c->service->heartbeat_in.msg != NULL
               ) {
                fprintf(stderr, "%s: No data or heartbeat received for %.0f seconds, reconnecting: %s port %s\n",
                        c->service->descr, (2 * Modes.net_heartbeat_interval) / 1000.0, c->host, c->port);
                modesCloseClient(c);
            }
        }
    }
}

// Set up the given service to listen on an address/port.
// _exits_ on failure!
void serviceListen(struct net_service *service, char *bind_addr, char *bind_ports, int epfd) {
    int *fds = NULL;
    int n = 0;
    char *p, *end;
    char buf[128];

    if (service->listener_count > 0) {
        fprintf(stderr, "Tried to set up the service %s twice!\n", service->descr);
        exit(1);
    }

    if (!bind_ports || !strcmp(bind_ports, "") || !strcmp(bind_ports, "0"))
        return;

    if (0 && Modes.debug_net) {
        fprintf(stderr, "serviceListen: %s with SNDBUF %d RCVBUF %d)\n", service->descr,  getSNDBUF(service), getRCVBUF(service));
    }

    p = bind_ports;
    while (p && *p) {
        int newfds[16];
        int nfds, i;

        int is_unix = 0;
        if (strncmp(p, "unix:", 5) == 0) {
            is_unix = 1;
            p += 5;
        }

        end = strpbrk(p, ", ");

        if (!end) {
            strncpy(buf, p, sizeof (buf) - 1);
            buf[sizeof (buf) - 1] = 0;
            p = NULL;
        } else {
            size_t len = end - p;
            if (len >= sizeof (buf))
                len = sizeof (buf) - 1;
            memcpy(buf, p, len);
            buf[len] = 0;
            p = end + 1;
        }
        if (is_unix) {
            if (service->unixSocket) {
                fprintf(stderr, "Multiple unix sockets per service are not supported! %s (%s): %s\n",
                        buf, service->descr, Modes.aneterr);
                exit(1);
            }

            sfree(service->unixSocket);
            service->unixSocket = strdup(buf);

            unlink(service->unixSocket);
            int fd = anetUnixSocket(Modes.aneterr, buf, SOCK_NONBLOCK);
            if (fd == ANET_ERR) {
                fprintf(stderr, "Error opening the listening port %s (%s): %s\n",
                        buf, service->descr, Modes.aneterr);
                exit(1);
            }
            if (chmod(service->unixSocket, 0666) != 0) {
                perror("serviceListen: couldn't set permissions for unix socket due to:");
            }
            newfds[0] = fd;
            nfds = 1;
        } else {
            //nfds = anetTcpServer(Modes.aneterr, buf, bind_addr, newfds, sizeof (newfds), SOCK_NONBLOCK, getSNDBUF(service), getRCVBUF(service));
            // explicitely setting tcp buffers causes failure of linux tcp window auto tuning ... it just doesn't work well without the auto tuning
            nfds = anetTcpServer(Modes.aneterr, buf, bind_addr, newfds, sizeof (newfds), SOCK_NONBLOCK, -1, -1);
            if (nfds == ANET_ERR) {
                fprintf(stderr, "Error opening the listening port %s (%s): %s\n",
                        buf, service->descr, Modes.aneterr);
                exit(1);
            }
        }

        char listenString[1024];
        snprintf(listenString, 1023, "%5s: %s port", buf, service->descr);
        fprintf(stderr, "%-38s\n", listenString);

        fds = realloc(fds, (n + nfds) * sizeof (int));
        if (!fds) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }

        for (i = 0; i < nfds; ++i) {
            fds[n++] = newfds[i];
        }
    }

    service->listener_count = n;
    service->listener_fds = fds;

    if (epfd >= 0) {
        service->listenSockets = cmalloc(service->listener_count * sizeof(struct client));
        memset(service->listenSockets, 0, service->listener_count * sizeof(struct client));
        for (int i = 0; i < service->listener_count; ++i) {

            // struct client for epoll purposes for each listen socket.
            struct client *c = &service->listenSockets[i];

            c->service = service;
            c->fd = service->listener_fds[i];
            c->acceptSocket = 1;
            c->epollEvent.events = EPOLLIN | EPOLLRDHUP | EPOLLERR | EPOLLHUP;
            c->epollEvent.data.ptr = c;

            if (epoll_ctl(epfd, EPOLL_CTL_ADD, c->fd, &c->epollEvent))
                perror("epoll_ctl fail:");

        }
    }
}
static void initMessageBuffers() {
    if (Modes.decodeThreads > 1) {
        pthread_mutex_init(&Modes.decodeLock, NULL);
        pthread_mutex_init(&Modes.trackLock, NULL);
        pthread_mutex_init(&Modes.outputLock, NULL);

        Modes.decodeTasks = allocate_task_group(Modes.decodeThreads);
        Modes.decodePool = threadpool_create(Modes.decodeThreads, 0);
    }

    Modes.netMessageBuffer = cmalloc(Modes.decodeThreads * sizeof(struct messageBuffer));
    memset(Modes.netMessageBuffer, 0x0, Modes.decodeThreads * sizeof(struct messageBuffer));
    for (int k = 0; k < Modes.decodeThreads; k++) {
        struct messageBuffer *buf = &Modes.netMessageBuffer[k];
        buf->alloc = 256 << Modes.net_sndbuf_size;
        buf->len = 0;
        buf->id = k;
        buf->activeClient = NULL;
        int bytes = buf->alloc * sizeof(struct modesMessage);
        buf->msg = cmalloc(bytes);
        //fprintf(stderr, "netMessageBuffer alloc: %d size: %d\n", buf->alloc, bytes);
    }
}

void modesInitNet(void) {
    initMessageBuffers();

    uat2esnt_initCrcTables();

    if (0) {
        char *msg[4] = { "-00a974f135362f522fc408c9122e1b015900;",
             "-08a78bea35705f5283880459010227605809e00d40a2040be2a5c2a00004a0000000;rs=2;",
             "-10ad7233358a9d528bc40aa900be3120880000000000000000000000000b10000000;rs=4;",
             "-10a78bea3570b152830c0449010626e04800000000000000000000000004a0000000;rs=2;" };

        for (int i = 0; i < 4; i++) {
            char output[2048];
            uat2esnt_convert_message(msg[i], msg[i] + strlen(msg[i]), output, output + sizeof(output));
            fprintf(stderr, "%s\n", output);
        }
        exit(1);
    }

    Modes.net_connector_delay_min = imax(50, Modes.net_connector_delay / 64);
    Modes.last_connector_fail = Modes.next_reconnect_callback = mstime();

    if (!Modes.net)
        return;
    struct net_service *beast_out;
    struct net_service *beast_reduce_out;
    struct net_service *garbage_out;
    struct net_service *uat_replay_service;
    struct net_service *raw_out;
    struct net_service *raw_in;
    struct net_service *vrs_out;
    struct net_service *json_out;
    struct net_service *feedmap_out;
    struct net_service *sbs_out;
    struct net_service *sbs_out_replay;
    struct net_service *sbs_out_mlat;
    struct net_service *sbs_out_jaero;
    struct net_service *sbs_out_prio;
    struct net_service *asterix_out;
    struct net_service *asterix_in;
    struct net_service *sbs_in;
    struct net_service *sbs_in_mlat;
    struct net_service *sbs_in_jaero;
    struct net_service *sbs_in_prio;
    struct net_service *gpsd_in;
    struct net_service *planefinder_in;

    signal(SIGPIPE, SIG_IGN);

    Modes.net_epfd = my_epoll_create(&Modes.exitNowEventfd);

    // set up listeners
    raw_out = serviceInit(&Modes.services_out, "Raw TCP output", &Modes.raw_out, raw_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(raw_out, Modes.net_bind_address, Modes.net_output_raw_ports, Modes.net_epfd);

    uat_replay_service = serviceInit(&Modes.services_out, "UAT TCP replay output", &Modes.uat_replay_out, no_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(uat_replay_service, Modes.net_bind_address, Modes.net_output_uat_replay_ports, Modes.net_epfd);

    beast_out = serviceInit(&Modes.services_out, "Beast TCP output", &Modes.beast_out, beast_heartbeat, no_heartbeat, READ_MODE_BEAST_COMMAND, NULL, handleBeastCommand);
    serviceListen(beast_out, Modes.net_bind_address, Modes.net_output_beast_ports, Modes.net_epfd);

    beast_reduce_out = serviceInit(&Modes.services_out, "BeastReduce TCP output", &Modes.beast_reduce_out, beast_heartbeat, no_heartbeat, READ_MODE_BEAST_COMMAND, NULL, handleBeastCommand);
    serviceListen(beast_reduce_out, Modes.net_bind_address, Modes.net_output_beast_reduce_ports, Modes.net_epfd);

    garbage_out = serviceInit(&Modes.services_out, "Garbage TCP output", &Modes.garbage_out, beast_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(garbage_out, Modes.net_bind_address, Modes.garbage_ports, Modes.net_epfd);

    vrs_out = serviceInit(&Modes.services_out, "VRS json output", &Modes.vrs_out, no_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(vrs_out, Modes.net_bind_address, Modes.net_output_vrs_ports, Modes.net_epfd);

    json_out = serviceInit(&Modes.services_out, "Position json output", &Modes.json_out, no_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(json_out, Modes.net_bind_address, Modes.net_output_json_ports, Modes.net_epfd);

    feedmap_out = serviceInit(&Modes.services_out, "Forward feed map data", &Modes.feedmap_out, no_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);

    sbs_out = serviceInit(&Modes.services_out, "SBS TCP output ALL", &Modes.sbs_out, sbs_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(sbs_out, Modes.net_bind_address, Modes.net_output_sbs_ports, Modes.net_epfd);

    sbs_out_replay = serviceInit(&Modes.services_out, "SBS TCP output MAIN", &Modes.sbs_out_replay, sbs_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    sbs_out_prio = serviceInit(&Modes.services_out, "SBS TCP output PRIO", &Modes.sbs_out_prio, sbs_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    sbs_out_mlat = serviceInit(&Modes.services_out, "SBS TCP output MLAT", &Modes.sbs_out_mlat, sbs_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    sbs_out_jaero = serviceInit(&Modes.services_out, "SBS TCP output JAERO", &Modes.sbs_out_jaero, sbs_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);

    serviceListen(sbs_out_jaero, Modes.net_bind_address, Modes.net_output_jaero_ports, Modes.net_epfd);


    int sbs_port_len = strlen(Modes.net_output_sbs_ports);
    int pos = sbs_port_len - 1;
    if (sbs_port_len <= 5 && Modes.net_output_sbs_ports[pos] == '5') {

        char *replay = strdup(Modes.net_output_sbs_ports);
        replay[pos] = '6';
        serviceListen(sbs_out_replay, Modes.net_bind_address, replay, Modes.net_epfd);

        char *mlat = strdup(Modes.net_output_sbs_ports);
        mlat[pos] = '7';
        serviceListen(sbs_out_mlat, Modes.net_bind_address, mlat, Modes.net_epfd);

        char *prio = strdup(Modes.net_output_sbs_ports);
        prio[pos] = '8';
        serviceListen(sbs_out_prio, Modes.net_bind_address, prio, Modes.net_epfd);

        char *jaero = strdup(Modes.net_output_sbs_ports);
        jaero[pos] = '9';
        if (sbs_out_jaero->listener_count == 0)
            serviceListen(sbs_out_jaero, Modes.net_bind_address, jaero, Modes.net_epfd);

        sfree(replay);
        sfree(mlat);
        sfree(prio);
        sfree(jaero);
    }

    sbs_in = serviceInit(&Modes.services_in, "SBS TCP input MAIN", NULL, no_heartbeat, sbs_heartbeat, READ_MODE_ASCII, "\n",  decodeSbsLine);
    serviceListen(sbs_in, Modes.net_bind_address, Modes.net_input_sbs_ports, Modes.net_epfd);

    sbs_in_mlat = serviceInit(&Modes.services_in, "SBS TCP input MLAT", NULL, no_heartbeat, sbs_heartbeat, READ_MODE_ASCII, "\n",  decodeSbsLineMlat);
    sbs_in_prio = serviceInit(&Modes.services_in, "SBS TCP input PRIO", NULL, no_heartbeat, sbs_heartbeat, READ_MODE_ASCII, "\n",  decodeSbsLinePrio);
    sbs_in_jaero = serviceInit(&Modes.services_in, "SBS TCP input JAERO", NULL, no_heartbeat, sbs_heartbeat, READ_MODE_ASCII, "\n",  decodeSbsLineJaero);


    serviceListen(sbs_in_jaero, Modes.net_bind_address, Modes.net_input_jaero_ports, Modes.net_epfd);

    sbs_port_len = strlen(Modes.net_input_sbs_ports);
    pos = sbs_port_len - 1;
    if (sbs_port_len <= 5 && Modes.net_input_sbs_ports[pos] == '6') {
        char *mlat = strdup(Modes.net_input_sbs_ports);
        mlat[pos] = '7';
        serviceListen(sbs_in_mlat, Modes.net_bind_address, mlat, Modes.net_epfd);

        char *prio = strdup(Modes.net_input_sbs_ports);
        prio[pos] = '8';
        serviceListen(sbs_in_prio, Modes.net_bind_address, prio, Modes.net_epfd);

        char *jaero = strdup(Modes.net_input_sbs_ports);
        jaero[pos] = '9';
        if (sbs_in_jaero->listener_count == 0)
            serviceListen(sbs_in_jaero, Modes.net_bind_address, jaero, Modes.net_epfd);

        sfree(mlat);
        sfree(prio);
        sfree(jaero);
    }

    asterix_out = serviceInit(&Modes.services_out, "ASTERIX output", &Modes.asterix_out, no_heartbeat, no_heartbeat, READ_MODE_IGNORE, NULL, NULL);
    serviceListen(asterix_out, Modes.net_bind_address, Modes.net_output_asterix_ports, Modes.net_epfd);

    asterix_in = serviceInit(&Modes.services_in, "ASTERIX TCP input", NULL, no_heartbeat, no_heartbeat, READ_MODE_ASTERIX, NULL, decodeAsterixMessage);
    serviceListen(asterix_in, Modes.net_bind_address, Modes.net_input_asterix_ports, Modes.net_epfd);

    gpsd_in = serviceInit(&Modes.services_in, "GPSD TCP input", &Modes.gpsd_in, no_heartbeat, no_heartbeat, READ_MODE_ASCII, "\n", handle_gpsd);

    if (Modes.json_dir && Modes.json_globe_index && Modes.globe_history_dir) {
        /* command input */
        struct net_service *commandService = serviceInit(&Modes.services_in, "command input", NULL, no_heartbeat, no_heartbeat, READ_MODE_ASCII, "\n", handleCommandSocket);
        char commandSocketFile[PATH_MAX];
        char commandSocket[PATH_MAX];
        snprintf(commandSocketFile, PATH_MAX, "%s/cmd.sock", Modes.json_dir);
        unlink(commandSocketFile);
        snprintf(commandSocket, PATH_MAX, "unix:%s/cmd.sock", Modes.json_dir);
        serviceListen(commandService, Modes.net_bind_address, commandSocket, Modes.net_epfd);
        chmod(commandSocket, 0600);
    }

    raw_in = serviceInit(&Modes.services_in, "Raw TCP input", NULL, no_heartbeat, raw_heartbeat, READ_MODE_ASCII, "\n", processHexMessage);
    serviceListen(raw_in, Modes.net_bind_address, Modes.net_input_raw_ports, Modes.net_epfd);

    /* Beast input via network */
    Modes.beast_in_service = serviceInit(&Modes.services_in, "Beast TCP input", &Modes.beast_in, no_heartbeat, beast_heartbeat, READ_MODE_BEAST, NULL, decodeBinMessage);
    if (Modes.netIngest) {
        Modes.beast_in_service->sendqOverrideSize = 2 * 1024;
        Modes.beast_in_service->recvqOverrideSize = 4 * 1024;
        // --net-buffer won't increase receive buffer for ingest server to avoid running out of memory using lots of connections
    }
    serviceListen(Modes.beast_in_service, Modes.net_bind_address, Modes.net_input_beast_ports, Modes.net_epfd);

    /* Planefinder input via network */
    planefinder_in = serviceInit(&Modes.services_in, "Planefinder TCP input", NULL, no_heartbeat, no_heartbeat, READ_MODE_PLANEFINDER, NULL, decodePfMessage);
    serviceListen(planefinder_in, Modes.net_bind_address, Modes.net_input_planefinder_ports, Modes.net_epfd);

    Modes.uat_in_service = serviceInit(&Modes.services_in, "UAT TCP input", NULL, no_heartbeat, no_heartbeat, READ_MODE_ASCII, "\n", decodeUatMessage);
    serviceListen(Modes.uat_in_service, Modes.net_bind_address, Modes.net_input_uat_ports, Modes.net_epfd);

    for (int i = 0; i < Modes.net_connectors_count; i++) {
        struct net_connector *con = &Modes.net_connectors[i];
        if (strcmp(con->protocol, "beast_out") == 0)
            con->service = beast_out;
        else if (strcmp(con->protocol, "beast_in") == 0)
            con->service = Modes.beast_in_service;
        else if (strcmp(con->protocol, "beast_reduce_out") == 0)
            con->service = beast_reduce_out;
        else if (strcmp(con->protocol, "beast_reduce_plus_out") == 0) {
            con->service = beast_reduce_out;
            con->enable_uuid_ping = 1;
        } else if (strcmp(con->protocol, "raw_out") == 0)
            con->service = raw_out;
        else if (strcmp(con->protocol, "raw_in") == 0)
            con->service = raw_in;
        else if (strcmp(con->protocol, "planefinder_in") == 0)
            con->service = planefinder_in;
        else if (strcmp(con->protocol, "vrs_out") == 0)
            con->service = vrs_out;
        else if (strcmp(con->protocol, "json_out") == 0)
            con->service = json_out;
        else if (strcmp(con->protocol, "feedmap_out") == 0)
            con->service = feedmap_out;
        else if (strcmp(con->protocol, "sbs_out") == 0)
            con->service = sbs_out;
        else if (strcmp(con->protocol, "asterix_out") == 0)
            con->service = asterix_out;
        else if (strcmp(con->protocol, "asterix_in") == 0)
            con->service = asterix_in;
        else if (strcmp(con->protocol, "sbs_in") == 0)
            con->service = sbs_in;
        else if (strcmp(con->protocol, "sbs_in_mlat") == 0)
            con->service = sbs_in_mlat;
        else if (strcmp(con->protocol, "sbs_in_jaero") == 0)
            con->service = sbs_in_jaero;
        else if (strcmp(con->protocol, "sbs_in_prio") == 0)
            con->service = sbs_in_prio;
        else if (strcmp(con->protocol, "sbs_out_mlat") == 0)
            con->service = sbs_out_mlat;
        else if (strcmp(con->protocol, "sbs_out_jaero") == 0)
            con->service = sbs_out_jaero;
        else if (strcmp(con->protocol, "sbs_out_prio") == 0)
            con->service = sbs_out_prio;
        else if (strcmp(con->protocol, "sbs_out_replay") == 0)
            con->service = sbs_out_replay;
        else if (strcmp(con->protocol, "gpsd_in") == 0)
            con->service = gpsd_in;
        else if (strcmp(con->protocol, "uat_in") == 0)
            con->service = Modes.uat_in_service;
        else if (strcmp(con->protocol, "uat_replay_out") == 0)
            con->service = uat_replay_service;

    }

    if (Modes.dump_beast_dir) {
        int res = mkdir(Modes.dump_beast_dir, 0755);
        if (res != 0 && errno != EEXIST) {
            perror("issue creating dump-beast-dir");
        } else {
            Modes.dump_fw = createZstdFw(4 * 1024 * 1024);
            Modes.dump_beast_index = -1;
            dump_beast_check(mstime());
        }
    }
}


//
//=========================================================================
// Accept new connections
static void modesAcceptClients(struct client *c, int64_t now) {
    if (!c || !c->acceptSocket)
        return;

    int listen_fd = c->fd;
    struct net_service *s = c->service;

    struct sockaddr_storage storage;
    struct sockaddr *saddr = (struct sockaddr *) &storage;
    socklen_t slen = sizeof(storage);

    int fd;
    errno = 0;
    while ((fd = anetGenericAccept(Modes.aneterr, listen_fd, saddr, &slen, SOCK_NONBLOCK)) >= 0) {

        if (Modes.modesClientCount > Modes.max_fds_net) {
            // drop new modes clients if the count gets near resource limits
            anetCloseSocket(c->fd);
            static int64_t antiSpam;
            if (now > antiSpam) {
                antiSpam = now + 30 * SECONDS;
                fprintf(stderr, "<3> Can't accept new connection, limited to %d clients, consider increasing ulimit!\n", Modes.max_fds_net);
            }
        }

        c = createSocketClient(s, fd, NULL);
        if (s->unixSocket && c) {
            strcpy(c->host, s->unixSocket);
            fprintf(stderr, "%s: new c at %s\n", c->service->descr, s->unixSocket);
        } else if (c) {
            // We created the client, save the sockaddr info and 'hostport'
            getnameinfo(saddr, slen,
                    c->host, sizeof(c->host),
                    c->port, sizeof(c->port),
                    NI_NUMERICHOST | NI_NUMERICSERV);

            setProxyString(c);
            if (Modes.debug_net && (!Modes.netIngest || c->service->group == &Modes.services_out)) {
                fprintf(stderr, "%s: new c from %s port %s (fd %d)\n",
                        c->service->descr, c->host, c->port, fd);
            }
        } else {
            fprintf(stderr, "%s: Fatal: createSocketClient shouldn't fail!\n", s->descr);
            exit(1);
        }

        setSockopts(c);

        sendFiveHeartbeats(c, now);
    }

    if (errno != EMFILE && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf(stderr, "%s: Error accepting new connection: %s\n", s->descr, Modes.aneterr);
    }
}

//
//=========================================================================
//
// On error free the client, collect the structure, adjust maxfd if needed.
//
void modesCloseClient(struct client *c) {
    if (!c->service) {
        fprintf(stderr, "warning: double close of net client\n");
        return;
    }
    if (Modes.netIngest || Modes.netReceiverId || Modes.debug_no_discard) {
        double elapsed = (mstime() - c->connectedSince + 1) / 1000.0;
        double kbitpersecond = c->bytesReceived / 128.0 / elapsed;

        char uuid[64]; // needs 36 chars and null byte
        sprint_uuid(c->receiverId, c->receiverId2, uuid);
        fprintf(stderr, "disc: %s %6.1f s %6.2f kbit/s rId %s %s\n",
                (c->rtt > PING_DISCONNECT) ? "RTT  " : ((c->garbage >= GARBAGE_THRESHOLD) ? "garb." : "     "),
                elapsed, kbitpersecond,
                uuid, c->proxy_string);
    }

    epoll_ctl(Modes.net_epfd, EPOLL_CTL_DEL, c->fd, &c->epollEvent);
    if (c->serial) {
        if (close(c->fd) < 0) {
            fprintf(stderr, "Serial client close error: %s\n", strerror(errno));
        }
    } else {
        anetCloseSocket(c->fd);
    }
    c->service->connections--;
    Modes.modesClientCount--;
    if (c->service->writer) {
        c->service->writer->connections--;
    }
    struct net_connector *con = c->con;
    if (con) {
        int64_t now = mstime();
        // Clean this up and set the next_reconnect timer for another try.
        con->connecting = 0;
        con->connected = 0;
        con->c = NULL;

        int64_t sinceLastConnect = now - con->lastConnect;
        if (sinceLastConnect > Modes.net_connector_delay) {
            // reset backoff
            con->backoff = Modes.net_connector_delay_min;
        }

        // force fresh DNS lookup on disconnect if connected for more than 5 seconds
        if (sinceLastConnect > 1 * SECONDS) {
            con->try_addr = NULL;
        }

        con->next_reconnect = con->lastConnect + con->backoff;

        Modes.next_reconnect_callback = imin(Modes.next_reconnect_callback, con->next_reconnect);
        Modes.last_connector_fail = now;
    }

    // mark it as inactive and ready to be freed
    c->fd = -1;
    c->service = NULL;
    c->modeac_requested = 0;

    if (Modes.mode_ac_auto)
        autoset_modeac();
}

static void lockReceiverId(struct client *c) {
    if (c->receiverIdLocked)
        return;
    c->receiverIdLocked = 1;
    if (Modes.netIngest && Modes.debug_net && c->garbage < 50) {
        char uuid[64]; // needs 36 chars and null byte
        sprint_uuid(c->receiverId, c->receiverId2, uuid);
        fprintf(stderr, "new client:                        rId %s %s\n",
                uuid, c->proxy_string);
    }
}

void pingClient(struct client *c, uint32_t ping) {
    // truncate to 24 bit for clarity
    ping = ping & ((1 << 24) - 1);
    if (c->sendq_len + 8 >= c->sendq_max)
        return;
    char *p = c->sendq + c->sendq_len;

    *p++ = 0x1a;
    *p++ = 'P';
    *p++ = (uint8_t) (ping >> 16);
    if (*(p-1) == 0x1a)
        *p++ = 0x1a;
    *p++ = (uint8_t) (ping >> 8);
    if (*(p-1) == 0x1a)
        *p++ = 0x1a;
    *p++ = (uint8_t) (ping >> 0);
    if (*(p-1) == 0x1a)
        *p++ = 0x1a;

    c->sendq_len = p - c->sendq;
    if (0 && Modes.debug_ping)
        fprintf(stderr, "Sending Ping c: %d\n", ping);
}
static void pong(struct client *c, int64_t now) {
    if (now < c->pingReceived)
        fprintf(stderr, "WAT?! now < c->pingReceived\n");
    pingClient(c, c->ping + (now - c->pingReceived));
}
static void pingSenders(struct net_service *service, int64_t now) {
    if (!Modes.ping)
        return;
    uint32_t newPing = now & ((1 << 24) - 1);
    // only send a ping every 50th interval or every 5 seconds
    // the respoder will interpolate using the local clock
    if (newPing >= Modes.currentPing + 5000 || newPing < Modes.currentPing) {
        Modes.currentPing = newPing;
        if (Modes.debug_ping)
            fprintf(stderr, "Sending Ping: %d\n", newPing);
        for (struct client *c = service->clients; c; c = c->next) {
            if (!c->service)
                continue;
            // some devices can't deal with any data on the backchannel
            // for the time being only send to receivers that enable it on connect
            if (Modes.netIngest && !c->pingEnabled)
                continue;
            if (!c->pongReceived && now > c->connectedSince + 20 * SECONDS)
                continue;
            pingClient(c, newPing);
            if (flushClient(c, now) < 0) {
                continue;
            }
        }
    }
}
int pongReceived(struct client *c, int64_t now) {
    c->pongReceived = now;
    static int64_t antiSpam;

    // times in milliseconds


    int64_t pong = c->pong;
    int64_t current = now & ((1LL << 24) - 1);

    // handle 24 bit overflow by making the 2 numbers comparable
    if (labs((long) (current - pong)) > (1LL << 24) * 7 / 8) {
        if (current < pong) {
            current += (1 << 24);
        } else {
            pong += (1 << 24);
        }
    }

    if (current < pong) {
        // even without overflow, current can be smaller than pong due to
        // the other clock ticking up a ms just after receiving the ping (with sub ms latency)
        // but this clock ticked up just before sending the ping
        // other clock anomalies can make this happen as well,
        // even when debugging let's not log it unless the it's more than 3 ms
        if (Modes.debug_ping && pong - current > 3 && now > antiSpam) {
            antiSpam = now + 100;
            fprintf(stderr, "pongReceived strange: current < pong by %ld\n", (long) (pong - current));
        }
    }

    c->rtt = current - pong;

    if (c->rtt < 0) {
        c->rtt = 0;
    }

    int32_t bucket = 0;
    float bucketsize = PING_BUCKETBASE;
    float bucketmax = 0;
    for (int i = 0; i < PING_BUCKETS; i++) {
        bucketmax += bucketsize;
        bucketmax = nearbyint(bucketmax / 10) * 10;
        bucketsize *= PING_BUCKETMULT;

        bucket = i;

        if (c->rtt <= bucketmax) {
            break;
        }
    }
    Modes.stats_current.remote_ping_rtt[bucket]++;

    // more quickly arrive at a sensible average
    if (c->recent_rtt <= 0) {
        c->recent_rtt = c->rtt;
    } else if (c->bytesReceived < 5000) {
        c->recent_rtt = c->recent_rtt * 0.9 +  c->rtt * 0.1;
    } else {
        c->recent_rtt = c->recent_rtt * 0.995 +  c->rtt * 0.005;
    }
    if (c->latest_rtt <= 0) {
        c->latest_rtt = c->rtt;
    } else {
        c->latest_rtt = c->latest_rtt * 0.9 +  c->rtt * 0.1;
    }

    if (Modes.debug_ping && 0) {
        char uuid[64]; // needs 36 chars and null byte
        sprint_uuid(c->receiverId, c->receiverId2, uuid);
        fprintf(stderr, "rId %s %ld %4.0f %s current: %ld pong: %ld\n",
                uuid, (long) c->rtt, c->recent_rtt, c->proxy_string, (long) current, (long) pong);
    }

    // only log if the average is greater the rejection threshold, don't log for single packet events
    // actual discard / rejection happens elsewhere int the code
    if (c->rtt > Modes.ping_reject && now > antiSpam) {
        char uuid[64]; // needs 36 chars and null byte
        sprint_uuid(c->receiverId, c->receiverId2, uuid);
        if (Modes.debug_nextra) {
            antiSpam = now + 250; // limit to 4 messages a second
        } else {
            antiSpam = now + 30 * SECONDS;
        }
        if (Modes.netIngest) {
            fprintf(stderr, "reject: %6.0fms %6.0fms %6.0fms rId %s %s\n",
                    (double) c->rtt, c->latest_rtt, c->recent_rtt, uuid, c->proxy_string);
        } else {
            fprintf(stderr, "high network delay: %6lld ms; discarding data: %s rId %s\n",
                    (long long) c->rtt, c->proxy_string, uuid);
        }
    }
    if (Modes.netIngest && c->latest_rtt > Modes.ping_reduce) {
        // tell the client to slow down via beast command
        // misuse pingReceived as a timeout variable
        if (now > c->pingReceived + PING_REDUCE_DURATION / 2) {
            if (Modes.debug_nextra && now > antiSpam) {
                antiSpam = now + 250; // limit to 4 messages a second
                char uuid[64]; // needs 36 chars and null byte
                sprint_uuid(c->receiverId, c->receiverId2, uuid);
                fprintf(stderr, "reduce: %6.0f ms %6.0f ms  rId %s %s\n",
                        c->latest_rtt, c->recent_rtt, uuid, c->proxy_string);
            }
            if (c->sendq_len + 3 < c->sendq_max) {
                c->sendq[c->sendq_len++] = 0x1a;
                c->sendq[c->sendq_len++] = 'W';
                c->sendq[c->sendq_len++] = 'S';
                c->pingReceived = now;
            }
            if (flushClient(c, now) < 0) {
                return 1;
            }
        }
    }
    if (Modes.netIngest && c->rtt > PING_DISCONNECT) {
        return 1; // disconnect the client if the messages are delayed too much
    }
    return 0;
}

void dropHalfUntil(int64_t now, struct client *c, int64_t until) {

    if (
            now > c->dropHalfAntiSpam
            && (now - c->connectedSince > 10 * SECONDS || Modes.debug_net || Modes.debug_flush)
       ) {
        int suppress;
        if (Modes.debug_flush) {
            suppress = 2;
        } else if (Modes.debug_net) {
            suppress = 10;
        } else {
            suppress = 300;
        }
        c->dropHalfAntiSpam = now + suppress * SECONDS;
        double lostPercent = 100.0 - (double) c->bytesSent / (double) c->bytesFromWriter * 100.0;
        fprintf(stderr, "%s: %s port %s: Bad connection, dropping data"
                " (connection lost %4.1f%% of data) (suppress msg for %d s)\n",
                c->service->descr, c->host, c->port, lostPercent, suppress);
    }
    c->dropHalfUntil = until;
    c->dropHalfDrop = 1;
}

int flushClient(struct client *c, int64_t now) {
    if (!c->service) { fprintf(stderr, "report error: Ahlu8pie\n"); return -1; }
    int toWrite = c->sendq_len;

    if (toWrite == 0) {
        return 0;
    }

    int bytesWritten = send(c->fd, c->sendq, toWrite, 0);
    int err = errno;

    // If we get -1, it's only fatal if it's not EAGAIN/EWOULDBLOCK
    if (bytesWritten < 0) {
        if (err != EAGAIN && err != EWOULDBLOCK) {
            fprintf(stderr, "%s: Send Error: %s: %s port %s (fd %d, SendQ %d, RecvQ %d)\n",
                    c->service->descr, strerror(err), c->host, c->port,
                    c->fd, c->sendq_len, c->buflen);
            modesCloseClient(c);
            return -1;
        }
    }
    if (bytesWritten < toWrite) {
        dropHalfUntil(now, c, now + 2 * SECONDS);
    }
    if (bytesWritten > toWrite) {
        fprintf(stderr, "%s: send() weirdness: bytesWritten > toWrite: %s: %s port %s (fd %d, SendQ %d, RecvQ %d)\n",
                c->service->descr, strerror(err), c->host, c->port,
                c->fd, c->sendq_len, c->buflen);
        modesCloseClient(c);
        return -1;
    }
    if (0 && bytesWritten < toWrite && Modes.debug_flush) {
        fprintTimePrecise(stderr, now);
        fprintf(stderr, " %s: send wrote: %d/%d bytes (%s port %s fd %d, SendQ %d)\n", c->service->descr, bytesWritten, toWrite, c->host, c->port, c->fd, c->sendq_len);
    }
    if (bytesWritten > 0) {
        Modes.stats_current.network_bytes_out += bytesWritten;
        // Advance buffer
        toWrite -= bytesWritten;
        c->sendq_len -= bytesWritten;

        c->last_send = now;	// If we wrote anything, update this.
        if (toWrite > 0) {
            memmove((void*)c->sendq, c->sendq + bytesWritten, toWrite);
        }
    }
    if (toWrite > 0 && !(c->epollEvent.events & EPOLLOUT)) {
        // if we couldn't flush our buffer, make epoll tell us when we can write again
        c->epollEvent.events |= EPOLLOUT;
        if (epoll_ctl(Modes.net_epfd, EPOLL_CTL_MOD, c->fd, &c->epollEvent))
            perror("epoll_ctl fail:");
    }
    if (toWrite == 0 && (c->epollEvent.events & EPOLLOUT)) {
        // if set, remove EPOLLOUT from epoll if flush was successful
        c->epollEvent.events ^= EPOLLOUT;
        if (epoll_ctl(Modes.net_epfd, EPOLL_CTL_MOD, c->fd, &c->epollEvent))
            perror("epoll_ctl fail:");
    }

    // if we haven't been able to send any data on this connection for 2 seconds, drop it
    int64_t sendTimeout = 5 * SECONDS;
    if (now - c->last_send > sendTimeout && now - c->connectedSince > 15 * SECONDS) {
        fprintf(stderr, "%s: Couldn't send any data for %.2fs (Insufficient bandwidth?): disconnecting: %s port %s (fd %d, SendQ %d)\n", c->service->descr, sendTimeout / 1000.0, c->host, c->port, c->fd, c->sendq_len);
        modesCloseClient(c);
        return -1;
    }

    return bytesWritten;
}

//
//=========================================================================
//
// Send the write buffer for the specified writer to all connected clients
//
static void flushWrites(struct net_writer *writer) {
    if (writer->dataUsed == 0) {
        return;
    }
    int64_t now = mstime();
    if (0 && Modes.debug_flush) {
        fprintTimePrecise(stderr, now);
        fprintf(stderr, " %s: flushWrites %5d bytes\n", writer->service->descr, writer->dataUsed);
    }
    for (struct client *c = writer->service->clients; c; c = c->next) {
        if (!c->service)
            continue;
        if (c->service->writer == writer->service->writer) {
            if (c->pingEnabled) {
                pong(c, now);
            }

            if (writer->dataUsed > c->sendq_max) {
                fprintf(stderr, "%s: ERROR: dataUsed > sendq_max: report this bug!\n", c->service->descr);
                continue;
            }

            c->bytesFromWriter += writer->dataUsed;

            int bufferInsufficient = (c->sendq_len + writer->dataUsed > c->sendq_max);

            if (bufferInsufficient) {
                dropHalfUntil(now, c, now + 2 * SECONDS);
            }

            if ((c->dropHalfUntil > now && c->dropHalfDrop) || bufferInsufficient) {
                // drop this chunk of data
            } else {
                // Append the data to the end of the queue, increment len
                memcpy(c->sendq + c->sendq_len, writer->data, writer->dataUsed);
                c->sendq_len += writer->dataUsed;
                c->bytesSent += writer->dataUsed;

                if (0) {
                    double lostPercent = 100.0 - (double) c->bytesSent / (double) c->bytesFromWriter * 100.0;
                    fprintf(stderr, "%s: %s port %s: lost %4.1f%% of data\n",
                            c->service->descr, c->host, c->port, lostPercent);
                }
            }
            if (c->dropHalfUntil > now) {
                // we toggle the dropping each packet, so half the packets are dropped
                c->dropHalfDrop = !c->dropHalfDrop;
            }
            // Try flushing...
            if (flushClient(c, now) < 0) {
                continue;
            }
            if (!c->service) {
                continue;
            }
        }
    }
    writer->lastReceiverId = 0; // unconditionally emit receiver id on start of new "packet"
    writer->dataUsed = 0;
    writer->lastWrite = now;
    return;
}

// Prepare to write up to 'len' bytes to the given net_writer.
// Returns a pointer to write to, or NULL to skip this write.
void *prepareWrite(struct net_writer *writer, int len) {
    if (!writer->connections) {
        return NULL;
    }

    if (writer->dataUsed && writer->dataUsed + len > Modes.net_output_flush_size) {
        flushWrites(writer);
    }
    if (writer->dataUsed + len > Modes.writerBufSize) {
        // this shouldn't happen, flushWrites never fails and writerBufSize
        // must be larger than the output_flush_size
        fprintf(stderr, "%s: prepareWrite: not enough space in writer buffer, requested len: %d, already in buffer: %d\n", writer->service->descr, len, writer->dataUsed);
        return NULL;
    }
    //fprintf(stderr, "%s: requested len: %d, dataUsed: %d, bufSize: %d\n", writer->service->descr, len, writer->dataUsed, Modes.writerBufSize);

    return writer->data + writer->dataUsed;
}

// Complete a write previously begun by prepareWrite.
// endptr should point one byte past the last byte written
// to the buffer returned from prepareWrite.
void completeWrite(struct net_writer *writer, void *endptr) {
    if (writer->dataUsed == 0 && endptr - writer->data > 0) {
        int64_t now = mstime();
        if (0 && Modes.debug_flush) {
            fprintTimePrecise(stderr, now);
            fprintf(stderr, " completeWrite starting packet for %s\n", writer->service->descr);
        }
        writer->nextFlush = now + writer->flushInterval;
    }

    writer->dataUsed = endptr - writer->data;
    if (writer->dataUsed > Modes.writerBufSize) {
        fprintf(stderr, "%s: ERROR: overflow: dataUsed: %d, bufSize: %d\n", writer->service->descr, writer->dataUsed, Modes.writerBufSize);
    }

    if (writer->dataUsed >= Modes.net_output_flush_size) {
        flushWrites(writer);
    }
}

static void send_heartbeat(struct net_service *service) {
    if (!service->writer || !service->heartbeat_out.msg) {
        return;
    }

    char *p = prepareWrite(service->writer, service->heartbeat_out.len);
    if (!p) {
        return;
    }

    memcpy(p, service->heartbeat_out.msg, service->heartbeat_out.len);
    p += service->heartbeat_out.len;
    completeWrite(service->writer, p);
}

//
//=========================================================================
//
// Turn an hex digit into its 4 bit decimal value.
// Returns -1 if the digit is not in the 0-F range.
//
static void modesSendRawOutput(struct modesMessage *mm) {
    int msgLen = mm->msgbits / 8;
    char *p = prepareWrite(&Modes.raw_out, msgLen * 2 + 15);
    int j;
    unsigned char *msg = (Modes.net_verbatim ? mm->verbatim : mm->msg);

    if (!p)
        return;

    if (Modes.mlat && mm->timestamp) {
        /* timestamp, big-endian */
        sprintf(p, "@%012" PRIX64,
                mm->timestamp);
        p += 13;
    } else
        *p++ = '*';

    for (j = 0; j < msgLen; j++) {
        printHexDigit(p, msg[j]);
        p += 2;
    }

    *p++ = ';';
    *p++ = '\n';

    completeWrite(&Modes.raw_out, p);
}
void jsonPositionOutput(struct modesMessage *mm, struct aircraft *a) {
    MODES_NOTUSED(mm);

    int buflen = 8192;
    char buf[8192];

    char *p = buf;
    char *end = buf + buflen;

    p = sprintAircraftObject(p, end, a, mm->sysTimestamp, 2, NULL);

    if (p + 1 >= end) {
        fprintf(stderr, "buffer insufficient jsonPositionOutput()\n");
        return;
    }
    *p++ = '\n';

    int size = p - buf;

    char *w = prepareWrite(&Modes.json_out, size);
    if (!w) {
        return;
    }
    memcpy(w, buf, size);
    w += size;

    completeWrite(&Modes.json_out, w);
}

void sendData(struct net_writer *output, char *data, int len) {
    char *p;

    int buflen = Modes.writerBufSize;

    while (len > 0) {
        p = prepareWrite(output, buflen);
        if (!p)
            return;

        int tsize = imin(len, buflen);
        memcpy(p, data, tsize);
        len -= tsize;
        p += tsize;
        completeWrite(output, p);
    }
}

void autoset_modeac() {
    if (!Modes.mode_ac_auto)
        return;

    Modes.mode_ac = 0;
    for (struct net_service *service = Modes.services_out.services; service->descr; service++) {
        for (struct client *c = service->clients; c; c = c->next) {
            if (c->modeac_requested) {
                Modes.mode_ac = 1;
                break;
            }
        }
    }
}

// Send some Beast settings commands to a client
static int handleCommandSocket(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(c);
    MODES_NOTUSED(remote);
    MODES_NOTUSED(now);
    MODES_NOTUSED(mb);
    char *saveptr = NULL;
    char *cmd = strtok_r(p, " ", &saveptr);
    if (strcmp(cmd, "deleteTrace") == 0) {
        char *t1 = strtok_r(NULL, " ", &saveptr);
        char *t2 = strtok_r(NULL, " ", &saveptr);
        char *t3 = strtok_r(NULL, " ", &saveptr);
        if (!t1 || !t2 || !t3) {
            fprintf(stderr, "commandSocket deleteTrace: not enough tokens\n");
            return 0;
        }
        struct hexInterval* new = cmalloc(sizeof(struct hexInterval));
        new->hex = (uint32_t) strtol(t1, NULL, 16);
        new->from = (int64_t) strtol(t2, NULL, 10);
        new->to = (int64_t) strtol(t3, NULL, 10);
        new->next = Modes.deleteTrace;
        Modes.deleteTrace = new;
        fprintf(stderr, "Deleting %06x from %lld to %lld\n", new->hex, (long long) new->from, (long long) new->to);
    } else {
        fprintf(stderr, "commandSocket: unrecognized command\n");
    }
    return 0;
}
//
// Handle a Beast command message.
// Currently, we just look for the Mode A/C command message
// and ignore everything else.
//
int decodeHexMessage(struct client *c, char *hex, int64_t now, struct modesMessage *mm) {
    int l = strlen(hex), j;
    unsigned char *msg = mm->msg;

    mm->client = c;

    // Mark messages received over the internet as remote so that we don't try to
    // pass them off as being received by this instance when forwarding them
    mm->remote = 1;
    mm->signalLevel = 0;

    // Remove spaces on the left and on the right
    while (l && isspace(hex[l - 1])) {
        hex[l - 1] = '\0';
        l--;
    }
    while (isspace(*hex)) {
        hex++;
        l--;
    }

    // https://www.aerobits.pl/wp-content/uploads/2021/07/OEM_MC_Datasheet.pdf
    // *RAW_FRAME;(SIGS,SIGQ,TS)\r\n
    // signal strength mV, signal quality mV, time from last PPS pulse in hex (microseconds?, document doesn't say)
    // let's just create a bogus timestamp and parse the signal strength ....
    if (hex[l - 1] == ')') {
        hex[l - 1] = '\0';
        int pos = l - 1;
        while (pos && hex[pos] != '(') {
            pos--;
        } // find opening (
        if (pos) {
            hex[pos] = '\0';
            l = pos;
        } else {
            return 0;
        } // incomplete
        char *saveptr = NULL;
        char *token = strtok_r(&hex[pos + 1], ",", &saveptr);
        if (!token) return 0;
        mm->signalLevel = strtol(token, NULL, 10);
        mm->signalLevel /= 1000; // let's assume 1000 mV max .. i only have a small sample, specification isn't clear
        mm->signalLevel = mm->signalLevel * mm->signalLevel; // square it to get power
        mm->signalLevel = fmin(1.0, mm->signalLevel); // cap at 1
                                                    //
        token = strtok_r(NULL, ",", &saveptr); // discard signal quality
        if (!token) return 0;

        token = strtok_r(NULL, ",", &saveptr);
        if (token) {
            int after_pps = strtol(token, NULL, 16);
            // round down to current second, go to microseconds and add after_pps, go to 12 MHz clock
            int64_t seconds = now / 1000;
            if (after_pps / 1000 > (now % 1000) + 500) {
                seconds -= 1;
                // assume our clock is one second in front of the GPS clock, go back one second before adding the after pps time
            }
            mm->timestamp = (seconds * (1000 * 1000) + after_pps) * 12;
        } else {
            mm->timestamp = now * 12e3; // make 12 MHz timestamp from microseconds
        }
    }
    // Turn the message into binary.
    // Accept
    // *-AVR: raw
    // @-AVR: beast_ts+raw
    // %-AVR: timeS+raw (CRC good)
    // <-AVR: beast_ts+sigL+raw
    // and some AVR records that we can understand
    if (hex[l - 1] != ';') {
        return (0);
    } // not complete - abort

    if (l <= 2 * MODEAC_MSG_BYTES)
        return (0); // too short

    switch (hex[0]) {
        // <TTTTTTTTTTTTSS
        case '<':
            {
                // skip <
                hex++;
                l--;
                // skip ;
                l--;

                if (l < 12)
                    return (0);
                for (j = 0; j < 12; j++) {
                    mm->timestamp = (mm->timestamp << 4) | hexDigitVal(*hex);
                    hex++;
                    l--;
                }
                if (l < 2)
                    return (0);
                mm->signalLevel = ((hexDigitVal(hex[0]) << 4) | hexDigitVal(hex[1])) / 255.0;
                hex += 2;
                l -= 2;
                mm->signalLevel = mm->signalLevel * mm->signalLevel;
                break;
            }

        case '@': // No CRC check
                  // example timestamp 03BA2A7C1DD1, should be 12 MHz treat it as such
                  // example message: @03BA2A7C1DD15D4CA7F9A0B84B;
            { // CRC is OK
                hex++;
                l -= 2; // Skip @ and ;

                if (l <= 12) // if we have only enough hex for the timestamp or less it's invalid
                    return (0);
                for (j = 0; j < 12; j++) {
                    mm->timestamp = (mm->timestamp << 4) | hexDigitVal(*hex);
                    hex++;
                }

                l -= 12; // timestamp now processed
                break;
            }
        case '%':
            { // CRC is OK
                hex += 13;
                l -= 14; // Skip @,%, and timestamp, and ;
                break;
            }

        case '*':
        case ':':
            {
                hex++;
                l -= 2; // Skip * and ;
                break;
            }

        default:
            {
                return (0); // We don't know what this is, so abort
                break;
            }
    }

    if ((l != (MODEAC_MSG_BYTES * 2))
            && (l != (MODES_SHORT_MSG_BYTES * 2))
            && (l != (MODES_LONG_MSG_BYTES * 2))) {
        return (0);
    } // Too short or long message... broken

    if ((0 == Modes.mode_ac)
            && (l == (MODEAC_MSG_BYTES * 2))) {
        return (0);
    } // Right length for ModeA/C, but not enabled

    for (j = 0; j < l; j += 2) {
        int high = hexDigitVal(hex[j]);
        int low = hexDigitVal(hex[j + 1]);

        if (high == -1 || low == -1) return 0;
        msg[j / 2] = (high << 4) | low;
    }

    // record reception time as the time we read it.
    mm->sysTimestamp = now;

    if (l == (MODEAC_MSG_BYTES * 2)) { // ModeA or ModeC
        Modes.stats_current.remote_received_modeac++;
        decodeModeAMessage(mm, ((msg[0] << 8) | msg[1]));
    } else { // Assume ModeS
        int result;

        Modes.stats_current.remote_received_modes++;
        result = decodeModesMessage(mm);
        if (result < 0) {
            if (result == -1)
                Modes.stats_current.remote_rejected_unknown_icao++;
            else
                Modes.stats_current.remote_rejected_bad++;
            return 0;
        } else {
            Modes.stats_current.remote_accepted[mm->correctedbits]++;
        }
    }

    return 1;
}

//
//
//=========================================================================
//
// This function decodes a string representing message in raw hex format
// like: *8D4B969699155600E87406F5B69F; The string is null-terminated.
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
//

static int processHexMessage(struct client *c, char *hex, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);

    struct modesMessage *mm = netGetMM(mb);

    int success = decodeHexMessage(c, hex, now, mm);

    if (success) {
        netUseMessage(mm);
    }

    return (0);
}

const char *hexDumpString(const char *str, int strlen, char *buf, int buflen) {
    int max = buflen / 4 - 4;
    if (max <= 0) {
        // fail silently
        buf[0] = 0;
        return buf;
    }

    char *out = buf;
    char *end = buf + buflen;

    for (int k = 0; k < max && k < strlen; k++) {
        out = safe_snprintf(out, end, "%02x ", (unsigned char) str[k]);
    }

    out = safe_snprintf(out, end, "|");

    for (int k = 0; k < max && k < strlen; k++) {
        unsigned char ch = str[k];
        if (ch < 32 || ch > 126) {
            out = safe_snprintf(out, end, ".");
        } else {
            out = safe_snprintf(out, end, "%c", (unsigned char) ch);
        }
    }

    out = safe_snprintf(out, end, "|");

    if (out >= end) {
        fprintf(stderr, "hexDumpstring: check logic\n");
    }

    return buf;
}

void garbageIncrement(struct client *c, int add, int line) {
    if (0) {
        fprintf(stderr, "garbageIncrement: %4d line: %4d\n", add, line);
    }
    c->garbage += add;
}


//
//=========================================================================
//
// This function polls the clients using read() in order to receive new
// messages from the net.
//
static int readClient(struct client *c, int64_t now) {
    int nread = 0;
    if (c->discard)
        c->buflen = 0;

    int left = c->bufmax - c->buflen - 4; // leave 4 extra byte for NUL termination in the ASCII case


    // If our buffer is full discard it, this is some badly formatted shit
    if (left <= 0) {
        garbageIncrement(c, c->buflen, __LINE__);
        Modes.stats_current.remote_malformed_beast += c->buflen;

        c->buflen = 0;
        c->som = c->buf;
        c->eod = c->buf + c->buflen;

        left = c->bufmax - c->buflen - 4; // leave 4 extra byte for NUL termination in the ASCII case
                                          // If there is garbage, read more to discard it ASAP
    }

    if (c->remote) {
        nread = recv(c->fd, c->buf + c->buflen, left, 0);
    } else {
        // read instead of recv for modesbeast / gns-hulc ....
        if (0 && Modes.debug_serial) {
            fprintTimePrecise(stderr, mstime());
            fprintf(stderr, " serial read ... fd: %d maxbytes: %d\n", c->fd, left);
        }
        nread = read(c->fd, c->buf + c->buflen, left);
        if (nread > 0 && Modes.debug_serial) {
            fprintTimePrecise(stderr, mstime());
            fprintf(stderr, " serial read return value: %d\n", nread);
        }
        static int64_t lastData;
        if (nread > 0 || !lastData) {
            lastData = now;
        }
        if (now - lastData > 5 * MINUTES) {
            fprintTimePrecise(stderr, mstime());
            fprintf(stderr, " no data from device for 5 minutes\n");
            lastData = now; // reset this so we don't keep printing it
        }
    }
    int err = errno;

    // If we didn't get all the data we asked for, then return once we've processed what we did get.
    if (nread != left) {
        c->bContinue = 0;

        // also note that we (likely) emptied the system network buffer
        c->last_read_flush = now;
    }

    if (nread < 0) {
        if (err == EAGAIN || err == EWOULDBLOCK) {
            // No data available, check later!
            return 0;
        }
        // Other errors
        if (c->serial) {
            fprintf(stderr, "Serial client read error: %s\n", strerror(err));
        }
        if (Modes.debug_net) {
            fprintf(stderr, "%s: Socket Error: %s: %s port %s (fd %d, SendQ %d, RecvQ %d)\n",
                    c->service->descr, strerror(err), c->host, c->port,
                    c->fd, c->sendq_len, c->buflen);
        }
        modesCloseClient(c);
        return 0;
    }

    // End of file
    if (nread == 0) {
        if (c->serial) {
            // for serial this just means we're doing non-blocking reads and there are no bytes available
            return 0;
        }

        if (c->con) {
            if (Modes.synthetic_now) {
                Modes.synthetic_now = 0;
            }
            fprintf(stderr, "%s: Remote server disconnected: %s port %s (fd %d, SendQ %d, RecvQ %d)\n",
                    c->service->descr, c->con->address, c->con->port, c->fd, c->sendq_len, c->buflen);
        } else if (Modes.debug_net && !Modes.netIngest) {
            fprintf(stderr, "%s: Listen client disconnected: %s port %s (fd %d, SendQ %d, RecvQ %d)\n",
                    c->service->descr, c->host, c->port, c->fd, c->sendq_len, c->buflen);
        }
        if (!c->con && Modes.debug_bogus) {
            setExit(1);
        }
        modesCloseClient(c);
        return 0;
    }

    // nread > 0 here
    Modes.stats_current.network_bytes_in += nread;

    if (Modes.netIngest) {
        int windowSeconds = 2;
        int aboveRate = (c->recentMessages > windowSeconds * Modes.ingestLimitRate) || (c->recentPositions > windowSeconds * Modes.ingestLimitPositionRate);
        if (aboveRate) {
            c->unreasonable_messagerate = 1;
            if (now > c->unreasonableRateReset) {
                char uuid[64]; // needs 36 chars and null byte
                sprint_uuid(c->receiverId, c->receiverId2, uuid);
                fprintf(stderr, "GARBAGE due to high message rate %ld > %d rId %s %s\n", (long int) (c->recentMessages / windowSeconds), Modes.ingestLimitRate, uuid, c->proxy_string);
            }
            c->unreasonableRateReset = now + 20 * SECONDS;
        }

        if (now > c->recentMessagesReset) {
            if (0) {
                char uuid[64]; // needs 36 chars and null byte
                sprint_uuid(c->receiverId, c->receiverId2, uuid);
                fprintf(stderr, "message rate %ld rId %s %s\n", (long int) (c->recentMessages / windowSeconds), uuid, c->proxy_string);
            }
            if (!aboveRate && now > c->unreasonableRateReset) {
                c->unreasonable_messagerate = 0;
            }
            c->recentMessagesReset = now + windowSeconds * SECONDS;
            c->recentMessages = 0;
            c->recentPositions = 0;
        }
    }

    if (!Modes.debug_no_discard && !c->discard && now - c->last_read < 800 && now - c->last_read_flush > 2400 && !Modes.synthetic_now) {
        c->discard = 1;
        if (Modes.netIngest && c->proxy_string[0] != '\0') {
            fprintf(stderr, "<3>ERROR, not enough CPU: Discarding data from: %s\n", c->proxy_string);
        } else {
            fprintf(stderr, "<3>%s: ERROR, not enough CPU: Discarding data from: %s port %s (fd %d)\n",
                    c->service->descr, c->host, c->port, c->fd);
        }
    }

    c->last_read = now;

    if (c->discard) {
        return nread;
    }

    c->buflen += nread;
    c->bytesReceived += nread;

    return nread;
}

static int readAscii(struct client *c, int64_t now, struct messageBuffer *mb) {
    //
    // This is the ASCII scanning case, AVR RAW or HTTP at present
    // If there is a complete message still in the buffer, there must be the separator 'sep'
    // in the buffer, note that we full-scan the buffer at every read for simplicity.
    //
    char *p;

    // replace null bytes with newlines so the routine doesn't stop working
    // while null bytes are an illegal input, let's deal with them regardless
    if (memchr(c->som, '\0', c->eod - c->som)) {
        p = c->som;
        while (p < c->eod) {
            if (*p == '\0') {
                *p = '\n';
            }
            p++;
        }
        // warn about illegal input
        static int64_t antiSpam;
        if (Modes.debug_garbage && now > antiSpam) {
            antiSpam = now + 30 * SECONDS;
            fprintf(stderr, "%s from %s port %s: Bad format, at least one null byte in input data!\n", c->service->descr, c->host, c->port);
        }
    }
    while (c->som < c->eod && (p = strstr(c->som, c->service->read_sep)) != NULL) { // end of first message if found
        *p = '\0'; // The handler expects null terminated strings
                   // remove \r for strings that still have it at the end
        if (p - 1 > c->som && *(p - 1) == '\r') {
            *(p - 1) = '\0';
        }
        char *start = c->som;
        c->som = p + c->service->read_sep_len; // Move to start of next message
        if (c->service->read_handler(c, start, c->remote, now, mb)) { // Pass message to handler.
            if (Modes.debug_net) {
                fprintf(stderr, "%s: Closing connection from %s port %s\n", c->service->descr, c->host, c->port);
            }
            modesCloseClient(c); // Handler returns 1 on error to signal we .
            return -1; // should close the client connection
        }
    }
    return 0;
}

static int readProxy(struct client *c) {
    char *proxy = strstr(c->som, "PROXY ");
    char *eop = strstr(c->som, "\r\n");
    if (proxy && proxy == c->som) {
        if (!eop) {
            // incomplete proxy string (shouldn't happen but let's check anyhow)
            return -2;
        }
        *eop = '\0';
        strncpy(c->proxy_string, proxy + 6, sizeof(c->proxy_string) - 1);
        c->proxy_string[sizeof(c->proxy_string) - 1] = '\0'; // make sure it's null terminated
                                                             //fprintf(stderr, "%s\n", c->proxy_string);
        *eop = '\r';

        // expected string example: "PROXY TCP4 172.12.2.132 172.191.123.45 40223 30005"

        char *space = proxy;
        for (int i = 0; i < 3; i++) {
            if (!space) {
                break; // check to avoid null deref
            }
            space = memchr(space + 1, ' ', eop - space - 1);
        }
        if (!space) {
            // incomplete proxy string
            return -2;
        }
        // hash up to 3rd space
        if (eop - proxy > 10) {
            //fprintf(stderr, "%ld %ld %s\n", eop - proxy, space - proxy, space);
            c->receiverId = fasthash64(proxy, space - proxy, 0x2127599bf4325c37ULL);
        }

        c->som = eop + 2;
    }
    return 0;
}

void requestCompression(struct client *c, int64_t now) {
    //memcpy(c->sendq + c->sendq_len, heartbeat_msg, heartbeat_len);
    //c->sendq_len += heartbeat_len;
    flushClient(c, now);
}

//
//=========================================================================
//
// The message is supposed to be separated from the next message by the
// separator 'sep', which is a null-terminated C string.
//
// Every full message received is decoded and passed to the higher layers
// calling the function's 'handler'.
//
// The handler returns 0 on success, or 1 to signal this function we should
// close the connection with the client in case of non-recoverable errors.
//
//
//
//

static int processClient(struct client *c, int64_t now, struct messageBuffer *mb) {

    struct net_service *service = c->service;
    read_mode_t read_mode = service->read_mode;

    //fprintf(stderr, "processing count %d, buf->id %d\n", c->processing, mb->id);

    if (now - c->connectedSince < 5 * SECONDS) {
        // check for PROXY v1 header if connection is new / low bytes received
        if ((Modes.netIngest || Modes.readProxy) && c->som == c->buf) {
            if (c->eod - c->som >= 6 && c->som[0] == 'P' && c->som[1] == 'R') {
                int res = readProxy(c);
                if (res != 0) {
                    return res;
                }
            }
        }
        const char *hb = service->heartbeat_in.msg;
        int hb_len = service->heartbeat_in.len;
        if (hb && c->eod - c->som >= 5 * hb_len) {
            int res = 0;
            for (int k = 0; k < 5; k++) {
                res += memcmp(hb, c->som + k * hb_len, hb_len);
            }
            if (res == 0) {
                requestCompression(c, now);
            }
        }
    }

    if (read_mode == READ_MODE_BEAST) {
        int res = readBeast(c, now, mb);
        if (res != 0) {
            return res;
        }

    } else if (read_mode == READ_MODE_ASCII) {
        int res = readAscii(c, now, mb);
        if (res != 0) {
            return res;
        }

    } else if (read_mode == READ_MODE_IGNORE) {
        // drop the bytes on the floor
        c->som = c->eod;

    } else if (read_mode == READ_MODE_BEAST_COMMAND) {
        int res = readBeastcommand(c, now, mb);
        if (res != 0) {
            return res;
        }
    } else if (read_mode == READ_MODE_ASTERIX) {
        int res = readAsterix(c, now, mb);
        if (res != 0) {
            return res;
	    }
    } else if (read_mode == READ_MODE_PLANEFINDER) {
        int res = readPlanefinder(c, now, mb);
        if (res != 0) {
            return res;
        }
    }

    if (!c->receiverIdLocked && (c->bytesReceived > 512 || now > c->connectedSince + 10000)) {
        lockReceiverId(c);
    }

    return 0;
}
static void modesReadFromClient(struct client *c, struct messageBuffer *mb) {
    if (!c->service) {
        fprintf(stderr, "c->service null jahFuN3e\n");
        return;
    }

    if (!c->bufferToProcess) {
        c->bContinue = 1;
    }
    for (int k = 0; c->bContinue; k++) {
        int64_t now = mstime();

        // guarantee at least one read before obeying the network time limit
        if (k > 0 && now > Modes.network_time_limit) {
            return;
        }

        if (Modes.synthetic_now && priorityTasksPending()) {
            return;
        }

        if (!c->service) {
            return;
        }

        if (!c->bufferToProcess) {
            // get more buffer to process
            int read = readClient(c, now);
            //fprintTimePrecise(stderr, now); fprintf(stderr, "readClient returned: %d\n", read);
            if (!read) {
                return;
            }
            if (c->discard) {
                continue;
            }
            c->som = c->buf;
            c->eod = c->buf + c->buflen; // one byte past end of data
            // Always NUL-terminate so we are free to use strstr()
            // nb: we never fill the last byte of the buffer with read data (see above) so this is safe
            if (likely(c->buflen < c->bufmax)) {
                *c->eod = '\0';
            } else {
                fprintf(stderr, "wtf Dieh2hau\n");
            }
            c->bufferToProcess = 1;

            mb->activeClient = c;
        }

        // process buffer

        c->processing++;
        //fprintf(stderr, "%d", c->processing);
        int res = processClient(c, now, mb);
        c->processing--;
        c->bufferToProcess = 0;

        mb->activeClient = NULL;

        if (res < 0) {
            return;
        }

        if (c->som > c->buf) { // We processed something - so
            c->buflen = c->eod - c->som; //     Update the unprocessed buffer length
            if (c->buflen > 0) {
                memmove(c->buf, c->som, c->buflen); //     Move what's remaining to the start of the buffer
            } else {
                if (c->buflen < 0) {
                    c->buflen = 0;
                    fprintf(stderr, "codepoint Si0wereH\n");
                }
            }
            c->som = c->buf;
            c->eod = c->buf + c->buflen; // one byte past end of data
                                         // Always NUL-terminate so we are free to use strstr()
                                         // nb: we never fill the last byte of the buffer with read data (see above) so this is safe
        }

        // disconnect garbage feeds
        // (only disconnect after a lot of garbage unless --net-ingest is specified)
        if ((c->garbage >= GARBAGE_THRESHOLD && Modes.netIngest) || c->garbage >= 1024 * 1024) {

            *c->eod = '\0';
            char sample[256];
            hexDumpString(c->som, c->eod - c->som, sample, sizeof(sample));
            sample[sizeof(sample) - 1] = '\0';
            if (c->proxy_string[0] != '\0') {
                fprintf(stderr, "Garbage: Close: %s sample: %s\n", c->proxy_string, sample);
            } else {
                fprintf(stderr, "Garbage: Close: %s port %s sample: %s\n", c->host, c->port, sample);
            }

            modesCloseClient(c);
            return;
        }
    }

    // reset discard status
    c->discard = 0;
}

/*
static inline unsigned unsigned_difference(unsigned v1, unsigned v2) {
    return (v1 > v2) ? (v1 - v2) : (v2 - v1);
}

static inline float heading_difference(float h1, float h2) {
    float d = fabs(h1 - h2);
    return (d < 180) ? d : (360 - d);
}
*/

const char *airground_enum_string(airground_t ag) {
    switch (ag) {
        case AG_AIRBORNE:
            return "A+";
        case AG_GROUND:
            return "G+";
        default:
            return "?";
    }
}

static void serviceFreeClients(struct net_service *s) {
    struct client *c, **prev;
    for (prev = &s->clients, c = *prev; c; c = *prev) {
        if (c->fd == -1) {
            // Recently closed, prune from list
            *prev = c->next;
            sfree(c->sendq);
            sfree(c->buf);
            sfree(c);
        } else {
            prev = &c->next;
        }
    }
}

// Unlink and free closed clients
static void netFreeClients() {
    for (struct net_service *service = Modes.services_out.services; service->descr; service++) {
        serviceFreeClients(service);
    }
    for (struct net_service *service = Modes.services_in.services; service->descr; service++) {
        serviceFreeClients(service);
    }
}

static void handleEpoll(struct net_service_group *group, struct messageBuffer *mb) {
    // Only process each epoll even in one thread
    // the variables for this are specific to the service group,
    // using locking each group can only be processed by one thread at a time
    // modesReadFromClient can unlock this lock which is fine as the while head
    // is lock protected
    while (group->event_progress < Modes.net_event_count) {
        int k = group->event_progress;
        group->event_progress += 1;

        struct epoll_event event = Modes.net_events[k];
        if (event.data.ptr == &Modes.exitNowEventfd) {
            return;
        }

        struct client *cl = (struct client *) Modes.net_events[k].data.ptr;
        if (!cl) { fprintf(stderr, "handleEpoll: epollEvent.data.ptr == NULL\n"); continue; }

        if (!cl->service) {
            // client is closed
            //fprintf(stderr, "handleEpoll(): client closed\n");
            continue;
        }

        if (cl->service->group != group) {
            //fprintf(stderr, "handleEpoll(): wrong group\n");
            continue;
        }

        if (cl->acceptSocket || cl->net_connector_dummyClient) {
            if (cl->acceptSocket) {
                modesAcceptClients(cl, mstime());
            }
            if (cl->net_connector_dummyClient) {
                checkServiceConnected(cl->con, mstime());
            }
        } else {
            if ((event.events & EPOLLOUT)) {
                // check if we need to flush a client because the send buffer was full previously
                if (flushClient(cl, mstime()) < 0) {
                    continue;
                }
            }

            if ((event.events & (EPOLLIN | EPOLLRDHUP | EPOLLERR | EPOLLHUP))) {
                modesReadFromClient(cl, mb);
            }
        }
    }
}

static int64_t checkFlushService(struct net_service *service, int64_t now) {
    int64_t default_wait = 1000;
    if (!service->writer) {
        return now + default_wait;
    }
    struct net_writer *writer = service->writer;
    if (Modes.net_heartbeat_interval && service->heartbeat_out.msg
            && now - writer->lastWrite >= Modes.net_heartbeat_interval) {
        // If we have generated no messages for a while, send a heartbeat
        send_heartbeat(service);
    }
    if (writer->dataUsed && now >= writer->nextFlush) {
        flushWrites(writer);
    }
    if (writer->dataUsed) {
        return writer->nextFlush;
    } else {
        return now + default_wait;
    }
}

static void decodeTask(void *arg, threadpool_threadbuffers_t *buffer_group) {
    MODES_NOTUSED(buffer_group);

    readsb_task_t *info = (readsb_task_t *) arg;
    struct messageBuffer *mb = &Modes.netMessageBuffer[info->from];

    //fprintf(stderr, "%.3f decodeTask %d\n", mstime()/1000.0, mb->id);

    pthread_mutex_lock(&Modes.decodeLock);
    //fprintf(stderr, "%.3f decoding %d\n", mstime()/1000.0, mb->id);

    handleEpoll(&Modes.services_in, mb);

    for (int kt = 0; kt < Modes.decodeThreads; kt++) {
        struct messageBuffer *otherbuf = &Modes.netMessageBuffer[kt];
        struct client *cl = otherbuf->activeClient;
        if (cl && cl->service) {
            modesReadFromClient(cl, mb);
        }
    }
    drainMessageBuffer(mb);

    pthread_mutex_unlock(&Modes.decodeLock);

    pthread_mutex_lock(&Modes.outputLock);
    handleEpoll(&Modes.services_out, mb);
    pthread_mutex_unlock(&Modes.outputLock);
}

//
// Perform periodic network work
//
void modesNetPeriodicWork(void) {
    static int64_t check_flush;
    static int64_t next_tcp_json;
    static struct timespec watch;

    if (!Modes.net_events) {
        epollAllocEvents(&Modes.net_events, &Modes.net_maxEvents);
    }

    int64_t now = mstime();

    dump_beast_check(now);

    int64_t wait_ms;
    if (SdrConfig.sdr_type != SDR_NONE && SdrConfig.sdr_type != SDR_MODESBEAST && SdrConfig.sdr_type != SDR_GNS) {
        // NO WAIT WHEN USING AN SDR !! IMPORTANT !!
        wait_ms = 0;
    } else {
        // wait in net-only mode (unless we get network packets, that wakes the wait immediately)
        wait_ms = imax(0, check_flush - now); // modify wait for next flush timer
        wait_ms = imin(wait_ms, Modes.next_reconnect_callback - now); // modify wait for reconnect callback timer
        wait_ms = imax(wait_ms, 0); // don't allow negative values
        if (0 && Modes.debug_serial) {
            wait_ms = 1000;
        }
    }
    #ifdef NO_EVENT_FD
    wait_ms = imin(wait_ms, 100); // no event_fd, limit sleep to 100 ms
    #endif

    // unlock decode mutex for waiting in handleEpoll
    pthread_mutex_unlock(&Threads.decode.mutex);

    if (priorityTasksPending()) {
        sched_yield();
    }
    Modes.net_event_count = epoll_wait(Modes.net_epfd, Modes.net_events, Modes.net_maxEvents, (int) wait_ms);
    Modes.services_in.event_progress = 0;
    Modes.services_out.event_progress = 0;

    if (0 && Modes.debug_serial && Modes.net_event_count > 0) {
        fprintTimePrecise(stderr, mstime()); fprintf(stderr, " event count %d wait_ms %d\n", Modes.net_event_count, (int) wait_ms);
    }

    if (0 && Modes.net_event_count > 0) {
        fprintTimePrecise(stderr, now); fprintf(stderr, " event count %d wait_ms %d\n", Modes.net_event_count, (int) wait_ms);
    }

    pthread_mutex_lock(&Threads.decode.mutex);

    int64_t interval = lapWatch(&watch);

    now = mstime();
    Modes.network_time_limit = now + 100;

    struct messageBuffer *mb = &Modes.netMessageBuffer[0];
    if (Modes.decodeThreads == 1) {
        handleEpoll(&Modes.services_in, mb);
        drainMessageBuffer(mb);
        handleEpoll(&Modes.services_out, mb);
    } else {
        readsb_task_t *infos = Modes.decodeTasks->infos;
        threadpool_task_t *tasks = Modes.decodeTasks->tasks;
        int taskCount = 0;

        for (int kt = 0; kt < Modes.decodeThreads; kt++) {
            threadpool_task_t *task = &tasks[kt];
            readsb_task_t *range = &infos[kt];

            range->from = kt;

            task->function = decodeTask;
            task->argument = range;
            taskCount++;
        }

        struct timespec before = threadpool_get_cumulative_thread_time(Modes.decodePool);
        threadpool_run(Modes.decodePool, tasks, taskCount);
        struct timespec after = threadpool_get_cumulative_thread_time(Modes.decodePool);
        timespec_add_elapsed(&before, &after, &Modes.stats_current.background_cpu);
    }

    /* Beast input from local Modes-S Beast via USB */
    if (SdrConfig.sdrInitialized && (SdrConfig.sdr_type == SDR_MODESBEAST || SdrConfig.sdr_type == SDR_GNS)) {
        if (!Modes.serial_client) {
            if (1 || Modes.debug_serial) {
                fprintTimePrecise(stderr, mstime());
                fprintf(stderr, " serial: creating socket client ... \n");
            }

            Modes.serial_client = createSocketClient(Modes.beast_in_service, Modes.beast_fd, NULL);
            if (1 || Modes.debug_serial) {
                fprintTimePrecise(stderr, mstime());
                fprintf(stderr, " serial: creating socket client ... done\n");
            }
        }
        if (Modes.serial_client->service) {
            modesReadFromClient(Modes.serial_client, mb);
            drainMessageBuffer(mb);
        }
        if (!Modes.serial_client->service) {
            fprintf(stderr, "Serial client closed unexpectedly, exiting!\n");
            setExit(2);
        }
    }

    if (Modes.net_event_count == Modes.net_maxEvents) {
        epollAllocEvents(&Modes.net_events, &Modes.net_maxEvents);
    }

    int64_t elapsed1 = lapWatch(&watch);

    now = mstime();

    pingSenders(Modes.beast_in_service, now);

    int64_t elapsed2 = lapWatch(&watch);

    // If we have data that has been waiting to be written for a while, write it now.
    if (SdrConfig.sdr_type != SDR_NONE || now >= check_flush || Modes.net_event_count > 0) {
        //fprintTimePrecise(stderr, now); fprintf(stderr, " checkFlush\n");

        check_flush = now + 200;

        for (struct net_service *service = Modes.services_out.services; service->descr; service++) {
            int64_t nextFlush = checkFlushService(service, now);
            check_flush = imin(check_flush, nextFlush);
        }
        for (struct net_service *service = Modes.services_in.services; service->descr; service++) {
            int64_t nextFlush = checkFlushService(service, now);
            check_flush = imin(check_flush, nextFlush);
        }
    }

    if (now >= Modes.next_reconnect_callback) {
        //fprintTimePrecise(stderr, now); fprintf(stderr, " reconnectCallback\n");

        int64_t since_fail = now - Modes.last_connector_fail;
        if (since_fail < 2 * SECONDS) {
            Modes.next_reconnect_callback = now + 20 + since_fail * Modes.net_connector_delay_min / ( 3 * SECONDS );
        } else {
            Modes.next_reconnect_callback = now + Modes.net_connector_delay_min;
        }
        serviceReconnectCallback(now);
    }

    static int64_t next_free_clients;
    int64_t free_client_interval = 1 * SECONDS;
    if (now > next_free_clients) {
        next_free_clients = now + free_client_interval;


        netFreeClients();

        if (Modes.receiverTable) {
            static uint32_t upcount;
            int nParts = RECEIVER_MAINTENANCE_INTERVAL / free_client_interval;
            receiverTimeout((upcount % nParts), nParts, now);
            upcount++;

            if (Modes.receiverCount > Modes.receiver_table_size * 3 / 4 && Modes.receiver_table_hash_bits < 16) {
                // we clear the table, rehashing would be nicer but more complex
                // this should be fine
                receiverCleanup();
                Modes.receiver_table_hash_bits = 16;
                receiverInit();
                fprintf(stderr, "receiverTable table size increased to: %d\n", Modes.receiver_table_size);
            }
        }
    }

    int64_t elapsed3 = lapWatch(&watch);

    static int64_t antiSpam;
    if ((elapsed1 > 2 * SECONDS || elapsed2 > 150 || elapsed3 > 150 || interval > 1 * SECONDS + Modes.net_output_flush_interval) && now > antiSpam + 5 * SECONDS) {
        antiSpam = now;
        fprintf(stderr, "<3>High load: modesNetPeriodicWork() elapsed1/2/3/interval %"PRId64"/%"PRId64"/%"PRId64"/%"PRId64" ms, suppressing for 5 seconds!\n",
                elapsed1, elapsed2, elapsed3, interval);
    }

    // supply JSON to vrs_out writer
    if (Modes.vrs_out.service && Modes.vrs_out.service->connections && now >= next_tcp_json) {
        static uint32_t part;
        static uint32_t count;
        uint32_t n_parts = 16; // must be 16 :)

        next_tcp_json = now + Modes.net_output_vrs_interval / n_parts;

        writeJsonToNet(&Modes.vrs_out, generateVRS(part, n_parts, (count % n_parts / 2 != part % 8)));
        if (++part == n_parts) {
            part = 0;
            count += 2;
        }
    }
}

void writeJsonToNet(struct net_writer *writer, struct char_buffer cb) {
    int len = cb.len;
    int written = 0;
    char *content = cb.buffer;
    char *pos;
    int bytes = Modes.writerBufSize;

    char *p = prepareWrite(writer, bytes);
    if (!p) {
        sfree(content);
        return;
    }

    pos = content;

    while (p && written < len) {
        if (bytes > len - written) {
            bytes = len - written;
        }
        memcpy(p, pos, bytes);
        p += bytes;
        pos += bytes;
        written += bytes;
        completeWrite(writer, p);

        p = prepareWrite(writer, bytes);
    }

    flushWrites(writer);
    sfree(content);
}


//
// =============================== Network IO ===========================
//

static void *pthreadGetaddrinfo(void *param) {
    struct net_connector *con = (struct net_connector *) param;

    struct addrinfo gai_hints;

    // no flags needed
    gai_hints.ai_flags = 0;
    // return both IPv4 and IPv6 addresses
    gai_hints.ai_family = AF_UNSPEC;

    // this is for protocol port lookups not for DNS, set to zero
    gai_hints.ai_protocol = 0;
    // setting this to zero returns the same address multiple times
    // just set it to TCP, should work just as well for UDP though
    gai_hints.ai_socktype = SOCK_STREAM;

    // these struct members don't seem to apply to the hints passed
    // they are used to return results but not in the hints
    gai_hints.ai_addrlen = 0;
    gai_hints.ai_addr = NULL;
    gai_hints.ai_canonname = NULL;
    gai_hints.ai_next = NULL;

    if (con->use_addr && con->address1) {
        con->address = con->address1;
        if (con->port1)
            con->port = con->port1;
        con->use_addr = 0;
    } else {
        con->address = con->address0;
        con->port = con->port0;
        con->use_addr = 1;
    }
    con->gai_error = getaddrinfo(con->address, con->port, &gai_hints, &con->addr_info);

    pthread_mutex_lock(&con->mutex);
    con->gai_request_done = 1;
    pthread_mutex_unlock(&con->mutex);
    return NULL;
}

static void cleanupService(struct net_service *s) {
    //fprintf(stderr, "cleanupService %s\n", s->descr);

    struct client *c = s->clients, *nc;
    while (c) {
        nc = c->next;

        anetCloseSocket(c->fd);
        c->sendq_len = 0;
        sfree(c->sendq);
        sfree(c->buf);
        sfree(c);

        c = nc;
    }

    if (s->listenSockets) {
        for (int i = 0; i < s->listener_count; ++i) {
            struct client *c = &s->listenSockets[i]; // not really a client
            epoll_ctl(Modes.net_epfd, EPOLL_CTL_DEL, c->fd, &c->epollEvent);
            anetCloseSocket(s->listener_fds[i]);
        }
        sfree(s->listenSockets);
    }
    sfree(s->listener_fds);
    if (s->writer && s->writer->data) {
        sfree(s->writer->data);
    }
    if (s->unixSocket) {
        unlink(s->unixSocket);
        sfree(s->unixSocket);
    }

    memset(s, 0, sizeof(struct net_service));
}

static void serviceGroupCleanup(struct net_service_group *group) {
    for (struct net_service *service = group->services; service->descr != NULL; service++) {
        cleanupService(service);
    }
    sfree(group->services);
    memset(group, 0x0, sizeof(struct net_service_group));
}

static void cleanupMessageBuffers() {

    if (Modes.decodeThreads > 1) {
        pthread_mutex_destroy(&Modes.decodeLock);
        pthread_mutex_destroy(&Modes.trackLock);
        pthread_mutex_destroy(&Modes.outputLock);

        threadpool_destroy(Modes.decodePool);
        destroy_task_group(Modes.decodeTasks);
    }

    for (int k = 0; k < Modes.decodeThreads; k++) {
        struct messageBuffer *buf = &Modes.netMessageBuffer[k];
        sfree(buf->msg);
        buf->len = 0;
        buf->alloc = 0;
    }
    sfree(Modes.netMessageBuffer);
}

void cleanupNetwork(void) {
    cleanupMessageBuffers();

    if (Modes.dump_fw) {
        zstdFwFinishFile(Modes.dump_fw);
        destroyZstdFw(Modes.dump_fw);
    }

    if (!Modes.net) {
        return;
    }
    serviceGroupCleanup(&Modes.services_out);
    serviceGroupCleanup(&Modes.services_in);

    close(Modes.net_epfd);

    for (int i = 0; i < Modes.net_connectors_count; i++) {
        struct net_connector *con = &Modes.net_connectors[i];
        if (con->gai_request_in_progress) {
            pthread_join(con->thread, NULL);
        }
        sfree(con->connect_string);
        freeaddrinfo(con->addr_info);
        pthread_mutex_destroy(&con->mutex);
    }
    sfree(Modes.net_connectors);
    sfree(Modes.net_events);

    Modes.net_connectors_count = 0;

}

char *read_uuid(struct client *c, char *p, char *eod) {
    if (c->receiverIdLocked) { // only allow the receiverId to be set once
        return p + 32;
    }

    unsigned char ch;
    char *start = p;
    uint64_t receiverId = 0;
    uint64_t receiverId2 = 0;
    // read ascii to binary
    int j = 0;
    char *breakReason = "";
    for (int i = 0; i < 128 && j < 32; i++) {
        if (p >= eod) {
            breakReason = "eod";
            break;
        }
        ch = *p++;
        //fprintf(stderr, "%c", ch);
        if (0x1A == ch) {
            breakReason = "0x1a";
            break;
        }
        if ('-' == ch || ' ' == ch) {
            continue;
        }

        unsigned char x = 0xff;

        if (ch <= 'f' && ch >= 'a') {
            x = ch - 'a' + 10;
        } else if (ch <= '9' && ch >= '0') {
            x = ch - '0';
        } else if (ch <= 'F' && ch >= 'A') {
            x = ch - 'A' + 10;
        } else {
            breakReason = "ill";
            break;
        }

        if (j < 16)
            receiverId = receiverId << 4 | x; // set 4 bits and shift them up
        else if (j < 32)
            receiverId2 = receiverId2 << 4 | x; // set 4 bits and shift them up
        j++;
    }
    int valid = j;
    if (j < 32) {
        while (j < 32) {
            if (j < 16)
                receiverId = receiverId << 4 | 0;
            else if (j < 32)
                receiverId2 = receiverId2 << 4 | 0;
            j++;
        }
        if (1 || valid > 5) {
            char uuid[64]; // needs 36 chars and null byte
            sprint_uuid(receiverId, receiverId2, uuid);
            fprintf(stderr, "read_uuid() incomplete (%s): UUID |%.*s| -> |%s|\n", breakReason, (int) imin(36, eod - start), start, uuid);
        }
    }

    if (valid >= 16) {

        c->receiverId = receiverId;
        c->receiverId2 = receiverId2;

        if (Modes.debug_uuid) {
            char uuid[64]; // needs 36 chars and null byte
            sprint_uuid(receiverId, receiverId2, uuid);
            fprintf(stderr, "reading UUID |%.*s| -> |%s|\n", (int) imin(36, eod - start), start, uuid);
            //fprintf(stderr, "ADDR %s,%s rId %016"PRIx64" UUID %.*s\n", c->host, c->port, c->receiverId, (int) imin(eod - start, 36), start);
        }
        lockReceiverId(c);
    }
    return p;
}

static void outputMessage(struct modesMessage *mm) {
    // filter messages with unwanted DF types (sbs_in are unknown DF type, filter them all, this is arbitrary but no one cares anyway)
    if (Modes.filterDF && (mm->sbs_in || !(Modes.filterDFbitset & (1 << mm->msgtype)))) {
        return;
    }
    if (!Modes.net) {
        return;
    }

    struct aircraft *ac = mm->aircraft;

    if (ac && ac->messages < Modes.net_forward_min_messages) {
        return;
    }

    int noforward = (mm->timestamp == MAGIC_NOFORWARD_TIMESTAMP) && !Modes.beast_forward_noforward;
    int64_t orig_ts = mm->timestamp;
    if (Modes.beast_set_noforward_timestamp) {
        mm->timestamp = MAGIC_NOFORWARD_TIMESTAMP;
    }

    // Suppress the first message when using an SDR
    // messages with crc 0 have an explicit checksum and are more reliable, don't suppress them when there was no CRC fix performed
    if (!mm->sbs_in
            && (Modes.net_only || Modes.net_verbatim || (ac && ac->messages > 1) || (mm->crc == 0 && mm->correctedbits == 0) || mm->msgtype == DFTYPE_MODEAC)
       ) {
        int is_mlat = (mm->source == SOURCE_MLAT);

        if (Modes.garbage_ports && (mm->garbage || mm->pos_bad) && !mm->pos_old && Modes.garbage_out.connections) {
            modesSendBeastOutput(mm, &Modes.garbage_out);
        }

        if (ac && (!Modes.sbsReduce || mm->reduce_forward)) {
            if ((!is_mlat || Modes.forward_mlat_sbs) && Modes.sbs_out.connections) {
                modesSendSBSOutput(mm, ac, &Modes.sbs_out);
            }
            if (is_mlat && Modes.sbs_out_mlat.connections) {
                modesSendSBSOutput(mm, ac, &Modes.sbs_out_mlat);
            }
        }

        if (!noforward && !is_mlat && (Modes.net_verbatim || mm->correctedbits < 2) && Modes.raw_out.connections) {
            // Forward 2-bit-corrected messages via raw output only if --net-verbatim is set
            // Don't ever forward mlat messages via raw output.
            modesSendRawOutput(mm);
        }

        if (!noforward && (!is_mlat || Modes.forward_mlat) && (mm->correctedbits < 2 || Modes.net_verbatim)) {
            // Forward 2-bit-corrected messages via beast output only if --net-verbatim is set
            // Forward mlat messages via beast output only if --forward-mlat is set
            if (Modes.beast_out.connections) {
                modesSendBeastOutput(mm, &Modes.beast_out);
            }
            if (mm->reduce_forward && Modes.beast_reduce_out.connections) {
                modesSendBeastOutput(mm, &Modes.beast_reduce_out);
            }
        }
        if (Modes.dump_fw && (!Modes.dump_reduce || mm->reduce_forward)) {
            modesDumpBeastData(mm);
        }
        if (Modes.asterix_out.connections && (!Modes.asterixReduce || mm->reduce_forward)){
            modesSendAsterixOutput(mm, &Modes.asterix_out);
        }
    }

    if (mm->jsonPositionOutputEmit && Modes.json_out.connections) {
        jsonPositionOutput(mm, ac);
    }

    if (mm->sbs_in && ac) {
        if (mm->reduce_forward || !Modes.sbsReduce) {
            if (Modes.sbs_out.connections) {
                modesSendSBSOutput(mm, ac, &Modes.sbs_out);
            }
            struct net_writer *extra_writer = NULL;
            switch(mm->source) {
                case SOURCE_SBS:
                    extra_writer = &Modes.sbs_out_replay;
                    break;
                case SOURCE_MLAT:
                    extra_writer = &Modes.sbs_out_mlat;
                    break;
                case SOURCE_JAERO:
                    extra_writer = &Modes.sbs_out_jaero;
                    break;
                case SOURCE_PRIO:
                    extra_writer = &Modes.sbs_out_prio;
                    break;

                default:
                    extra_writer = NULL;
            }
            if (extra_writer && extra_writer->connections) {
                modesSendSBSOutput(mm, ac, extra_writer);
            }
        }
    }

    mm->timestamp = orig_ts;

}

static inline int skipMessage(struct modesMessage *mm) {
    if (Modes.debug_yeet && mm->addr % 0x100 != 0xd) {
        return 1;
    }
    if (Modes.process_only != BADDR && mm->addr != Modes.process_only) {
        return 1;
    }
    if (Modes.receiver_focus && mm->receiverId != Modes.receiver_focus) {
        return 1;
    }
    return 0;
}

static void drainMessageBuffer(struct messageBuffer *buf) {
    //fprintf(stderr, "drainMessageBuffer: %d\n", buf->len);
    if (Modes.decodeThreads < 2) {
        for (int k = 0; k < buf->len; k++) {
            struct modesMessage *mm = &buf->msg[k];
            if (skipMessage(mm)) {
                continue;
            }
            trackUpdateFromMessage(mm);
        }
        for (int k = 0; k < buf->len; k++) {
            struct modesMessage *mm = &buf->msg[k];
            if (skipMessage(mm)) {
                continue;
            }
            outputMessage(mm);
        }
        buf->len = 0;
    } else {

        pthread_mutex_unlock(&Modes.decodeLock);


        sched_yield();
        //fprintf(stderr, "thread %d draining\n", buf->id);

        pthread_mutex_lock(&Modes.trackLock);
        for (int k = 0; k < buf->len; k++) {
            struct modesMessage *mm = &buf->msg[k];
            if (skipMessage(mm)) {
                continue;
            }
            trackUpdateFromMessage(mm);
        }
        pthread_mutex_unlock(&Modes.trackLock);

        pthread_mutex_lock(&Modes.outputLock);
        for (int k = 0; k < buf->len; k++) {
            struct modesMessage *mm = &buf->msg[k];
            if (skipMessage(mm)) {
                continue;
            }
            outputMessage(mm);
        }
        pthread_mutex_unlock(&Modes.outputLock);

        buf->len = 0;

        pthread_mutex_lock(&Modes.decodeLock);
        //fprintf(stderr, "thread %d drain done, back to decoding\n", buf->id);
    }
}

// get a zeroed spot in in the message buffer, only messages from this buffer may be passed to netUseMessage

struct modesMessage *netGetMM(struct messageBuffer *buf) {
    struct modesMessage *mm = &buf->msg[buf->len];
    memset(mm, 0x0, sizeof(struct modesMessage));
    mm->messageBuffer = buf;
    return mm;
}

//
//=========================================================================
//
// When a new message is available, because it was decoded from the RTL device,
// file, or received in the TCP input port, or any other way we can receive a
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//

void netUseMessage(struct modesMessage *mm) {
    struct messageBuffer *buf = mm->messageBuffer;
    if (mm != &buf->msg[buf->len]) {
        fprintf(stderr, "FATAL: fix netUseMessage / get_mm\n");
        exit(1);
    }
    buf->len++;
    if (buf->len == buf->alloc) {
        drainMessageBuffer(buf);
    }
}

void netDrainMessageBuffers() {
    for (int kt = 0; kt < Modes.decodeThreads; kt++) {
        struct messageBuffer *mb = &Modes.netMessageBuffer[kt];
        drainMessageBuffer(mb);
    }
}
